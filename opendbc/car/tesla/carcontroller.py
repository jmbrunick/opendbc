import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.preap.carcontroller import PreAPLongController, init_preap_can
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}

      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.preap_long = PreAPLongController()
        self.stock_cc = StockCCSpoofer()
        self.tesla_can = init_preap_can(dbc_names, self.packers)
        self.radar_vin_idx = 0
      else:
        self.tesla_can = TeslaCANRaven(self.packers)

      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return self._update_preap(CC, CS, now_nanos)

    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0:
      if self.CP.carFingerprint in LEGACY_CARS and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        cntr = (self.frame // 10) % 16
        can_sends.append(self.tesla_can.create_steering_allowed(cntr))
      elif self.CP.carFingerprint not in LEGACY_CARS:
        can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # ACC_ON / ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))
    else:
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  def _update_preap(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    lat_yield = CS.engagement.lat_yield
    # A B-edge (hands-on while lateral was yielded) refuses active steering
    # until the hands release, same as panda; type 0 frames keep flowing.
    lat_active = CC.latActive and CS.hands_on_level < 3 and not lat_yield.blocked

    if self.frame % 2 == 0:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      # Stamp what actually goes on the wire: decides A (full control, a
      # hands-on edge cancels) vs B (yielded, it does not). Panda infers the
      # same thing from these very 0x488 frames; no status bit is sent.
      # INFERENCE CONTRACT: type 1 must mean "OP is steering", and only that.
      # Any change to lateral transitions / handoff timing / a new lateral
      # release or re-arm path must update preap/lat_yield.py and
      # tesla_preap_latyield.h (see the module docstring), or a driver
      # maneuver cancels OP (or a real yank is ignored).
      lat_yield.note_steer_tx(bool(lat_active), engaged=bool(CS.engagement.cruiseEnabled))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1))

    # Reset pccEvent each tick so it expresses one-frame edge events. Without
    # this, the previous frame's value sticks (preap_long resets it, but only
    # runs in pedal mode), and the teslaCC{Engaged,Disengaged} alert
    # re-triggers indefinitely instead of fading after its 0.8s duration.
    CS.pccEvent = None

    # Pedal-mode longitudinal control. Runs only when op-long is on
    # (i.e. Comma Pedal present). May write CS.preap_cc_cancel_needed when
    # pedal mode wants to drop a running stock CC — consumed by stock_cc below.
    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.preap_long.update(CC, CS, self.frame, self.tesla_can, CANBUS.party, now_nanos))

    # Stock-CC stalk spoofs (CANCEL / SET_ACCEL). Independent of op-long —
    # the engagement FSM publishes its intent through CarState flags and the
    # spoofer is the only TX path for 0x45 STW_ACTN_RQ frames.
    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.party))
    if self.stock_cc.pcc_event:
      CS.pccEvent = self.stock_cc.pcc_event

    # Tinkla 0.6.6 donor contract: stream VIN/position/EPAS on 0x560
    # when radar is on. Empty VIN is 17 spaces (this-car passthrough);
    # position and EPAS still apply. Panda stays silent until all three
    # fragments arrive, so 10 Hz keeps that pause around 300 ms.
    if nap_conf.radar_enabled and self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_radar_vin_msg(
        self.radar_vin_idx, nap_conf.radar_donor_vin, True,
        nap_conf.radar_position, nap_conf.radar_epas_type,
      ))
      self.radar_vin_idx = (self.radar_vin_idx + 1) % 3

    # Turn-signal drive: keep the indicator flashing during the lane-change
    # arming window and maneuver. controlsd sets CC.leftBlinker/rightBlinker
    # whenever laneChangeState != off, and clears them when it returns to off
    # (so the blinker stops automatically when the maneuver completes).
    # turn: 0=none, 1=left, 2=right. Pre-AP has no AP ECU, so openpilot is the
    # sole source of DAS_bodyControls.
    if self.frame % 10 == 0:
      turn = int(CC.rightBlinker) * 2 + int(CC.leftBlinker)
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_body_controls_message(turn, 0, CANBUS.party, cntr))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
