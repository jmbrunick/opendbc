from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, TeslaLegacyParams, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.preap.interface import get_preap_params, get_preap_accel_limits


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return get_preap_accel_limits(current_speed)
    return CarInterfaceBase.get_pid_accel_limits(CP, current_speed, cruise_speed)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    # NAP: radar_interface.py is the Bosch (Model S/X legacy) implementation; comma's Model 3/Y
    # Continental-radar support is not carried in this fork.
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    # Tesla expanded DAS_steeringControl->DAS_steeringControlType to 3 bits: first in the FSD 14 builds for HW4 around 10-26-2025,
    # then in the other HW4 builds around 03-02-2026, and for HW3 and HW2.5 with 2026.8.6 around 04-03-2026.
    # The values were pushed to the right by 1 bit and kept the same definitions, plus 4 = FSD was added.
    # These messages are only sent by the 3-bit firmware:
    # - HW3/HW4: DAS_redundantBrakingControl (0x489)
    # - HW2.5/HW3/HW4: DI_autonomyHealth (0x054)
    das_steering_3_bit = 0x489 in fingerprint[CANBUS.autopilot_party] or 0x054 in fingerprint[CANBUS.party]

    ret.dashcamOnly = (candidate in (CAR.TESLA_MODEL_X,) or  # dashcam only, pending find invalidLkasSetting signal
                       (not das_steering_3_bit and not docs))  # the car's software needs to be updated

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate == CAR.TESLA_MODEL_S_PREAP:
      return get_preap_params(ret, fingerprint)

    if candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW2,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW3,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    # Legacy Tesla ports in this tree run openpilot longitudinal by default
    # (not as an optional alpha toggle), so mark alpha availability false.
    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = True
    ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  # NOTE: Event handling is done in the main openpilot repo's controlsd/selfdrived,
  # NOT in opendbc's CarInterface. The CS.longCtrlEvent field is available for
  # the main repo to read via self.CS if needed, but we do NOT modify ret.events here.
  # The previous attempt to do so caused card to crash.
