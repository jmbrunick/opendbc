"""Tip vs hold brake-cancel: ramp interceptor regen, then stock.

Short/light cancel keeps interceptor ENABLE and interpolates
accel_request 0 → stock. Held / deeper brake RELEASEs immediately.
FCW / AEB / hard lead (long still on) and full session cancel are
unchanged.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.brake_cancel_regen import (
  BRAKE_CANCEL_FIRM_A_EGO,
  BRAKE_CANCEL_RAMP_S,
  BRAKE_CANCEL_STOCK_REGEN_A,
  BRAKE_TIP_HOLD_S,
  BrakeCancelMode,
  BrakeCancelRegen,
  interpolate_regen_accel,
)
from opendbc.car.tesla.preap.carcontroller import (
  ENGAGE_GRACE_FRAMES,
  PedalCommandAction,
  PreAPLongController,
)
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES, REGEN_MAX
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.preap.tests.test_pedal_authority import (
  _activate_longitudinal,
  _decode_pedal_command,
)


DT = 0.01


def _step(fsm, **kwargs):
  defaults = dict(
    requested_long=False,
    cruise_enabled=True,
    brake_pressed=False,
    gas_pressed=False,
    a_ego=0.0,
    v_ego=15.0,
    dt=DT,
  )
  defaults.update(kwargs)
  return fsm.update(**defaults)


def test_interpolate_regen_is_linear_zero_to_stock():
  assert interpolate_regen_accel(0.0) == pytest.approx(0.0)
  assert interpolate_regen_accel(BRAKE_CANCEL_RAMP_S / 2) == pytest.approx(
    BRAKE_CANCEL_STOCK_REGEN_A / 2,
  )
  assert interpolate_regen_accel(BRAKE_CANCEL_RAMP_S) == pytest.approx(
    BRAKE_CANCEL_STOCK_REGEN_A,
  )
  assert interpolate_regen_accel(BRAKE_CANCEL_RAMP_S + 1.0) == pytest.approx(
    BRAKE_CANCEL_STOCK_REGEN_A,
  )
  assert BRAKE_CANCEL_STOCK_REGEN_A == pytest.approx(REGEN_MAX)
  assert BRAKE_CANCEL_STOCK_REGEN_A < 0.0


def test_short_brake_cancel_ramps_then_releases():
  fsm = BrakeCancelRegen()
  assert not _step(fsm, requested_long=True)

  assert _step(fsm, brake_pressed=True)
  assert fsm.mode == BrakeCancelMode.PENDING
  assert fsm.commanded_accel() == pytest.approx(0.0)

  for _ in range(10):  # 0.10 s tip
    assert _step(fsm, brake_pressed=True)
    assert fsm.commanded_accel() == pytest.approx(0.0)

  assert _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeCancelMode.RAMP
  assert fsm.commanded_accel() == pytest.approx(0.0)

  accels = [fsm.commanded_accel()]
  while fsm.ramp_s + DT + 1e-9 < BRAKE_CANCEL_RAMP_S:
    assert _step(fsm, brake_pressed=False)
    assert fsm.mode == BrakeCancelMode.RAMP
    accels.append(fsm.commanded_accel())

  assert accels[0] == pytest.approx(0.0)
  assert accels[-1] < accels[len(accels) // 2] < accels[0]
  assert accels[-1] <= BRAKE_CANCEL_STOCK_REGEN_A * 0.85
  # Not a coast plateau: most of the ramp is actually commanding regen.
  assert sum(1 for a in accels if a < -0.05) >= 3 * len(accels) // 4

  assert not _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeCancelMode.IDLE
  assert fsm.ramp_s + 1e-9 >= BRAKE_CANCEL_RAMP_S


def test_held_brake_is_firm_immediately_after_tip_window():
  fsm = BrakeCancelRegen()
  _step(fsm, requested_long=True)
  assert _step(fsm, brake_pressed=True)

  while fsm.held_s + DT + 1e-9 < BRAKE_TIP_HOLD_S:
    assert _step(fsm, brake_pressed=True)
    assert fsm.mode == BrakeCancelMode.PENDING

  assert not _step(fsm, brake_pressed=True)
  assert fsm.mode == BrakeCancelMode.FIRM
  assert not _step(fsm, brake_pressed=True)


def test_deeper_brake_is_firm_on_first_firm_aego():
  fsm = BrakeCancelRegen()
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, a_ego=BRAKE_CANCEL_FIRM_A_EGO)
  assert fsm.mode == BrakeCancelMode.FIRM


def test_full_session_cancel_does_not_keep_ramp():
  fsm = BrakeCancelRegen()
  _step(fsm, requested_long=True)
  assert not _step(fsm, cruise_enabled=False, brake_pressed=True)
  assert fsm.mode == BrakeCancelMode.IDLE


def test_fcw_hard_lead_while_long_on_is_unchanged():
  fsm = BrakeCancelRegen()
  for _ in range(5):
    assert not _step(
      fsm, requested_long=True, brake_pressed=False, a_ego=-2.0,
    )
  assert fsm.mode == BrakeCancelMode.IDLE


def test_gas_or_standstill_does_not_keep_ramp():
  fsm = BrakeCancelRegen()
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, gas_pressed=True)
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, v_ego=0.2)


def test_long_resume_clears_ramp():
  fsm = BrakeCancelRegen()
  _step(fsm, requested_long=True)
  _step(fsm, brake_pressed=True)
  _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeCancelMode.RAMP
  assert not _step(fsm, requested_long=True)
  assert fsm.mode == BrakeCancelMode.IDLE


def _pedal_conf():
  return SimpleNamespace(
    use_pedal=True,
    pedal_factor=1.0,
    di_to_pedal=lambda pedal_di: pedal_di,
    get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
  )


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: 0.0,
    update=lambda *_args, **_kwargs: None,
  )


@pytest.fixture
def controller_env(monkeypatch):
  zero_torque = _zero_torque()
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    enableJustCC=False,
    engagement=engagement,
    real_brake_pressed=False,
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    cruise_buttons=0,
    prev_cruise_buttons=0,
    pedal=feedback,
    pedal_timeout=feedback.timeout,
    pccEvent=None,
    preap_cc_cancel_needed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.4),
    longActive=False,
    orientationNED=[],
  )
  return PreAPLongController(), cc, cs, TeslaCANPreAP({})


def _drop_long_on_brake(cc, cs, *, a_ego=0.0):
  cs.enableLongControl = False
  cs.engagement.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  cs.out.aEgo = a_ego


def test_controller_short_cancel_ramps_regen(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  active = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(active[0]).enabled

  _drop_long_on_brake(cc, cs, a_ego=-0.3)
  tip = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert len(tip) == 1
  assert _decode_pedal_command(tip[0]).enabled
  assert cs.pedal_brake_cancel_ramp
  assert cs.pedal_authority_action == int(PedalCommandAction.ENABLE)
  assert controller.brake_cancel.commanded_accel() == pytest.approx(0.0)

  cs.real_brake_pressed = False
  cs.out.aEgo = 0.0
  cc.actuators.accel = -1.2  # stale planner must not punch the ramp
  accels = []
  pedal_di = []
  for frame in range(4, 20, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert _decode_pedal_command(sent[0]).enabled
    assert cs.pedal_brake_cancel_ramp
    accels.append(controller.brake_cancel.commanded_accel())
    pedal_di.append(controller.prev_pedal_di)

  assert accels[-1] < accels[0] <= 0.0
  assert accels[-1] < -0.05
  assert pedal_di[-1] < pedal_di[0]
  # Actual interceptor command must follow the ramp, not sit at coast.
  assert controller.vdas.prev_accel_effort < -0.05


def test_controller_held_brake_releases_to_stock(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=0.0)

  released = False
  for frame in range(2, 80, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if sent and not _decode_pedal_command(sent[0]).enabled:
      released = True
      assert _decode_pedal_command(sent[0]).raw_command == 0
      assert frame * 0.01 >= BRAKE_TIP_HOLD_S - 0.03
      break
    assert sent and _decode_pedal_command(sent[0]).enabled
  assert released
  assert controller.update(cc, cs, frame=frame + 2, tesla_can=tesla_can, can_bus_party=0) == []


def test_controller_firm_brake_releases_immediately(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=-2.0)
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert not getattr(cs, "pedal_brake_cancel_ramp", False)


def test_controller_hard_lead_while_long_on_still_commands_regen(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = 0.0
  for frame in range(0, ENGAGE_GRACE_FRAMES + 2, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
  cc.actuators.accel = -1.2
  cs.out.aEgo = -0.4
  limited = []
  for frame in range(ENGAGE_GRACE_FRAMES + 2, ENGAGE_GRACE_FRAMES + 24, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert _decode_pedal_command(sent[0]).enabled
    assert not getattr(cs, "pedal_brake_cancel_ramp", False)
    limited.append(controller.vdas.jerk_limiter.a_limited)
  assert min(limited) < 0.0


def test_controller_full_cancel_does_not_keep_ramp(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  cs.cruiseEnabled = False
  cs.enableLongControl = False
  cs.engagement.cruiseEnabled = False
  cs.engagement.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  cs.out.aEgo = 0.0
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert not getattr(cs, "pedal_brake_cancel_ramp", False)
