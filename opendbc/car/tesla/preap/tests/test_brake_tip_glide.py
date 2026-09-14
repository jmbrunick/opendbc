"""Tip vs hold brake-cancel: comfort-shaped regen ramp, then stock.

A short/light cancel keeps interceptor ENABLE and ramps regen
gentle → strong over ~2.5 s (no frame-1 bite). Held / deeper brake
RELEASEs immediately. Not the reverted 0.75 s 0→REGEN_MAX fade.
FCW / AEB / hard lead (long still on) and full session cancel are
unchanged.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.brake_tip_glide import (
  BRAKE_CANCEL_FIRM_A_EGO,
  BRAKE_GLIDE_A_MIN,
  BRAKE_GLIDE_A_START,
  BRAKE_GLIDE_DURATION_S,
  BRAKE_GLIDE_V_BAND_HI,
  BRAKE_GLIDE_V_BAND_LO,
  BRAKE_GLIDE_V_TARGET,
  BRAKE_TIP_HOLD_S,
  BrakeTipGlide,
  BrakeTipGlideMode,
  comfort_ramp_accel,
  glide_end_accel,
  glide_speed_ref,
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
V_GLIDE = 20.0 * CV.MPH_TO_MS
V_HIGH = 40.0 * CV.MPH_TO_MS


def _step(fsm, **kwargs):
  defaults = dict(
    requested_long=False,
    cruise_enabled=True,
    brake_pressed=False,
    gas_pressed=False,
    a_ego=0.0,
    v_ego=V_GLIDE,
    dt=DT,
  )
  defaults.update(kwargs)
  return fsm.update(**defaults)


def test_comfort_ramp_is_gentle_then_strong_not_a_step():
  assert 2.0 <= BRAKE_GLIDE_DURATION_S <= 3.0
  assert BRAKE_GLIDE_DURATION_S != pytest.approx(0.75)
  assert BRAKE_GLIDE_V_BAND_LO < BRAKE_GLIDE_V_TARGET < BRAKE_GLIDE_V_BAND_HI
  assert BRAKE_GLIDE_A_MIN == pytest.approx(REGEN_MAX)
  assert BRAKE_GLIDE_A_MIN < BRAKE_GLIDE_A_START < -0.1

  a_end_20 = glide_end_accel(V_GLIDE)
  a_end_40 = glide_end_accel(V_HIGH)
  # 20 / 40 mph cannot hit 12 mph with an ease-in to REGEN_MAX — same shape.
  assert a_end_20 == pytest.approx(BRAKE_GLIDE_A_MIN)
  assert a_end_40 == pytest.approx(BRAKE_GLIDE_A_MIN)

  assert comfort_ramp_accel(0.0, a_end_20) == pytest.approx(BRAKE_GLIDE_A_START)
  assert comfort_ramp_accel(0.0, a_end_20) > -0.5
  # Mid is between start and end; late is fairly hard.
  mid = comfort_ramp_accel(BRAKE_GLIDE_DURATION_S / 2, a_end_20)
  late = comfort_ramp_accel(BRAKE_GLIDE_DURATION_S, a_end_20)
  assert late < mid < BRAKE_GLIDE_A_START
  assert late == pytest.approx(BRAKE_GLIDE_A_MIN)
  # Not the reverted linear 0→−1.5 / 0.75 s fade.
  at_075 = comfort_ramp_accel(0.75, a_end_20)
  assert at_075 > BRAKE_GLIDE_A_MIN * 0.85

  v_after = glide_speed_ref(V_HIGH, BRAKE_GLIDE_DURATION_S)
  assert v_after > BRAKE_GLIDE_V_BAND_HI


def test_near_band_end_accel_is_guided_by_speed_target():
  v16 = 16.0 * CV.MPH_TO_MS
  a_end = glide_end_accel(v16)
  # Reachable-ish: a_end is at or only slightly milder than full regen.
  assert a_end <= -1.0
  assert a_end >= BRAKE_GLIDE_A_MIN - 1e-9
  v_after = glide_speed_ref(v16, BRAKE_GLIDE_DURATION_S)
  assert v_after <= 16.0 * CV.MPH_TO_MS
  assert v_after + 1.0 >= BRAKE_GLIDE_V_TARGET


def test_short_brake_cancel_ramps_then_releases():
  fsm = BrakeTipGlide()
  assert not _step(fsm, requested_long=True)

  assert _step(fsm, brake_pressed=True)
  assert fsm.mode == BrakeTipGlideMode.PENDING
  # Tip cancel has started: noticeable but gentle. Not coast, not full.
  assert fsm.commanded_accel() == pytest.approx(BRAKE_GLIDE_A_START, abs=0.05)
  assert fsm.commanded_accel() > -0.5
  assert fsm.commanded_accel() < -0.1

  for _ in range(10):  # 0.10 s tip
    assert _step(fsm, brake_pressed=True)
    assert fsm.mode == BrakeTipGlideMode.PENDING
    assert fsm.commanded_accel() > -0.5

  assert _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeTipGlideMode.GLIDE
  first = fsm.commanded_accel()
  assert first == pytest.approx(comfort_ramp_accel(fsm.glide_s, fsm.a_end))
  assert first > BRAKE_GLIDE_A_MIN + 0.4

  accels = [first]
  while fsm.glide_s + DT + 1e-9 < BRAKE_GLIDE_DURATION_S:
    assert _step(fsm, brake_pressed=False, v_ego=V_GLIDE)
    assert fsm.mode == BrakeTipGlideMode.GLIDE
    accels.append(fsm.commanded_accel())

  assert accels[-1] < accels[len(accels) // 2] < accels[0]
  assert accels[-1] <= BRAKE_GLIDE_A_MIN * 0.85
  assert accels[0] > -0.5
  assert fsm.glide_s + DT + 1e-9 >= 2.0
  assert not _step(fsm, brake_pressed=False, v_ego=V_GLIDE)
  assert fsm.mode == BrakeTipGlideMode.IDLE
  assert fsm.glide_s + 1e-9 >= BRAKE_GLIDE_DURATION_S


def test_glide_ends_when_speed_reaches_target_band():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  _step(fsm, brake_pressed=True)
  _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeTipGlideMode.GLIDE

  assert _step(fsm, brake_pressed=False, v_ego=16.0 * CV.MPH_TO_MS)
  assert fsm.mode == BrakeTipGlideMode.GLIDE
  assert not _step(fsm, brake_pressed=False, v_ego=BRAKE_GLIDE_V_TARGET)
  assert fsm.mode == BrakeTipGlideMode.IDLE


def test_already_in_band_releases_instead_of_gliding():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True, v_ego=BRAKE_GLIDE_V_BAND_HI)
  assert _step(fsm, brake_pressed=True, v_ego=BRAKE_GLIDE_V_BAND_HI)
  assert fsm.mode == BrakeTipGlideMode.PENDING
  assert fsm.commanded_accel() == pytest.approx(0.0)
  assert not _step(fsm, brake_pressed=False, v_ego=14.0 * CV.MPH_TO_MS)
  assert fsm.mode == BrakeTipGlideMode.IDLE


def test_high_speed_same_shape_then_hands_off():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True, v_ego=V_HIGH)
  _step(fsm, brake_pressed=True, v_ego=V_HIGH)
  assert fsm.commanded_accel() == pytest.approx(BRAKE_GLIDE_A_START, abs=0.05)
  assert _step(fsm, brake_pressed=False, v_ego=V_HIGH)
  assert fsm.mode == BrakeTipGlideMode.GLIDE
  assert fsm.commanded_accel() > BRAKE_GLIDE_A_MIN + 0.4

  while fsm.glide_s + DT + 1e-9 < BRAKE_GLIDE_DURATION_S:
    assert _step(fsm, brake_pressed=False, v_ego=V_HIGH)
  assert fsm.commanded_accel() == pytest.approx(BRAKE_GLIDE_A_MIN, abs=0.08)
  assert not _step(fsm, brake_pressed=False, v_ego=V_HIGH)
  assert fsm.mode == BrakeTipGlideMode.IDLE


def test_held_brake_is_firm_immediately_after_tip_window():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  assert _step(fsm, brake_pressed=True)

  while fsm.held_s + DT + 1e-9 < BRAKE_TIP_HOLD_S:
    assert _step(fsm, brake_pressed=True)
    assert fsm.mode == BrakeTipGlideMode.PENDING

  assert not _step(fsm, brake_pressed=True)
  assert fsm.mode == BrakeTipGlideMode.FIRM
  assert not _step(fsm, brake_pressed=True)


def test_deeper_brake_is_firm_on_first_firm_aego():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, a_ego=BRAKE_CANCEL_FIRM_A_EGO)
  assert fsm.mode == BrakeTipGlideMode.FIRM


def test_reapply_during_glide_is_firm():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  _step(fsm, brake_pressed=True)
  _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeTipGlideMode.GLIDE
  assert not _step(fsm, brake_pressed=True, a_ego=-0.2)
  assert fsm.mode == BrakeTipGlideMode.FIRM


def test_full_session_cancel_does_not_keep_glide():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  assert not _step(fsm, cruise_enabled=False, brake_pressed=True)
  assert fsm.mode == BrakeTipGlideMode.IDLE


def test_fcw_hard_lead_while_long_on_is_unchanged():
  fsm = BrakeTipGlide()
  for _ in range(5):
    assert not _step(
      fsm, requested_long=True, brake_pressed=False, a_ego=-2.0,
    )
  assert fsm.mode == BrakeTipGlideMode.IDLE


def test_gas_or_standstill_does_not_keep_glide():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, gas_pressed=True)
  _step(fsm, requested_long=True)
  assert not _step(fsm, brake_pressed=True, v_ego=0.2)


def test_long_resume_clears_glide():
  fsm = BrakeTipGlide()
  _step(fsm, requested_long=True)
  _step(fsm, brake_pressed=True)
  _step(fsm, brake_pressed=False)
  assert fsm.mode == BrakeTipGlideMode.GLIDE
  assert not _step(fsm, requested_long=True)
  assert fsm.mode == BrakeTipGlideMode.IDLE


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
    out=SimpleNamespace(vEgo=V_GLIDE, aEgo=0.0, gasPressed=False),
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


def _drop_long_on_brake(cc, cs, *, a_ego=0.0, v_ego=V_GLIDE):
  cs.enableLongControl = False
  cs.engagement.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  cs.out.aEgo = a_ego
  cs.out.vEgo = v_ego


def _step_controller(controller, cc, cs, tesla_can, frame):
  return controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)


def test_controller_short_cancel_comfort_ramps(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  active = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(active[0]).enabled

  _drop_long_on_brake(cc, cs, a_ego=-0.3)
  tip = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert len(tip) == 1
  assert _decode_pedal_command(tip[0]).enabled
  assert cs.pedal_brake_tip_glide
  assert cs.pedal_authority_action == int(PedalCommandAction.ENABLE)
  # Frame 1 of the cancel is gentle — not stock, not REGEN_MAX.
  assert controller.brake_cancel.commanded_accel() == pytest.approx(
    BRAKE_GLIDE_A_START, abs=0.05,
  )
  assert controller.brake_cancel.commanded_accel() > -0.5

  cs.real_brake_pressed = False
  cs.out.aEgo = 0.0
  cc.actuators.accel = -1.2  # stale planner must not punch the ramp
  accels = []
  pedal_di = []
  for frame in range(3, 220):
    sent = _step_controller(controller, cc, cs, tesla_can, frame)
    if frame % 2:
      continue
    assert _decode_pedal_command(sent[0]).enabled
    assert cs.pedal_brake_tip_glide
    accels.append(controller.brake_cancel.commanded_accel())
    pedal_di.append(controller.prev_pedal_di)

  assert accels[0] > -0.5
  assert accels[-1] < accels[0]
  assert accels[-1] < -0.5
  assert pedal_di[-1] < pedal_di[0]
  assert controller.vdas.prev_accel_effort < -0.05
  assert controller.brake_cancel.glide_s > 0.75


def test_controller_glide_releases_after_window(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=-0.2)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  cs.real_brake_pressed = False
  cs.out.aEgo = 0.0

  released = False
  last_enabled_frame = 2
  for frame in range(3, 320):
    sent = _step_controller(controller, cc, cs, tesla_can, frame)
    if frame % 2:
      continue
    if sent and not _decode_pedal_command(sent[0]).enabled:
      released = True
      assert frame * 0.01 >= 2.0
      break
    assert sent and _decode_pedal_command(sent[0]).enabled
    last_enabled_frame = frame
  assert released
  assert last_enabled_frame * 0.01 > 0.75
  assert _step_controller(controller, cc, cs, tesla_can, frame + 2) == []


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
  assert not getattr(cs, "pedal_brake_tip_glide", False)


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
    assert not getattr(cs, "pedal_brake_tip_glide", False)
    limited.append(controller.vdas.jerk_limiter.a_limited)
  assert min(limited) < 0.0


def test_controller_full_cancel_does_not_keep_glide(controller_env):
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
  assert not getattr(cs, "pedal_brake_tip_glide", False)
