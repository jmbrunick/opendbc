#!/usr/bin/env python3
"""Pre-AP lateral-yield rule (opendbc side).

A hands-on >= 2 / EPAS-reject edge cancels the session only while OP is in
FULL control of lateral (A). While lateral is yielded to the driver (B) it
must not: longitudinal and the session stay, and active steering is refused
until the hands release. Mirror of tesla_preap_latyield.h.
"""
import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.preap import lat_yield as ly
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.lat_yield import LatYieldTracker


class FakeClock:
  def __init__(self, t=100.0):
    self.t = t

  def __call__(self):
    return self.t

  def advance(self, dt):
    self.t += dt


def _tracker():
  clk = FakeClock()
  return LatYieldTracker(clock=clk), clk


def _steer(tr, clk, seconds, active=True, step=0.02):
  n = int(round(seconds / step))
  for _ in range(n):
    tr.note_steer_tx(active)
    clk.advance(step)


class TestTracker(unittest.TestCase):
  def test_no_history_is_not_full_control(self):
    tr, _ = _tracker()
    self.assertFalse(tr.full_control())

  def test_first_active_frame_is_full_control_without_grace(self):
    tr, clk = _tracker()
    tr.note_steer_tx(True)
    self.assertTrue(tr.full_control())
    clk.advance(0.06)  # would be inside a grace if one had started
    tr.note_steer_tx(True)
    self.assertTrue(tr.full_control())

  def test_recent_window(self):
    tr, clk = _tracker()
    tr.note_steer_tx(True)
    clk.advance(ly.RECENT_S - 0.001)
    self.assertTrue(tr.full_control())
    clk.advance(0.002)
    self.assertFalse(tr.full_control())

  def test_release_frames_do_not_count(self):
    tr, clk = _tracker()
    _steer(tr, clk, 0.5)
    _steer(tr, clk, 0.5, active=False)
    self.assertFalse(tr.full_control())

  def test_short_lapse_is_not_a_yield(self):
    tr, clk = _tracker()
    _steer(tr, clk, 0.5)
    clk.advance(ly.GAP_S - 0.04)
    _steer(tr, clk, 0.3)
    self.assertTrue(tr.full_control())  # no grace started

  def test_resume_after_yield_starts_grace_after_delay(self):
    tr, clk = _tracker()
    _steer(tr, clk, 0.5)
    _steer(tr, clk, 0.4, active=False)  # yielded
    tr.note_steer_tx(True)  # resume
    self.assertTrue(tr.full_control())  # delay: cancel-eager for 50 ms
    clk.advance(ly.GRACE_DELAY_S + 0.005)
    tr.note_steer_tx(True)
    self.assertFalse(tr.full_control())
    # stream continues through the blend
    for _ in range(int((ly.GRACE_S - ly.GRACE_DELAY_S) / 0.02) - 3):
      clk.advance(0.02)
      tr.note_steer_tx(True)
    self.assertFalse(tr.full_control())

  def test_grace_expires(self):
    tr, clk = _tracker()
    _steer(tr, clk, 0.5)
    _steer(tr, clk, 0.4, active=False)
    _steer(tr, clk, ly.GRACE_S + 0.05)
    self.assertTrue(tr.full_control())

  def test_disengaged_resets_history_and_grace(self):
    tr, clk = _tracker()
    _steer(tr, clk, 0.5)
    _steer(tr, clk, 0.4, active=False)
    _steer(tr, clk, 0.2)  # grace running
    tr.note_steer_tx(False, engaged=False)
    self.assertFalse(tr.full_control())
    tr.note_steer_tx(True)  # fresh session, first lateral
    self.assertTrue(tr.full_control())
    clk.advance(0.1)
    tr.note_steer_tx(True)
    self.assertTrue(tr.full_control())  # no grace on a first engage

  def test_block_lifecycle(self):
    tr, clk = _tracker()
    self.assertFalse(tr.blocked)
    tr.block()
    self.assertTrue(tr.blocked)
    tr.update_block(True)
    clk.advance(5)
    tr.update_block(True)
    self.assertTrue(tr.blocked)  # hands still on
    tr.update_block(False)
    clk.advance(ly.BLOCK_CLEAR_S - 0.01)
    tr.update_block(False)
    self.assertTrue(tr.blocked)
    tr.update_block(True)  # flicker restarts the clear timer
    tr.update_block(False)
    clk.advance(ly.BLOCK_CLEAR_S - 0.01)
    tr.update_block(False)
    self.assertTrue(tr.blocked)
    clk.advance(0.02)
    tr.update_block(False)
    self.assertFalse(tr.blocked)

  def test_card_is_never_less_eager_than_panda(self):
    # Ordering contract: panda alone must never cancel.
    self.assertGreaterEqual(ly.RECENT_S, ly.PANDA_RECENT_S + 0.04)
    self.assertGreaterEqual(ly.GAP_S, ly.PANDA_GAP_S + 0.04)
    self.assertLessEqual(ly.GRACE_S, ly.PANDA_GRACE_S - 0.1)
    self.assertGreater(ly.GRACE_DELAY_S, 0.0)
    self.assertGreaterEqual(ly.BLOCK_CLEAR_S, ly.PANDA_BLOCK_CLEAR_S)
    # Grace must still cover the 1 s take-back blend from the first frame.
    self.assertGreaterEqual(ly.GRACE_S, 1.0)

  def test_constants_match_panda_header(self):
    import re
    from pathlib import Path
    hdr = (Path(ly.__file__).resolve().parents[3] / "safety/modes/tesla_preap_latyield.h").read_text()

    def us(name):
      return int(re.search(rf"#define {name}\s+(\d+)U", hdr).group(1)) / 1e6

    self.assertAlmostEqual(us("PREAP_LAT_RECENT_US"), ly.PANDA_RECENT_S)
    self.assertAlmostEqual(us("PREAP_LAT_GAP_US"), ly.PANDA_GAP_S)
    self.assertAlmostEqual(us("PREAP_LAT_GRACE_US"), ly.PANDA_GRACE_S)
    self.assertAlmostEqual(us("PREAP_LAT_BLOCK_CLEAR_US"), ly.PANDA_BLOCK_CLEAR_S)
    self.assertAlmostEqual(us("PREAP_LAT_ASSUMED_BLEND_US"), ly.PANDA_ASSUMED_BLEND_S)
    self.assertAlmostEqual(ly.PANDA_ASSUMED_BLEND_S, ly.ASSUMED_BLEND_TIME_S)

  def test_inference_contract_is_documented_where_it_is_implemented(self):
    from pathlib import Path
    root = Path(ly.__file__).resolve().parents[3]
    for rel in ("car/tesla/preap/lat_yield.py", "car/tesla/carcontroller.py",
                "safety/modes/tesla_preap_latyield.h"):
      with self.subTest(file=rel):
        self.assertIn("INFERENCE CONTRACT", (root / rel).read_text())

  def test_gap_is_shorter_than_the_shortest_yield(self):
    # A yield is at least the hands-off confirm plus one frame of type 0;
    # a gap threshold longer than that would never see the lapse.
    shortest = ly.ASSUMED_HANDS_OFF_CONFIRM_S + ly.ASSUMED_FRAME_PERIOD_S
    self.assertLess(ly.GAP_S, shortest - 0.02)
    self.assertGreaterEqual(ly.GAP_S, 3 * ly.ASSUMED_FRAME_PERIOD_S)
    self.assertGreaterEqual(ly.RECENT_S, 3 * ly.ASSUMED_FRAME_PERIOD_S)
    self.assertGreaterEqual(ly.GRACE_S, ly.ASSUMED_BLEND_TIME_S)

  def test_stash_roundtrip(self):
    for hands in range(4):
      self.assertTrue(ly.stash_full_control(ly.encode_hands_stash(hands, True)))
      v = ly.encode_hands_stash(hands, False)
      self.assertFalse(ly.stash_full_control(v))
      self.assertEqual(int(round(v)), hands)  # cs_hands_on_level still reads the level

  def test_stash_defaults_to_full_control(self):
    for v in (None, 0, 0.0, 1, 2.0, "x"):
      self.assertTrue(ly.stash_full_control(v))
    self.assertTrue(ly.stash_full_control(12.5))  # another brand's real EPS torque


class TestEngagementEdge(unittest.TestCase):
  def _eng(self):
    e = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
    e.cruiseEnabled = True
    e.enableLongControl = True
    return e

  def test_default_is_a_and_tears_down(self):
    e = self._eng()
    e.handle_steering_disengage(True)
    self.assertFalse(e.cruiseEnabled)
    self.assertFalse(e.enableLongControl)

  def test_a_tears_down(self):
    e = self._eng()
    e.handle_steering_disengage(True, True)
    self.assertFalse(e.cruiseEnabled)
    self.assertFalse(e.enableLongControl)
    self.assertFalse(e.lat_yield.blocked)

  def test_b_keeps_session_and_long_and_blocks_steering(self):
    e = self._eng()
    e.handle_steering_disengage(True, False)
    self.assertTrue(e.cruiseEnabled)
    self.assertTrue(e.enableLongControl)
    self.assertTrue(e.lat_yield.blocked)
    self.assertTrue(e.prev_steering_disengage)  # no re-edge while held

  def test_b_hold_then_release_is_not_an_edge(self):
    e = self._eng()
    e.handle_steering_disengage(True, False)
    e.handle_steering_disengage(True, True)  # lat "back" while hands still on: no edge
    self.assertTrue(e.cruiseEnabled)
    e.handle_steering_disengage(False, True)
    self.assertTrue(e.cruiseEnabled)

  def test_b_while_not_engaged_keeps_old_reset(self):
    e = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
    e.stalk_pull_time_ms = 1234
    e.handle_steering_disengage(True, False)
    self.assertEqual(e.stalk_pull_time_ms, 0)
    self.assertFalse(e.lat_yield.blocked)

  def test_new_edge_after_release_is_a_again(self):
    e = self._eng()
    e.handle_steering_disengage(True, False)
    e.handle_steering_disengage(False, True)
    e.handle_steering_disengage(True, True)
    self.assertFalse(e.cruiseEnabled)


class TestCardIntegration(unittest.TestCase):
  """Real CarInterface carstate: the edge decision follows the TX history."""

  @staticmethod
  def _epas(hands, eac_status=1, eac_error=0):
    """EPAS frame plus a closed-door Drive frame, so check_can_engage keeps the session."""
    packer = CANPacker("tesla_preap")
    pkts = []
    for msg, vals in (("DI_torque2", {"DI_gear": 4}), ("GTW_carState", {}),
                      ("EPAS_sysStatus", {"EPAS_handsOnLevel": hands, "EPAS_eacStatus": eac_status,
                                          "EPAS_eacErrorCode": eac_error})):
      addr, dat, bus = packer.make_can_msg(msg, 0, vals)
      pkts.append(CanData(addr, dat, bus))
    return [(1, pkts)]

  def _ci(self):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    CP = CarInterface.get_params("TESLA_MODEL_S_PREAP", {i: {} for i in range(8)}, [],
                                 alpha_long=False, is_release=False, docs=False)
    ci = CarInterface(CP)
    ci.update([])
    clk = FakeClock()
    ci.CS.engagement.lat_yield._clock = clk
    ci.CS.engagement.cruiseEnabled = True
    ci.CS.engagement.enableLongControl = True
    return ci, clk

  @staticmethod
  def _steer(ci, clk, seconds, lat_active=True):
    """Stand in for the carcontroller's 50 Hz stamp (the real wiring is
    tested in openpilot selfdrive/car/tesla/tests/test_preap_lat_yield.py)."""
    lat = ci.CS.engagement.lat_yield
    for _ in range(int(round(seconds / 0.02))):
      lat.note_steer_tx(bool(lat_active) and not lat.blocked, engaged=ci.CS.engagement.cruiseEnabled)
      clk.advance(0.02)

  def test_a_yank_while_steering_cancels(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    out = ci.update(self._epas(2))
    self.assertTrue(out.steeringDisengage)
    self.assertFalse(ci.CS.engagement.cruiseEnabled)

  def test_b_hands_on_while_yielded_keeps_session_and_long(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    self._steer(ci, clk, 0.4, lat_active=False)  # driver handoff / blinker pause
    out = ci.update(self._epas(2))
    self.assertTrue(out.steeringDisengage)  # the signal itself is unchanged
    self.assertTrue(ci.CS.engagement.cruiseEnabled)
    self.assertTrue(ci.CS.engagement.enableLongControl)
    self.assertFalse(ci.CS.lat_full_control)

  def test_b_blocks_active_steering_until_hands_release(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    self._steer(ci, clk, 0.4, lat_active=False)
    ci.update(self._epas(2))
    lat = ci.CS.engagement.lat_yield
    self.assertTrue(lat.blocked)
    ci.update(self._epas(2))
    clk.advance(1.0)
    ci.update(self._epas(2))
    self.assertTrue(lat.blocked)  # hands still on
    ci.update(self._epas(0))
    clk.advance(ly.BLOCK_CLEAR_S + 0.02)
    ci.update(self._epas(0))
    self.assertFalse(lat.blocked)

  def test_grace_after_resume_makes_hands_on_b(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    self._steer(ci, clk, 0.4, lat_active=False)
    self._steer(ci, clk, 0.5, lat_active=True)  # blend
    ci.update(self._epas(2))
    self.assertTrue(ci.CS.engagement.cruiseEnabled)

  def test_epas_reject_while_yielded_is_b(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    self._steer(ci, clk, 0.4, lat_active=False)
    ci.update(self._epas(0, eac_status=0, eac_error=7))
    self.assertTrue(ci.CS.engagement.cruiseEnabled)

  def test_same_direction_and_roundabout_predicates(self):
    # Angle error: command more right than the wheel, driver torque to the right.
    self.assertTrue(ly.torque_helps_op(-3.5, -40.0, -30.0))
    # Driver ahead of a lagging command: angle error opposes, not help...
    self.assertFalse(ly.torque_helps_op(-3.5, -30.0, -40.0))
    # ...unless OP is under-tracking and the torque matches the wheel.
    self.assertTrue(ly.torque_helps_op(-3.5, -30.0, -40.0, undertrack=True))
    # Opposite yank, even while under-tracking a right wheel.
    self.assertFalse(ly.torque_helps_op(3.5, -50.0, -40.0, undertrack=True))
    # Deadband is not help.
    self.assertFalse(ly.torque_helps_op(0.2, -40.0, -30.0))
    self.assertFalse(ly.torque_helps_op(-3.5, -30.5, -30.0))
    self.assertFalse(ly.hands_edge_is_yield(
      hands=0, torque_nm=-3.5, commanded_angle_deg=-40.0, measured_angle_deg=-30.0))
    self.assertTrue(ly.hands_edge_is_yield(
      hands=3, torque_nm=-3.5, commanded_angle_deg=-40.0, measured_angle_deg=-30.0))
    self.assertFalse(ly.hands_edge_is_yield(
      hands=3, torque_nm=3.5, commanded_angle_deg=-40.0, measured_angle_deg=-30.0))
    # Roundabout: any hands-on, including an opposite yank. Hands-off EPAS reject is not.
    self.assertTrue(ly.hands_edge_is_yield(
      hands=2, torque_nm=3.5, commanded_angle_deg=-40.0, measured_angle_deg=-30.0, roundabout=True))
    self.assertFalse(ly.hands_edge_is_yield(
      hands=0, torque_nm=0.0, commanded_angle_deg=-40.0, measured_angle_deg=-30.0, roundabout=True))
    self.assertTrue(ly.roundabout_yield_context(False, True, 40.0))
    self.assertTrue(ly.roundabout_yield_context(True, False, 100.0))
    self.assertFalse(ly.roundabout_yield_context(False, True, 41.0))
    self.assertEqual(ly.encode_yield_flag(True, True), bytes((0xA3,)))
    self.assertEqual(ly.encode_yield_flag(False, False), bytes((0xA0,)))

  def test_farm_torque_trace_final_yank_is_a_yield(self):
    # 14:32:18-19, one sample per frame (not held). Right turn, driver ahead
    # of the command, curvature under-tracked. Hands 0 until the last sample.
    torques = [0.70, -0.95, 0.47, -2.19, 0.86, -1.79, 0.15, -1.00, -3.29, -3.77]
    for i, tq in enumerate(torques):
      meas = -26.0 + (-44.0 + 26.0) * i / (len(torques) - 1)
      cmd = meas + 8.0  # command lags the wheel
      hands = 3 if i == len(torques) - 1 else 0
      got = ly.hands_edge_is_yield(
        hands=hands, torque_nm=tq, commanded_angle_deg=cmd, measured_angle_deg=meas,
        undertrack=True)
      self.assertEqual(got, i == len(torques) - 1)
    # Opposite final yank (positive torque, command still to the right of the wheel).
    self.assertFalse(ly.hands_edge_is_yield(
      hands=3, torque_nm=3.77, commanded_angle_deg=-50.0, measured_angle_deg=-40.0, undertrack=True))

  def test_not_engaged_resets_history(self):
    ci, clk = self._ci()
    self._steer(ci, clk, 0.5)
    ci.CS.engagement.cruiseEnabled = False
    self._steer(ci, clk, 0.04)
    self.assertFalse(ci.CS.engagement.lat_yield.full_control())


if __name__ == "__main__":
  unittest.main()
