#!/usr/bin/env python3
"""Panda tesla_preap lateral-yield rule.

A hands-on >= 2 / EPAS-reject edge cancels (drops controls_allowed) only
while openpilot is in FULL control of lateral: an active
DAS_steeringControl (type 1) went out within 100 ms and no re-arm grace
(1.2 s after the active stream resumes from a lapse) is running (A).
Otherwise lateral was yielded to the driver (B): controls_allowed, the
cruise latch and pedal TX stay up, and active steering is refused until
the hands release.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests import test_tesla_preap as preap_tests

PREAP_FLAG_ENABLE_PEDAL = preap_tests.PREAP_FLAG_ENABLE_PEDAL
STALK_FWD_CANCEL = preap_tests.STALK_FWD_CANCEL

# Mirrors tesla_preap_latyield.h.
RECENT_US = 100000
GRACE_US = 1200000
BLOCK_CLEAR_US = 150000
T0 = 1000000


class _LatYieldMixin:
  __test__ = False
  # common.CarSafetyTest.test_tx_hook_on_wrong_safety_mode imports every Test* class.
  TX_MSGS = preap_tests.TestTeslaPreAPSteeringOnly.TX_MSGS
  BASE = preap_tests.TestTeslaPreAPSteeringOnly

  def setUp(self):
    self.base = self.BASE()
    self.base.setUp()
    self.safety = self.base.safety
    self.packer = self.base.packer

  def _rx(self, msg):
    return self.base._rx(msg)

  def _tx(self, msg):
    return self.base._tx(msg)

  cur_hands = 0

  def _hands(self, level, t_us=None, **kw):
    if t_us is not None:
      self.safety.set_timer(t_us)
    self.cur_hands = level
    self._rx(self.base._angle_meas_msg(0, hands_on_level=level, **kw))

  def _steer(self, t_us, state=1):
    """DAS_steeringControl at t_us. Returns whether panda let it out."""
    self.safety.set_timer(t_us)
    # The EPAS frame keeps flowing at the current hands level.
    self._rx(self.base._angle_meas_msg(0, hands_on_level=self.cur_hands))
    return self._tx(self.base._angle_cmd_msg(0, state, increment_timer=False))

  def _engage(self, t_us=T0):
    self.safety.set_timer(0)
    self._rx(self.base._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(t_us)
    return t_us

  def _pedal_on(self):
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    return self._tx(msg)

  def _stream(self, t0, t1, state=1, step=20000):
    ok = True
    t = t0
    while t <= t1:
      ok = self._steer(t, state) and ok
      t += step
    return ok

  # ---- A: full control, yank cancels -------------------------------

  def test_a_active_steering_then_hands_on_cancels(self):
    t = self._engage()
    self.assertTrue(self._stream(t, t + 500000))
    self._hands(2, t + 500000)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_a_epas_reject_while_steering_cancels(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._hands(0, t + 500000, eac_status=0, eac_error_code=8)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_a_hands_level_3_cancels(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._hands(3, t + 500000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_a_window_edge_just_inside_recent_cancels(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._hands(2, t + 500000 + RECENT_US)  # exactly 100 ms after the last frame
    self.assertFalse(self.safety.get_controls_allowed())

  # ---- B: lateral yielded, wheel input must not cancel --------------

  def test_b_no_lateral_ever_sent_keeps_controls(self):
    self._engage()
    self._hands(2, T0 + 300000)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())

  def test_b_window_edge_just_outside_recent_keeps_controls(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._hands(2, t + 500000 + RECENT_US + 1000)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())

  def test_b_released_lateral_type0_stream_keeps_controls(self):
    # Python yields: type 0 frames keep flowing at 50 Hz. Not active steering.
    t = self._engage()
    self._stream(t, t + 500000)
    self.assertTrue(self._stream(t + 520000, t + 900000, state=0))
    self._hands(2, t + 900000)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_b_epas_reject_while_yielded_keeps_controls(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 900000, state=0)
    self._hands(0, t + 900000, eac_status=0, eac_error_code=7)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_b_hands_level_3_while_yielded_keeps_controls(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 900000, state=0)
    self._hands(3, t + 900000)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_b_long_stays_engaged_pedal_tx_still_allowed(self):
    if self.BASE is not preap_tests.TestTeslaPreAPWithPedal:
      self.skipTest("pedal variant only")
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 900000, state=0)
    self._hands(2, t + 900000)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._pedal_on())

  def test_b_hands_edge_blocks_active_steering_until_release(self):
    t = self._engage()
    self._hands(2, t + 300000)
    self.assertTrue(self.safety.get_controls_allowed())
    # Active steering refused while the hands are on...
    self.assertFalse(self._steer(t + 320000))
    self.assertFalse(self._steer(t + 700000))
    # ...type 0 (release) always passes...
    self.assertTrue(self._steer(t + 720000, state=0))
    # ...and still refused 100 ms after hands-off (< 150 ms clear window).
    self._hands(0, t + 800000)
    self.assertFalse(self._steer(t + 800000 + BLOCK_CLEAR_US - 30000))
    # Cleared once hands have stayed off >= 150 ms.
    self._hands(0, t + 800000 + BLOCK_CLEAR_US)
    self.assertTrue(self._steer(t + 800000 + BLOCK_CLEAR_US + 20000))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_b_hands_flicker_does_not_clear_block(self):
    t = self._engage()
    self._hands(2, t + 300000)
    self._hands(0, t + 400000)
    self._hands(2, t + 450000)  # not a new edge for controls; block holds
    self.assertTrue(self.safety.get_controls_allowed())
    self._hands(0, t + 500000)
    self.assertFalse(self._steer(t + 500000 + BLOCK_CLEAR_US - 20000))

  def test_b_type1_before_edge_is_not_blocked_a_reply_after_is(self):
    # A frame that raced the edge (still inside the recent window) passes
    # before the edge; after a B edge the next one is refused.
    t = self._engage()
    self.assertTrue(self._steer(t))
    self._hands(2, t + 150000)  # 150 ms later: B
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self._steer(t + 170000))

  # ---- re-arm grace --------------------------------------------------

  def test_first_engage_has_no_grace(self):
    t = self._engage()
    self.assertTrue(self._steer(t))
    self._hands(2, t + 20000)  # first active frame ever, then a yank
    self.assertFalse(self.safety.get_controls_allowed())

  def test_grace_after_yield_makes_yank_b(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 1000000, state=0)  # yielded
    self.assertTrue(self._stream(t + 1020000, t + 1300000))  # take-back blend
    self._hands(2, t + 1300000)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self._steer(t + 1320000))

  def test_grace_expires_then_yank_is_a(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 1000000, state=0)
    resume = t + 1020000
    self._stream(resume, resume + GRACE_US + 100000)
    self._hands(2, resume + GRACE_US + 100000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_grace_just_before_expiry_is_b(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 1000000, state=0)
    resume = t + 1020000
    self._stream(resume, resume + GRACE_US - 40000)
    self._hands(2, resume + GRACE_US - 40000)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_short_lapse_under_gap_is_not_a_yield(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._steer(t + 500000 + 80000)  # one 80 ms hiccup, < 100 ms
    self._stream(t + 580000, t + 700000)
    self._hands(2, t + 700000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_new_session_resets_grace_and_history(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._stream(t + 520000, t + 1000000, state=0)
    self._stream(t + 1020000, t + 1300000)  # in grace
    # Stalk cancel ends the session...
    self.safety.set_timer(t + 1400000)
    self._rx(self.base._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    # ...a fresh engage + first lateral has no grace and no stale history.
    self.safety.set_timer(0)
    self._rx(self.base._pcm_status_msg(True))
    t2 = t + 3000000
    self._steer(t2)
    self._hands(2, t2 + 20000)
    self.assertFalse(self.safety.get_controls_allowed())

  # ---- unchanged exemptions / other cancels --------------------------

  def test_blinker_turn_still_keeps_controls_while_steering(self):
    t = self._engage()
    self._stream(t, t + 500000)
    self._rx(self.base.packer.make_can_msg_safety(
      "GTW_carState", 0, {"BC_indicatorLStatus": 1, "BC_indicatorRStatus": 0}))
    self._hands(2, t + 500000)
    self.assertTrue(self.safety.get_controls_allowed())
    # Blinker-exempt edge is not a B edge: no steering block.
    self.assertTrue(self._steer(t + 520000))

  def test_stalk_cancel_while_yielded_still_drops(self):
    t = self._engage()
    self._hands(2, t + 300000)
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(t + 900000)
    self._rx(self.base._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_gear_while_yielded_still_drops(self):
    t = self._engage()
    self._hands(2, t + 300000)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self.base._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_door_while_yielded_still_drops(self):
    t = self._engage()
    self._hands(2, t + 300000)
    self._rx(self.base._door_msg(door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_controls_off_hands_edge_is_harmless(self):
    self.safety.set_timer(T0)
    self._hands(2, T0)
    self.assertFalse(self.safety.get_controls_allowed())
    self._hands(0, T0 + 10000)
    self._rx(self.base._pcm_status_msg(False))
    self._rx(self.base._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._steer(T0 + 2000000))  # no stale block after re-engage


class TestTeslaPreAPLatYieldSteeringOnly(_LatYieldMixin, unittest.TestCase):
  __test__ = True
  BASE = preap_tests.TestTeslaPreAPSteeringOnly


class TestTeslaPreAPLatYieldWithPedal(_LatYieldMixin, unittest.TestCase):
  __test__ = True
  BASE = preap_tests.TestTeslaPreAPWithPedal


assert CarParams  # noqa: keep import (safety hooks are set by the base class)
assert PREAP_FLAG_ENABLE_PEDAL

if __name__ == "__main__":
  unittest.main()
