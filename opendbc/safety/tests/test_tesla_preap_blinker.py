#!/usr/bin/env python3
"""Panda tesla_preap blinker-turn latch — keep controls_allowed on hands-on >= 2."""
import unittest

from opendbc.safety.tests import test_tesla_preap as preap_tests

STALK_FWD_CANCEL = preap_tests.STALK_FWD_CANCEL


class TestTeslaPreAPBlinkerTurn(unittest.TestCase):
  """Latch cases only — does not re-collect TeslaPreAPTestMixin."""
  # common.CarSafetyTest.test_tx_hook_on_wrong_safety_mode imports every Test* class.
  TX_MSGS = preap_tests.TestTeslaPreAPSteeringOnly.TX_MSGS

  def setUp(self):
    self.base = preap_tests.TestTeslaPreAPSteeringOnly()
    self.base.setUp()
    self.safety = self.base.safety
    self.packer = self.base.packer

  def _rx(self, msg):
    return self.base._rx(msg)

  def _angle_meas_msg(self, *args, **kwargs):
    return self.base._angle_meas_msg(*args, **kwargs)

  def _pcm_status_msg(self, enable):
    return self.base._pcm_status_msg(enable)

  def _gear_msg(self, gear):
    return self.base._gear_msg(gear)

  def _gtw_blinker_msg(self, left=0, right=0, door_fl=0):
    values = {
      "BC_indicatorLStatus": left,
      "BC_indicatorRStatus": right,
      "DOOR_STATE_FL": door_fl,
    }
    return self.packer.make_can_msg_safety("GTW_carState", 0, values)

  def _stw_turn_msg(self, stalk=0, lever=0):
    return self.packer.make_can_msg_safety(
      "STW_ACTN_RQ", 0, {"TurnIndLvr_Stat": stalk, "SpdCtrlLvr_Stat": lever})

  def _engage_past_echo(self, t_us=1000000):
    """Engage via stalk and park the timer past the 600ms echo filter."""
    self._rx(self._pcm_status_msg(True))
    self.safety.set_timer(t_us)
    return t_us

  def test_hands_on_during_one_lamp_keeps_controls(self):
    # Justin's high-torque blinker-held turn: one GTW lamp XOR must keep
    # controls_allowed so selfdrived does not controlsMismatch after the
    # Python hold ends.
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())

  def test_hands_on_during_right_lamp_keeps_controls(self):
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(right=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_hands_on_during_flash_gap_keeps_controls(self):
    t = self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 400000)  # typical Tesla off-period ~0.3s
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(t + 800000)
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_held_stalk_keeps_controls_without_lamp(self):
    # Physical LEFT/RIGHT past 0.40s is a driver turn even if the lamp
    # bit has not arrived yet (or is in a flash gap).
    t = self._engage_past_echo()
    self._rx(self._stw_turn_msg(stalk=1))
    self.safety.set_timer(t + 400000)
    self._rx(self._stw_turn_msg(stalk=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_physical_stalk_hard_gate_before_tip_window(self):
    # blinker_turn_blocks_steering_disengage treats any LEFT/RIGHT as a
    # hard gate, including the unclassified 0.40s tip window.
    self._engage_past_echo()
    self._rx(self._stw_turn_msg(stalk=2))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_one_second_dark_ends_turn_then_hands_on_drops(self):
    t = self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.safety.set_timer(t + 10000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self.safety.set_timer(t + 10000 + 1000000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_post_turn_hand_on_keeps_then_release_allows_later_override(self):
    # High torque during the turn never drops allowed. 1s dark with the
    # wheel still lightly held (level 1) ends turn_active but keeps the
    # post-turn hold. Release, then a new hands-on >= 2 is a real override.
    t = self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=1))
    self.safety.set_timer(t + 10000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self.safety.set_timer(t + 10000 + 1000000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_hazards_do_not_keep_controls(self):
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1, right=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_hazards_after_turn_do_not_keep_latch(self):
    t = self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 100000)
    self._rx(self._gtw_blinker_msg(left=1, right=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_alc_tip_keep_alive_does_not_latch_turn(self):
    # Tip (LEFT then IDLE within 0.40s) is ALC. Leftover keep-alive
    # flashes must not become a driver-turn latch. After 1s dark, a
    # hands-on >= 2 is a real override.
    t = self._engage_past_echo()
    self._rx(self._stw_turn_msg(stalk=1))
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 200000)
    self._rx(self._stw_turn_msg(stalk=0))
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 500000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 600000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self.safety.set_timer(t + 1600000)
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_alc_same_direction_hold_1s_becomes_turn(self):
    t = self._engage_past_echo()
    self._rx(self._stw_turn_msg(stalk=1))
    self._rx(self._gtw_blinker_msg(left=1))
    self.safety.set_timer(t + 200000)
    self._rx(self._stw_turn_msg(stalk=0))
    self.safety.set_timer(t + 300000)
    self._rx(self._stw_turn_msg(stalk=1))
    self.safety.set_timer(t + 1200000)  # 0.9s same-direction: not yet a turn
    self._rx(self._stw_turn_msg(stalk=1))
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.safety.set_timer(t + 1400000)  # +1.1s from the second press
    self._rx(self._stw_turn_msg(stalk=1))
    self._rx(self._gtw_blinker_msg(left=0, right=0))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_epas_reject_during_lamp_keeps_controls(self):
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(right=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=7))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_stalk_cancel_during_blinker_turn_still_drops(self):
    t = self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(t + 100000)
    self._rx(self._stw_turn_msg(stalk=1, lever=STALK_FWD_CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_door_during_blinker_turn_still_drops(self):
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._gtw_blinker_msg(left=1, door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_gear_during_blinker_turn_still_drops(self):
    self._engage_past_echo()
    self._rx(self._gtw_blinker_msg(left=1))
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
