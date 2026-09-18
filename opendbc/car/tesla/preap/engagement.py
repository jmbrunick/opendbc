from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.nap_conf import ONE_PEDAL_GAS_DI_PRESSED
from opendbc.car.tesla.values import CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type

# Echo filter windows: suppress auto-cancel echoes from spoofed stalk messages
CANCEL_ECHO_WINDOW_MS = 600
SPOOF_ECHO_WINDOW_MS = 300


class PreAPEngagement:
  """Pre-AP engagement FSM: double-pull detection, target speed, brake override, CC spoof flags."""

  def __init__(self, double_pull_enabled, double_pull_window_ms):
    self.enableDoublePull = double_pull_enabled
    self.double_pull_window_ms = double_pull_window_ms

    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.pending_enable = False

    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000

    self.pedal_speed_kph = 0.0
    self.longCtrlEvent = None
    self.pedal_unavailable = False

    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False
    self.preap_last_cc_spoof_ms = 0
    self.pending_cancel_at_ms = 0

    self.preap_brake_pressed_prev = False
    self.preap_gas_pressed_prev = False
    self._preap_one_pedal_long_was_on = False
    # One-Pedal Long: gas takeover while long was already holding
    # (foot off) stays paused until a stalk SET. Lift must not resume.
    self._one_pedal_pause_latched = False
    self._one_pedal_had_long_at_rest = False
    self._one_pedal_long_on = False
    # Stock gasPressed falling this cycle (DI crossed 2→below). Overlay
    # extra kick at DI>1 must not treat that lift-through as a tip-in.
    self._one_pedal_gas_falling_edge = False
    # Long was armed while the foot was already down (SET-while-gas /
    # engage-while-gas). Lift-to-start, not a from-rest takeover, until
    # interceptor DI is fully off.
    self._one_pedal_armed_with_gas = False
    self.last_stalk_non_cancel_ms = -10000
    self.prev_steering_disengage = False

  def _clear_one_pedal_pause_latch(self):
    self._one_pedal_pause_latched = False
    self._one_pedal_had_long_at_rest = False
    self._one_pedal_gas_falling_edge = False
    self._one_pedal_armed_with_gas = False

  def latch_one_pedal_gas_takeover(self):
    """Long was holding; driver took the pedal. Pause until SET.

    Always drop-keep-lat while the session is up so NAP's one-SET
    resume overlay can latch `_nap_long_resume_pending` even when long
    was already off (kick-miss / controller latch).
    """
    if self.cruiseEnabled:
      self._drop_longitudinal_keep_lateral()
      self._one_pedal_pause_latched = True
    return bool(self._one_pedal_pause_latched)

  def _drop_longitudinal_keep_lateral(self):
    was_long_active = self.enableLongControl
    if self.cruiseEnabled:
      self.enableLongControl = False
      self.enableJustCC = True
      self.pending_enable = False
      self.pedal_speed_kph = 0.0
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"

  def _clear_pedal_unavailable(self):
    self.pedal_unavailable = False

  def handle_pedal_unavailable(self):
    """Drop pedal longitudinal, keep lateral, and latch a publishable fault."""
    self.pedal_unavailable = True
    self._drop_longitudinal_keep_lateral()

  def handle_steering_disengage(self, steering_disengage):
    """Reset engagement on steering disengage rising edge."""
    if steering_disengage and not self.prev_steering_disengage:
      was_long_active = self.enableLongControl
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
      self.pedal_speed_kph = 0.0
      self.stalk_pull_time_ms = 0
      self.prev_stalk_pull_time_ms = -1000
      self.pending_cancel_at_ms = 0
      self._clear_pedal_unavailable()
      self._clear_one_pedal_pause_latch()
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
    self.prev_steering_disengage = steering_disengage

  def process_buttons(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                      v_ego, speed_units, use_pedal, pedal_long_allowed,
                      long_control_allowed, real_brake_pressed, di_cruise_state="OFF"):
    button_events = []
    long_control_allowed = long_control_allowed and (not use_pedal or not real_brake_pressed)

    # Stalk-spoof intent flags are single-frame events. Clear at the top so
    # downstream consumers (StockCCSpoofer) see them only on the frame they
    # are produced.
    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False

    # MAIN button: rising edge only. One SET clears a One-Pedal gas-pause
    # latch so long can come back; lift alone must not.
    if cruise_buttons == CruiseButtons.MAIN and prev_cruise_buttons != CruiseButtons.MAIN:
      self._clear_one_pedal_pause_latch()
      carlog.debug("STALK MAIN | cruiseEnabled=%s enableLong=%s pending=%s pedal=%s doublePull=%s",
                   self.cruiseEnabled, self.enableLongControl, self.pending_enable,
                   use_pedal, self.enableDoublePull)
      if self.enableDoublePull:
        self._handle_double_pull(curr_time_ms, v_ego, speed_units,
                                 use_pedal, pedal_long_allowed, long_control_allowed,
                                 di_cruise_state)
      else:
        carlog.debug("STALK single-pull engage — full control")
        self.cruiseEnabled = True
        self.pending_enable = False
        self.enableLongControl = long_control_allowed
        self.enableJustCC = not long_control_allowed
        if pedal_long_allowed and self.enableLongControl:
          self._clear_pedal_unavailable()
        if pedal_long_allowed and self.enableLongControl:
          self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
        else:
          self.pedal_speed_kph = 0.0
          if not use_pedal and di_cruise_state == "STANDBY":
            self.preap_cc_engage_needed = True
            self.preap_last_cc_spoof_ms = curr_time_ms

    if cruise_buttons != prev_cruise_buttons:
      be = self._make_button_event(cruise_buttons, prev_cruise_buttons, curr_time_ms,
                                   v_ego, speed_units, use_pedal)
      button_events.append(be)

    # Double-pull window expired
    if self.pending_enable:
      if curr_time_ms - self.stalk_pull_time_ms > self.double_pull_window_ms:
        self.pending_enable = False

    # Brake drops longitudinal while keeping lateral (pedal mode only).
    # Use the level so engagement cannot acquire pedal authority under a
    # brake that was already held before the stalk request.
    if use_pedal:
      if real_brake_pressed and self.cruiseEnabled and self.enableLongControl:
        carlog.debug("BRAKE held — dropping longitudinal")
        self._drop_longitudinal_keep_lateral()
    self.preap_brake_pressed_prev = real_brake_pressed

    return button_events

  def maybe_one_pedal_gas_kick(self, gas_pressed, one_pedal_long, interceptor_di=None):
    """Gas takeover while long was holding pauses until a stalk SET.

    When long is **already holding with the foot off**, a later
    accelerator press is a takeover: `_drop_longitudinal_keep_lateral`
    (enableLongControl off, cruiseEnabled stays, lat stays) and latch
    `_one_pedal_pause_latched`. Not a full session cancel.

    `_one_pedal_had_long_at_rest` is sticky: any gas after that rest
    latches, not only a single interceptor rising edge. Soft / late
    `gasPressed` samples and a missed edge still pause. Lift / coast /
    regen must not restore long / re-ACQUIRE. Brake pause does not set
    this latch.

    Engage-while-gas-pressed (one or two SET pulls with the foot already
    down) never sets `_one_pedal_had_long_at_rest`, so lift still runs
    A+B / A3. `_one_pedal_armed_with_gas` holds that grace through the
    lift frame (dc 13:16:14: SET→lift ~197 ms) until interceptor DI is
    fully off — do not set at-rest or overlay-pause on that falling edge.
    Same-frame SET + first gas sample is engage-with-gas.
    Do not pause a standstill wait-for-gas resume or a SET that just
    restored long. Toggle Off: gas stays OVERRIDE (`enableLongControl`
    remains true).

    Pause gas is stock `gasPressed` (DI > 2) **or** interceptor DI > 1
    when One-Pedal is On. Passing `interceptor_di` keeps lift-through
    the 1–2 deadzone as still-on-gas (not a falling then overlay extra
    rising that would latch a pause and drop enableLongControl).
    """
    gas_pressed = bool(gas_pressed)
    one_pedal_long = bool(one_pedal_long)
    self._one_pedal_long_on = one_pedal_long
    pause_gas = gas_pressed
    if one_pedal_long and interceptor_di is not None:
      try:
        pause_gas = pause_gas or (float(interceptor_di) > ONE_PEDAL_GAS_DI_PRESSED)
      except (TypeError, ValueError):
        pass
    gas_rising = pause_gas and not bool(self.preap_gas_pressed_prev)
    gas_falling = (not pause_gas) and bool(self.preap_gas_pressed_prev)
    # Overlay extra kick(True) after orig already processed stock
    # gasPressed falling this cycle is lift-through DI 1–2, not a
    # light tip-in. Do not manufacture a rising-edge takeover.
    phantom_rise = gas_rising and bool(self._one_pedal_gas_falling_edge)
    if gas_falling:
      self._one_pedal_gas_falling_edge = True
    elif not pause_gas:
      self._one_pedal_gas_falling_edge = False
    # Fully off with a known interceptor DI: overlay extra will not
    # fire this cycle, so do not leave falling_edge to block the next
    # from-rest press.
    if interceptor_di is not None and not pause_gas:
      self._one_pedal_gas_falling_edge = False
    self.preap_gas_pressed_prev = pause_gas
    long_already_on = bool(getattr(self, "_preap_one_pedal_long_was_on", False))
    paused = False
    skip_resume = bool(getattr(self, "_nap_set_resume_long", False) or getattr(
      self, "_nap_resume_wait_gas", False))

    if not self.cruiseEnabled or not one_pedal_long:
      # Session down / toggle Off: stop holding.
      self._clear_one_pedal_pause_latch()
      self._preap_one_pedal_long_was_on = bool(self.enableLongControl)
      return False

    if skip_resume:
      # SET / wait-gas is the intended resume. Clear latch only here,
      # not on a later lift frame. SET-while-gas must not immediately
      # re-latch (treat like engage-with-gas until the foot is off).
      self._clear_one_pedal_pause_latch()
      self._one_pedal_had_long_at_rest = bool(
        self.enableLongControl and not pause_gas)
      self._one_pedal_armed_with_gas = bool(
        self.enableLongControl and pause_gas)
      self._preap_one_pedal_long_was_on = bool(self.enableLongControl)
      return False

    if (
      self.enableLongControl
      and pause_gas
      and not self._one_pedal_had_long_at_rest
      and not self._one_pedal_pause_latched
    ):
      self._one_pedal_armed_with_gas = True

    if self._one_pedal_armed_with_gas:
      fully_off = not pause_gas
      if interceptor_di is not None:
        try:
          fully_off = fully_off and (float(interceptor_di) <= ONE_PEDAL_GAS_DI_PRESSED)
        except (TypeError, ValueError):
          pass
      elif pause_gas:
        fully_off = False
      else:
        # Stock falling without DI: stay armed so overlay extra True is
        # lift-through, not a tip-in. Overlay completes when DI ≤ 1.
        fully_off = False
      if fully_off:
        self._one_pedal_armed_with_gas = False
        if self.enableLongControl:
          self._one_pedal_had_long_at_rest = True
      self._preap_one_pedal_long_was_on = bool(self.enableLongControl)
      return False

    if phantom_rise:
      # Same-cycle overlay extra True. Consume the falling-edge flag so a
      # later from-rest press can still pause.
      self._one_pedal_gas_falling_edge = False
      self._preap_one_pedal_long_was_on = bool(self.enableLongControl)
      return False

    if self.enableLongControl and not pause_gas:
      self._one_pedal_had_long_at_rest = True

    takeover = pause_gas and (
      bool(self._one_pedal_had_long_at_rest)
      or (gas_rising and long_already_on and self.enableLongControl)
    )
    if takeover:
      was_long = bool(self.enableLongControl)
      carlog.debug("ONE-PEDAL LONG — gas takeover pausing longitudinal")
      self.latch_one_pedal_gas_takeover()
      paused = was_long
    elif self._one_pedal_pause_latched and self.enableLongControl:
      # Held pause: if anything restored long without SET, drop it again.
      carlog.debug("ONE-PEDAL LONG — holding gas-pause until SET")
      self._drop_longitudinal_keep_lateral()
      paused = True

    self._preap_one_pedal_long_was_on = bool(self.enableLongControl)
    return paused

  def check_can_engage(self, door_open, gear_shifter, seatbelt_unlatched):
    """Check engagement prerequisites. Resets state if blocked."""
    in_drive = gear_shifter == structs.CarState.GearShifter.drive
    can_engage = not door_open and in_drive and not seatbelt_unlatched
    if not can_engage and self.cruiseEnabled:
      carlog.debug("ENGAGE BLOCKED: door=%s gear=%s seatbelt=%s", door_open, gear_shifter, seatbelt_unlatched)
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
      self._clear_pedal_unavailable()
      self._clear_one_pedal_pause_latch()
    return can_engage

  def _handle_double_pull(self, curr_time_ms, v_ego, speed_units,
                          use_pedal, pedal_long_allowed, long_control_allowed,
                          di_cruise_state="OFF"):
    self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
    self.stalk_pull_time_ms = curr_time_ms
    double_pull = (self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms) < self.double_pull_window_ms

    if double_pull:
      self.pending_cancel_at_ms = 0
      carlog.debug("STALK double-pull (dt=%dms)", self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms)
      self.cruiseEnabled = True
      self.pending_enable = False
      self.enableLongControl = long_control_allowed
      self.enableJustCC = not long_control_allowed
      if pedal_long_allowed and self.enableLongControl:
        self._clear_pedal_unavailable()
      if pedal_long_allowed and self.enableLongControl:
        self.longCtrlEvent = "pccEnabled"
        self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
      else:
        self.pedal_speed_kph = 0.0
        # Always fire engage_needed on a no-pedal double-pull. The first pull
        # already fired an immediate cancel; even if di_cruise_state still
        # reads ENABLED at this frame (CAN lag — DI hasn't observed the cancel
        # yet), our cancel is in-flight and will land within ~100ms, dropping
        # DI to STANDBY. The spoofer's ENGAGING phase exits cleanly if DI is
        # observed ENABLED on any frame, so a no-op retry costs nothing.
        if not use_pedal:
          self.preap_cc_engage_needed = True
          self.preap_last_cc_spoof_ms = curr_time_ms
    else:
      carlog.debug("STALK first pull — lateral only (window=%dms)", self.double_pull_window_ms)
      was_long_active = self.enableLongControl
      self.cruiseEnabled = True
      self.enableLongControl = False
      self.enableJustCC = True
      self.pedal_speed_kph = 0.0
      self.pending_enable = True
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
      # No-pedal: the driver's physical MAIN pull engages stock CC at the DI
      # whenever it's armed (STANDBY → ENABLED on the same pull). Fire the
      # cancel immediately so unintended CC engagement is killed within ~100 ms
      # (CANCEL_DELAY_FRAMES + frame slot alignment in the spoofer). A
      # subsequent second pull within the window will set engage_needed and
      # the spoofer will re-engage via SET_ACCEL — visible briefly as a
      # cancel-then-engage flicker, which is the safety-favoring tradeoff.
      if not use_pedal:
        self.preap_cc_cancel_needed = True
        self.preap_last_cc_spoof_ms = curr_time_ms

  def _make_button_event(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                         v_ego, speed_units, use_pedal):
    be = structs.CarState.ButtonEvent()
    be.pressed = cruise_buttons != CruiseButtons.IDLE
    state = cruise_buttons if be.pressed else prev_cruise_buttons

    if state == CruiseButtons.MAIN:
      be.type = ButtonType.setCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms

    elif state == CruiseButtons.CANCEL:
      # Suppress auto-cancel echoes from our spoofed stalk messages
      is_echo = (
        (self.cruiseEnabled and (curr_time_ms - self.last_stalk_non_cancel_ms) < CANCEL_ECHO_WINDOW_MS)
        or ((curr_time_ms - self.preap_last_cc_spoof_ms) < SPOOF_ECHO_WINDOW_MS)
      )
      if not is_echo:
        carlog.debug("STALK CANCEL — disabling all control")
        be.type = ButtonType.cancel
        was_long_active = self.enableLongControl
        self.cruiseEnabled = False
        self.enableLongControl = False
        self.enableJustCC = False
        self.pending_enable = False
        self.pedal_speed_kph = 0.0
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        self.pending_cancel_at_ms = 0
        self._clear_pedal_unavailable()
        self._clear_one_pedal_pause_latch()
        if was_long_active:
          self.longCtrlEvent = "pccDisabled"
      else:
        be.type = ButtonType.unknown

    elif CruiseButtons.is_accel(state):
      be.type = ButtonType.accelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        # No-pedal: the DI handles speed adjust natively from the driver's
        # direct stalk message — NAP stays out. Only mutate our target when
        # we own longitudinal (pedal mode, long active).
        if use_pedal and self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          actual_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
          if state == CruiseButtons.RES_ACCEL:
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + speed_uom_kph
          else:
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + 5 * speed_uom_kph
          self.pedal_speed_kph = min(self.pedal_speed_kph, 270.0)

    elif CruiseButtons.is_decel(state):
      be.type = ButtonType.decelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        if use_pedal and self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          if state == CruiseButtons.DECEL_SET:
            self.pedal_speed_kph -= speed_uom_kph
          else:
            self.pedal_speed_kph -= 5 * speed_uom_kph
          self.pedal_speed_kph = max(self.pedal_speed_kph, 0.0)

    else:
      be.type = ButtonType.unknown

    return be

  @staticmethod
  def _capture_target_speed(v_ego, speed_units):
    speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
    current_speed_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
    return max(current_speed_kph, 0.0)
