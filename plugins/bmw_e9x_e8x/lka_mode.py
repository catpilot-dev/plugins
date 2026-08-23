"""BMW LKA mode — two-stage disengagement via the selfdrived.events_filter hook.

DCC's minimum engagement speed (30 km/h) makes intersection turns fully manual
in stock form. This filter keeps openpilot's lateral control engaged when DCC
drops (cancel press, brake press, or DCC's own min-speed cutout), so the driver
owns gas/brake while the StepperServo keeps steering:

  FULL (lat+long, DCC on) --cancel/brake/DCC drop--> LKA (lat only) --cancel--> OFF

The longitudinal path is physically inert in LKA — carcontroller only emits
stalk-emulation commands while cruiseState.enabled — but openpilot must also
stop *claiming* longitudinal authority, or checks that key off carControl
believe it is driving. See the gasPressedOverride note below.

Stage-2 detection keys on the cancel press's RISING edge: buttonCancel fires on
both press and release edges, and the stage-1 release lands after DCC has
already dropped — acting on the release would cascade straight to OFF.

On the entry side the filter clears exactly one event, resumeBlocked, on the
sub-30 stalk engage gesture; see the note in filter().

Everything else (DM escalation, doors, seatbelt, steering faults, soft
disables) passes through untouched and still fully disengages. Stripping
pedalPressed while engaged also neutralizes DisengageOnAccelerator on this car.

Brief: .superpowers/sdd/2026-08-14-bmw-lka-mode/lka-mode-brief.md
"""


# CarInterface's ret.minEnableSpeed, in m/s. Duplicated from
# bmw.values.CruiseSettings rather than imported: this module runs inside
# selfdrived, whose import graph must stay clear of the car interface.
# tests/test_lka_mode.py guards the two against drift.
MIN_ENABLE_SPEED = 30. / 3.6


class LkaModeFilter:
  def __init__(self):
    # True iff the most recent cancel rising edge occurred while already in
    # LKA (openpilot enabled, DCC off) — only such a press may disengage.
    self.cancel_press_in_lka = False

  def filter(self, events, CS, CS_prev, op_enabled):
    from cereal import car, log
    EventName = log.OnroadEvent.EventName
    ButtonType = car.CarState.ButtonEvent.Type
    GearShifter = car.CarState.GearShifter

    dcc_on = CS.cruiseState.enabled
    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel and be.pressed:
        self.cancel_press_in_lka = op_enabled and not dcc_on

    if not op_enabled:
      # Fresh-start engage block (route 418 seg 0, 2026-08-22). card.py only
      # calls initialize_v_cruise on the carControl.enabled rising edge, so
      # carState.vCruise sits at V_CRUISE_UNSET (255) until openpilot has
      # engaged once in the drive. Stock then refuses entry on any
      # accelCruise/resumeCruise event while vCruise > 250 — a guard against
      # resuming a cruise that was never set. But below minEnableSpeed this
      # port's own engage gesture IS an accelCruise release edge
      # (carstate.should_button_enable source 2), so the request and the block
      # land on the same frame and LKA can never be entered on a fresh boot.
      #
      # Nothing the guard protects is at risk here: card initializes v_cruise
      # on the enable edge either way, and LKA holds no longitudinal authority
      # at all. So clear it on exactly the frames the port calls an engage
      # gesture — DCC off, below minEnableSpeed, accel/decelCruise release —
      # and leave it standing everywhere else, including a resume press and
      # any press at DCC speeds. Every other NO_ENTRY alert passes through.
      if not dcc_on and CS.vEgo < MIN_ENABLE_SPEED and any(
          be.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not be.pressed
          for be in CS.buttonEvents):
        events.events[:] = [e for e in events.events if e != EventName.resumeBlocked]
      return  # stock entry behavior otherwise untouched

    strip = {EventName.pedalPressed}
    if not (self.cancel_press_in_lka and not dcc_on):
      strip.add(EventName.buttonCancel)

    if not dcc_on:
      # LKA holds no longitudinal authority — carcontroller gates every stalk
      # send on cruiseState.enabled — so openpilot must not claim any either.
      # ET.OVERRIDE_LONGITUDINAL is what clears CC.longActive; nothing else
      # does here, since stripping pedalPressed removed the only event that
      # dropped it on this car. Route 411 seg 5 (09:59:33): a 0.85 g driver
      # brake in LKA ran ExcessiveActuationCheck's longitudinal branch against
      # the driver's own deceleration, latching an "Excessive Actuation"
      # soft-disable that then blocked re-engagement for the rest of the drive.
      # State.overriding keeps latActive (it is in ACTIVE_STATES) and still
      # yields to USER_DISABLE, so stage-2 cancel is unaffected.
      events.add(EventName.gasPressedOverride)

    # Only Drive permits engagement (route 3fb seg 2, user ruling 2026-08-16):
    # any other definite gear — FULL or LKA — disengages directly instead of
    # wrongGear's soft-disable "Gear not D" countdown. pcmDisable =
    # USER_DISABLE with the normal disengage chime; it never occurs otherwise
    # on this car (pcmCruise=False). `unknown` (transient CAN glitch) is left
    # to the stock soft-disable so a one-frame dropout can't instantly
    # disengage. Entry-side gating is stock wrongGear NO_ENTRY, untouched.
    if CS.gearShifter not in (GearShifter.drive, GearShifter.unknown):
      strip.add(EventName.wrongGear)
      events.add(EventName.pcmDisable)

    events.events[:] = [e for e in events.events if e not in strip]


_filter = LkaModeFilter()


def on_events_filter(default, events, CS, CS_prev, op_enabled):
  """Hook callback: selfdrived.events_filter (end of SelfdriveD.update_events)."""
  _filter.filter(events, CS, CS_prev, op_enabled)
  return default
