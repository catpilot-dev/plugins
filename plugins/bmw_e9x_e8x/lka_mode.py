"""BMW LKA mode — two-stage disengagement via the selfdrived.events_filter hook.

DCC's minimum engagement speed (30 km/h) makes intersection turns fully manual
in stock form. This filter keeps openpilot's lateral control engaged when DCC
drops (cancel press, brake press, or DCC's own min-speed cutout), so the driver
owns gas/brake while the StepperServo keeps steering:

  FULL (lat+long, DCC on) --cancel/brake/DCC drop--> LKA (lat only) --cancel--> OFF

The longitudinal path needs no gating: carcontroller only emits stalk-emulation
commands while cruiseState.enabled, so it is physically inert in LKA.

Stage-2 detection keys on the cancel press's RISING edge: buttonCancel fires on
both press and release edges, and the stage-1 release lands after DCC has
already dropped — acting on the release would cascade straight to OFF.

Everything else (DM escalation, doors, seatbelt, steering faults, soft
disables) passes through untouched and still fully disengages. Stripping
pedalPressed while engaged also neutralizes DisengageOnAccelerator on this car.

Brief: .superpowers/sdd/2026-08-14-bmw-lka-mode/lka-mode-brief.md
"""


class LkaModeFilter:
  def __init__(self):
    # True iff the most recent cancel rising edge occurred while already in
    # LKA (openpilot enabled, DCC off) — only such a press may disengage.
    self.cancel_press_in_lka = False

  def filter(self, events, CS, CS_prev, op_enabled):
    from cereal import car, log
    EventName = log.OnroadEvent.EventName
    ButtonType = car.CarState.ButtonEvent.Type

    dcc_on = CS.cruiseState.enabled
    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel and be.pressed:
        self.cancel_press_in_lka = op_enabled and not dcc_on

    if not op_enabled:
      return  # stock entry behavior (NO_ENTRY alerts) untouched

    strip = {EventName.pedalPressed}
    if not (self.cancel_press_in_lka and not dcc_on):
      strip.add(EventName.buttonCancel)
    events.events[:] = [e for e in events.events if e not in strip]


_filter = LkaModeFilter()


def on_events_filter(default, events, CS, CS_prev, op_enabled):
  """Hook callback: selfdrived.events_filter (end of SelfdriveD.update_events)."""
  _filter.filter(events, CS, CS_prev, op_enabled)
  return default
