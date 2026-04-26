"""Real-time drive statistics tracker.

Accumulates distance, duration, and engagement during driving from cereal
messages already subscribed by the UI process. Writes a summary JSON on
offroad transition so the home screen and COD can display results instantly
without parsing qlogs.

Samples on deviceState updates (~2Hz) — matching qlog resolution exactly.
"""
import json
import math
import os
import time
from config import PLUGINS_RUNTIME_DIR

LAST_DRIVE_FILE = os.path.join(PLUGINS_RUNTIME_DIR, '.last_drive.json')
MIN_TRACE_DIST_M = 50  # minimum distance between trace points


class DriveTracker:
  """Lightweight drive stats accumulator, gated on deviceState (2Hz)."""

  def __init__(self):
    self._active = False
    self._last_tick = 0.0

    # Accumulators
    self._distance_m = 0.0
    self._duration_s = 0.0
    self._engaged_s = 0.0
    self._start_time = 0.0
    self._start_lat = 0.0
    self._start_lng = 0.0
    self._end_lat = 0.0
    self._end_lng = 0.0
    self._has_gps = False
    self._trace = []  # [[lat, lng], ...]

    # Register for onroad/offroad transitions
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.add_offroad_transition_callback(self._on_transition)

  def _on_transition(self):
    from openpilot.selfdrive.ui.ui_state import ui_state
    if ui_state.started:
      self._reset()
    else:
      self._save()

  def _reset(self):
    self._distance_m = 0.0
    self._duration_s = 0.0
    self._engaged_s = 0.0
    self._start_time = time.time()
    self._start_lat = 0.0
    self._start_lng = 0.0
    self._end_lat = 0.0
    self._end_lng = 0.0
    self._has_gps = False
    self._trace = []
    self._last_tick = time.monotonic()
    self._active = True

  def tick(self, sm):
    if not self._active or not sm.updated.get('deviceState', False):
      return

    now = time.monotonic()
    dt = now - self._last_tick
    self._last_tick = now

    # Clamp dt to avoid spikes from process pauses
    if dt > 2.0:
      dt = 0.5

    v_ego = sm['carState'].vEgo
    self._distance_m += v_ego * dt
    self._duration_s += dt

    if sm['selfdriveState'].enabled:
      self._engaged_s += dt

    # GPS: capture start once, continuously update end, accumulate trace
    if sm.updated.get('gpsLocationExternal', False):
      gps = sm['gpsLocationExternal']
      if getattr(gps, 'flags', 0) & 1:
        lat, lng = gps.latitude, gps.longitude
        if not self._has_gps:
          self._start_lat = lat
          self._start_lng = lng
          self._has_gps = True
          self._trace.append([lat, lng])
        elif self._far_enough(lat, lng):
          self._trace.append([lat, lng])
        self._end_lat = lat
        self._end_lng = lng

  def _far_enough(self, lat, lng):
    """Check if point is at least MIN_TRACE_DIST_M from the last trace point."""
    if not self._trace:
      return True
    prev_lat, prev_lng = self._trace[-1]
    dlat = (lat - prev_lat) * 111320
    dlng = (lng - prev_lng) * 111320 * math.cos(math.radians(prev_lat))
    return dlat * dlat + dlng * dlng > MIN_TRACE_DIST_M * MIN_TRACE_DIST_M

  def _save(self):
    self._active = False
    if self._duration_s < 5.0 or self._distance_m < 100.0:
      return

    # Preserve the previous drive's GPS trace if this drive had no GPS lock.
    trace = self._trace
    has_gps = self._has_gps
    start_lat, start_lng = self._start_lat, self._start_lng
    end_lat, end_lng = self._end_lat, self._end_lng
    if not has_gps:
      prev = get_last_drive()
      if prev and prev.get('has_gps') and prev.get('trace'):
        trace = prev['trace']
        has_gps = prev['has_gps']
        start_lat = prev.get('start_lat', 0.0)
        start_lng = prev.get('start_lng', 0.0)
        end_lat = prev.get('end_lat', 0.0)
        end_lng = prev.get('end_lng', 0.0)

    data = {
      'version': 1,
      'start_time': self._start_time,
      'duration_s': round(self._duration_s, 1),
      'distance_m': round(self._distance_m, 1),
      'engaged_s': round(self._engaged_s, 1),
      'start_lat': start_lat,
      'start_lng': start_lng,
      'end_lat': end_lat,
      'end_lng': end_lng,
      'has_gps': has_gps,
      'trace': trace,
    }

    try:
      tmp = LAST_DRIVE_FILE + '.tmp'
      with open(tmp, 'w') as f:
        json.dump(data, f)
      os.replace(tmp, LAST_DRIVE_FILE)
    except OSError:
      pass

  @property
  def summary(self):
    """Current accumulated stats (for live display if needed)."""
    if not self._active:
      return None
    return {
      'distance_m': self._distance_m,
      'duration_s': self._duration_s,
      'engaged_s': self._engaged_s,
    }


def get_last_drive() -> dict | None:
  """Read the last drive summary. Returns None if unavailable."""
  try:
    with open(LAST_DRIVE_FILE) as f:
      return json.load(f)
  except (OSError, json.JSONDecodeError):
    return None
