from openpilot.common.constants import CV

# Lead vehicle override: if lead is traveling above the speed limit,
# the OSM/inferred limit is likely wrong — skip capping until lead slows down.
LEAD_OVERRIDE_THRESHOLD = 0.10  # 10% above speed limit
LEAD_MIN_STATUS = True  # lead must be tracked (status=True)



def _effective_offset_percent(speed_limit_kph):
  """Tiered offset: generous at low limits (comfort), strict at high limits (tickets).

  Low speed limits (≤50 kph) are often turns, residential streets, or construction
  zones where real traffic flows well above the posted limit. Enforcement is rare.
  High speed limits are highways where speed cameras and enforcement are strict.
  """
  if speed_limit_kph <= 50:
    return 40
  elif speed_limit_kph <= 60:
    return 30
  else:
    return 10


def _lead_overrides_limit(sm, speed_limit_kph):
  """Return True if lead vehicle speed suggests the speed limit data is wrong."""
  try:
    lead = sm['radarState'].leadOne
    if not lead.status:
      return False
    # lead.vLead is absolute speed in m/s
    lead_kph = lead.vLead * CV.MS_TO_KPH
    return lead_kph > speed_limit_kph * (1 + LEAD_OVERRIDE_THRESHOLD)
  except Exception:
    return False


def on_planner_subscriptions(services):
  if 'speedLimitState' not in services:
    services.append('speedLimitState')
  return services


def on_v_cruise(v_cruise, v_ego, sm):
  if not sm.valid.get('speedLimitState', False) and sm.recv_frame.get('speedLimitState', 0) == 0:
    return v_cruise

  sls = sm['speedLimitState']
  if sls.confirmed and sls.speedLimit > 0:
    # Skip speed limit if lead vehicle is much faster — limit data likely wrong
    if _lead_overrides_limit(sm, sls.speedLimit):
      return v_cruise

    offset_pct = _effective_offset_percent(sls.speedLimit)
    v_limit = sls.speedLimit * (1 + offset_pct / 100.0) * CV.KPH_TO_MS
    if v_limit < v_cruise:
      return v_limit

  return v_cruise
