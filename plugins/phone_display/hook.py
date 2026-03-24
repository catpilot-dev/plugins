"""
phone_display hooks:

  webrtc.app_routes         — registers GET /health on webrtcd at startup.
  webrtc.session_started    — publishes phone_active=True on plugin bus
                               immediately when a phone WebRTC session connects.
  webrtc.session_ended      — publishes phone_active=False immediately on
                               disconnect, so watchdog reacts without waiting
                               for the next poll cycle.

  selfdrived.alert_registry — registers PhoneDisplayUnavailable alert
                               definition into EVENTS at selfdrived startup.
  selfdrived.events         — injects phoneDisplayUnavailable event name each
                               cycle when phone is required but absent.

Reads latest state from the plugin bus (published by phone_watchdog or session
hooks). Module-level state is process-local (each process has its own copy).
"""
from openpilot.common.swaglog import cloudlog


# ---------------------------------------------------------------------------
# webrtc hooks
# ---------------------------------------------------------------------------

async def _health_handler(request):
  """GET /health — returns active WebRTC session count."""
  from aiohttp import web
  streams = request.app.get('streams', {})
  return web.json_response({"status": "ok", "sessions": len(streams)})


def on_webrtc_app_routes(routes, app):
  """Register /health route on the webrtcd aiohttp app."""
  app.router.add_get("/health", _health_handler)
  cloudlog.info("phone_display: registered GET /health on webrtcd")
  return routes + ["/health"]


def _publish_session_state(phone_active: bool, session_id: str):
  """Publish phone session state on plugin bus (best-effort, non-blocking)."""
  try:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    from openpilot.common.params import Params
    from openpilot.system.hardware import HARDWARE
    required = HARDWARE.get_device_type() == 'rk3588' or Params().get_bool("CatEyePhoneRequired")
    pub = PluginPub("phone_display")
    pub.send({"required": required, "phone_active": phone_active})
    pub.close()
  except Exception as e:
    cloudlog.error("phone_display: failed to publish session state on bus: %s", e)


def on_webrtc_session_started(result, session_id):
  """Immediately signal phone connected — avoids waiting for next watchdog poll."""
  cloudlog.info("phone_display: session started (%s)", session_id)
  _publish_session_state(True, session_id)
  return result


def on_webrtc_session_ended(result, session_id):
  """Immediately signal phone disconnected — engagement blocked without delay."""
  cloudlog.info("phone_display: session ended (%s)", session_id)
  _publish_session_state(False, session_id)
  return result

# Module-level state (per-process, updated by draining the plugin bus)
_sub = None          # PluginSub instance, or _SUB_FAILED sentinel
_SUB_FAILED = object()  # set on PluginSub init failure — stops retrying

_required: bool = False
_phone_active: bool = True  # optimistic default: allow engagement until watchdog reports


def _ensure_sub() -> bool:
  """Initialise PluginSub once. Returns False (and logs) if creation failed."""
  global _sub
  if _sub is not None:
    return _sub is not _SUB_FAILED
  try:
    from openpilot.selfdrive.plugins.plugin_bus import PluginSub
    _sub = PluginSub(["phone_display"])
    cloudlog.info("phone_display hook: PluginSub connected to 'phone_display' bus")
    return True
  except Exception as e:
    cloudlog.error("phone_display hook: failed to create PluginSub — "
                   "phone state will not update: %s", e)
    _sub = _SUB_FAILED
    return False


def on_alert_registry(registrations):
  """Register PhoneDisplayUnavailable alert into selfdrived's EVENTS dict."""
  from cereal import log
  from openpilot.selfdrive.selfdrived.events import ET, NoEntryAlert, NormalPermanentAlert

  try:
    event_id = log.OnroadEvent.EventName.phoneDisplayUnavailable
  except AttributeError:
    cloudlog.error("phone_display: EventName.phoneDisplayUnavailable not found — "
                   "plugin builder may not have patched log.capnp. "
                   "Engagement block will NOT be active.")
    return registrations

  registrations[event_id] = {
    ET.NO_ENTRY: NoEntryAlert("Connect phone before engaging",
                              alert_text_1="Phone Display Required"),
    ET.PERMANENT: NormalPermanentAlert("Phone Display Required",
                                       "Connect phone before engaging"),
  }
  cloudlog.info("phone_display: registered alert for EventName.phoneDisplayUnavailable (%d)",
                event_id)
  return registrations


def on_selfdrived_events(events, CS, sm):
  """Drain plugin bus and inject phoneDisplayUnavailable if phone required but absent."""
  global _required, _phone_active

  if not _ensure_sub():
    # Already logged at creation time — don't spam on every 100Hz cycle
    return events

  # Drain all pending messages, keep latest state
  while True:
    try:
      msg = _sub.recv()
    except Exception as e:
      cloudlog.error("phone_display hook: error reading plugin bus: %s", e)
      break
    if msg is None:
      break
    _, data = msg
    _required = data.get("required", False)
    _phone_active = data.get("phone_active", True)

  if _required and not _phone_active:
    try:
      from cereal import log
      events = list(events) + [log.OnroadEvent.EventName.phoneDisplayUnavailable]
    except AttributeError:
      cloudlog.error("phone_display hook: EventName.phoneDisplayUnavailable missing — "
                     "cannot inject engagement block event")

  return events
