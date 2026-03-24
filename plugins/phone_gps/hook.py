"""
phone_gps hook — registers GET /ws/gps on webrtcd.

Phone browser connects via WebSocket and streams Geolocation API fixes.
Each fix is published as a gpsLocationExternal cereal message so loggerd
captures it alongside any on-device GPS for later comparison.

JSON payload from browser:
  {
    "latitude":         <degrees, Float64>,
    "longitude":        <degrees, Float64>,
    "altitude":         <meters above WGS84, Float64 or null>,
    "speed":            <m/s, Float32 or null>,
    "heading":          <degrees, Float32 or null>,
    "accuracy":         <horizontal accuracy metres, Float32>,
    "altitudeAccuracy": <vertical accuracy metres, Float32 or null>,
    "timestamp":        <Unix ms, Int64>
  }
"""
import json

from openpilot.common.swaglog import cloudlog


def _publish_gps(pm, data: dict) -> None:
  """Write one gpsLocationExternal message from a browser Geolocation dict."""
  import cereal.messaging as messaging

  msg = messaging.new_message('gpsLocationExternal')
  fix = msg.gpsLocationExternal

  fix.latitude          = float(data.get('latitude',  0.0))
  fix.longitude         = float(data.get('longitude', 0.0))
  fix.altitude          = float(data.get('altitude')         or 0.0)
  fix.speed             = float(data.get('speed')            or 0.0)
  fix.bearingDeg        = float(data.get('heading')          or 0.0)
  fix.horizontalAccuracy = float(data.get('accuracy',  100.0))
  fix.verticalAccuracy  = float(data.get('altitudeAccuracy') or 0.0)
  fix.unixTimestampMillis = int(data.get('timestamp', 0))
  fix.hasFix            = True   # browser only fires when it has a position
  fix.source            = 5      # SensorSource.external

  pm.send('gpsLocationExternal', msg)


async def _gps_ws_handler(request):
  """aiohttp WebSocket handler for /ws/gps — one connection per phone."""
  from aiohttp import web, WSMsgType
  import cereal.messaging as messaging

  ws = web.WebSocketResponse()
  await ws.prepare(request)

  cloudlog.info("phone_gps: phone connected to /ws/gps")
  pm = None
  try:
    pm = messaging.PubMaster(['gpsLocationExternal'])
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
          _publish_gps(pm, data)
        except Exception as e:
          cloudlog.error("phone_gps: failed to publish GPS fix: %s", e)
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
        break
  except Exception as e:
    cloudlog.error("phone_gps: WebSocket handler error: %s", e)
  finally:
    if pm is not None:
      pm.close()
    cloudlog.info("phone_gps: phone disconnected from /ws/gps")

  return ws


def on_webrtc_app_routes(routes, app):
  """Register /ws/gps WebSocket route on the webrtcd aiohttp app."""
  app.router.add_get("/ws/gps", _gps_ws_handler)
  cloudlog.info("phone_gps: registered GET /ws/gps on webrtcd")
  return routes + ["/ws/gps"]
