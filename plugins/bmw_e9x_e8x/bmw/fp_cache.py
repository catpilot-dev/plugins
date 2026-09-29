"""Persistent fingerprint cache — DESIGN.md: "Fingerprint cache".

openpilot's own cache never applies to this car (no FW queries → empty carFw,
and CarParamsCache is cleared on every manager start), so without this every
boot runs the full VIN/FW sweep with OBD multiplexing toggled. A cache hit
transmits nothing; bmw.vin_check confirms the VIN once the drive is under way.
"""
import json
import os

PLUGIN_ID = 'bmw_e9x_e8x'
KEY = 'FingerprintCache'
VIN_LEN = 17

# Set when this process fingerprinted from the cache: the VIN the deferred
# check has to confirm. None after a live fingerprint.
cached_vin = None


def load():
  """The cached {'vin', 'fingerprint'}, or None if missing or malformed."""
  try:
    from config import read_plugin_param
    data = json.loads(read_plugin_param(PLUGIN_ID, KEY))
    vin, fingerprint = data['vin'], data['fingerprint']
  except Exception:
    return None
  if not isinstance(vin, str) or len(vin) != VIN_LEN or not isinstance(fingerprint, str):
    return None
  return {'vin': vin, 'fingerprint': fingerprint}


def save(vin, fingerprint):
  try:
    from config import write_plugin_param
    write_plugin_param(PLUGIN_ID, KEY, json.dumps({'vin': vin, 'fingerprint': fingerprint}))
  except Exception:
    pass


def clear():
  try:
    from config import plugin_data_dir
    (plugin_data_dir(PLUGIN_ID) / KEY).unlink(missing_ok=True)
  except Exception:
    pass


def wrap_fingerprint(orig, can_fingerprint, is_valid_vin, source_fw, models):
  """Wrap car_helpers.fingerprint (same signature and return tuple).

  Hit: return what a live VIN fingerprint of this car returns — VIN-derived
  model, no FW, source fw, fuzzy — after only the passive CAN fingerprint.
  Miss: run the live query and cache an identified BMW with a valid VIN.
  FINGERPRINT / SKIP_FW_QUERY keep their stock meaning and bypass the cache.
  """
  def fingerprint(can_recv, can_send, set_obd_multiplexing, num_pandas, cached_params):
    global cached_vin
    cache = None
    if not os.environ.get('FINGERPRINT') and not os.environ.get('SKIP_FW_QUERY'):
      cache = load()
    if cache is not None and cache['fingerprint'] in models:
      set_obd_multiplexing(False)
      can_recv()
      _, finger = can_fingerprint(can_recv)
      cached_vin = cache['vin']
      return cache['fingerprint'], finger, cache['vin'], [], source_fw, False

    cached_vin = None
    result = orig(can_recv, can_send, set_obd_multiplexing, num_pandas, cached_params)
    car_fingerprint, _, vin, _, source, _ = result
    if car_fingerprint in models and source == source_fw and is_valid_vin(vin):
      save(vin, car_fingerprint)
    return result
  return fingerprint
