"""mapd hook — health check for the mapd process.

Dormancy-aware since 2026-08-19. The plugin is deliberately kept installed with
its cereal interface warm (slots 17-19 + the mapdOut service) while declaring
NO process, so the Go binary never launches — see README. A plain
"process not running" warning would then fire every plugind poll forever, and a
permanent expected warning is worse than no warning: it trains the reader to
ignore mapd health, so a REAL failure after re-activation would not stand out.

So the manifest is the source of truth for what "healthy" means here: no
declared process ⇒ dormant is the correct state ⇒ report ok. Restoring the
processes entry re-arms the warning automatically, with no edit to this file.
"""
import json
import os

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))


def _pid_alive(name: str) -> bool:
  try:
    pid = int(open(f'/data/plugins-runtime/.pids/{name}.pid').read().strip())
    os.kill(pid, 0)
    return True
  except Exception:
    return False


def _declares_process() -> bool:
  """True when the manifest declares a mapd process for plugind to spawn.

  On a read/parse failure, assume a process IS expected: that is the state
  where a missing binary is a real problem worth warning about, so the failure
  mode of this helper stays the loud one.
  """
  try:
    with open(os.path.join(_PLUGIN_DIR, 'plugin.json')) as f:
      procs = json.load(f).get('processes', [])
    return any(p.get('name') == 'mapd' for p in procs)
  except Exception:
    return True


def on_health_check(acc, **kwargs):
  alive = _pid_alive("mapd")
  expected = _declares_process()

  if not expected:
    # Dormant by design. Report process_alive honestly so the field keeps its
    # meaning, and flag the dormancy so an rlog reader can tell "switched off"
    # from "crashed" without consulting the manifest.
    return {**acc, "mapd": {"status": "ok", "process_alive": alive, "dormant": True}}

  result = {"status": "ok" if alive else "warning", "process_alive": alive, "dormant": False}
  if not alive:
    result["warnings"] = ["mapd process not running"]
  return {**acc, "mapd": result}
