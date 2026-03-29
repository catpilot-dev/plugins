"""mapd hook — health check for the mapd process."""
import os


def _pid_alive(name: str) -> bool:
  try:
    pid = int(open(f'/data/plugins-runtime/.pids/{name}.pid').read().strip())
    os.kill(pid, 0)
    return True
  except Exception:
    return False


def on_health_check(acc, **kwargs):
  alive = _pid_alive("mapd")
  result = {"status": "ok" if alive else "warning", "process_alive": alive}
  if not alive:
    result["warnings"] = ["mapd process not running"]
  return {**acc, "mapd": result}
