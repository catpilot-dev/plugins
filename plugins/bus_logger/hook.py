"""bus_logger hook — health check for the bus_logger process."""
import os

_BUS_DIR = "/tmp/plugin_bus"


def _pid_alive(name: str) -> bool:
  try:
    pid = int(open(f'/data/plugins-runtime/.pids/{name}.pid').read().strip())
    os.kill(pid, 0)
    return True
  except Exception:
    return False


def on_health_check(acc, **kwargs):
  alive = _pid_alive("bus_logger")
  try:
    topics = [f for f in os.listdir(_BUS_DIR) if not f.startswith('.')] if os.path.isdir(_BUS_DIR) else []
    topic_count = len(topics)
  except Exception:
    topic_count = 0
  result = {
    "status": "ok" if alive else "warning",
    "process_alive": alive,
    "topic_count": topic_count,
  }
  if not alive:
    result["warnings"] = ["bus_logger process not running"]
  return {**acc, "bus_logger": result}
