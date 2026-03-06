#!/usr/bin/env python3
"""Plugin manager daemon — discovers, loads, and monitors plugins.

Runs as an always_run process. Responsibilities:
  1. Scan /data/plugins/ for installed plugins
  2. Validate compatibility (min/max openpilot version)
  3. Load enabled plugins, register hooks + processes
  4. Spawn and monitor standalone plugin processes
  5. Serve REST API on localhost for COD
  6. Poll for enable/disable changes

Plugin processes use subprocess.Popen with os.setpgrp (like DaemonProcess
for athena) so they survive plugind restarts and are isolated from
signal propagation. PIDs are tracked on disk for reconnection.
"""
import os
import signal
import subprocess
import sys
import time

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.plugins.registry import PluginRegistry
from openpilot.selfdrive.plugins.api import set_registry, start_api_server

POLL_INTERVAL = 5.0  # seconds between checking for config changes
PID_DIR = '/data/plugins/.pids'
LOG_DIR = '/tmp/plugin_logs'
RESTART_MARKER = '/data/plugins/.needs_restart'


class PluginProcessManager:
  """Spawns and monitors standalone plugin processes.

  Uses subprocess.Popen with preexec_fn=os.setpgrp (own process group)
  so plugin processes are isolated from signals sent to plugind's group
  and survive plugind restarts. PID files enable reconnection.
  """

  def __init__(self):
    self._pids: dict[str, int] = {}
    os.makedirs(PID_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

  def _pid_file(self, name: str) -> str:
    return os.path.join(PID_DIR, f'{name}.pid')

  def _log_file(self, name: str) -> str:
    return os.path.join(LOG_DIR, f'plugin_{name}.log')

  def _is_running(self, name: str) -> int | None:
    """Check if a plugin process is still alive. Returns PID or None.

    Validates via cmdline containing plugin_{name} OR the plugin module name,
    to handle processes that execv into a native binary (e.g. mapd_runner
    exec's into /data/media/0/osm/mapd). Falls back to checking if the PID
    was started after plugind (guards against stale PIDs from prior boot).
    """
    pid = self._pids.get(name)
    if pid is None:
      try:
        pid = int(open(self._pid_file(name)).read().strip())
      except (FileNotFoundError, ValueError):
        return None

    try:
      os.kill(pid, 0)
      # Check for zombie — os.kill(pid, 0) succeeds for zombies
      try:
        with open(f'/proc/{pid}/status') as f:
          for line in f:
            if line.startswith('State:'):
              if 'Z' in line:
                # Reap the zombie
                try:
                  os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                  pass
                raise OSError("zombie process")
              break
      except FileNotFoundError:
        raise OSError("process gone")
      # Verify this PID belongs to us — check cmdline or start time
      with open(f'/proc/{pid}/cmdline') as f:
        cmdline = f.read()
      if f'plugin_{name}' in cmdline or name in cmdline:
        self._pids[name] = pid
        return pid
      # Process may have execv'd — check it started after plugind
      pid_start = os.stat(f'/proc/{pid}').st_mtime
      plugind_start = os.stat(f'/proc/{os.getpid()}').st_mtime
      if pid_start >= plugind_start:
        self._pids[name] = pid
        return pid
    except (OSError, FileNotFoundError):
      pass

    self._pids.pop(name, None)
    return None

  @staticmethod
  def _services_ready(requires) -> bool:
    """Check if required cereal services are available via shared memory.

    Args:
      requires: dict with "services" list, or empty dict/list
    """
    if not requires:
      return True
    services = requires.get('services', [])
    return all(os.path.exists(f'/dev/shm/msgq_{s}') for s in services)

  def sync(self, desired: list[dict]):
    """Start/stop processes to match desired state."""
    desired_names = {p['name'] for p in desired}

    # Stop processes no longer needed
    for name in list(self._pids):
      if name not in desired_names:
        self._stop(name)
    # Also stop orphaned PID files
    if os.path.isdir(PID_DIR):
      for f in os.listdir(PID_DIR):
        if f.endswith('.pid'):
          name = f[:-4]
          if name not in desired_names:
            self._stop(name)

    # Start processes that aren't running (respecting required services)
    for proc_def in desired:
      name = proc_def['name']
      if self._is_running(name) is not None:
        continue

      # Probe required cereal services before spawning
      requires = proc_def.get('requires', [])
      if not self._services_ready(requires):
        continue

      if name in self._pids:
        cloudlog.warning(f"plugin process '{name}' died, restarting")
      self._start(proc_def)

  def _start(self, proc_def: dict):
    name = proc_def['name']
    plugin_dir = proc_def['plugin_dir']
    module = proc_def['module']
    module_file = os.path.join(plugin_dir, *module.split('.')) + '.py'

    # Inherit parent sys.path so child can import cereal, openpilot, etc.
    parent_paths = [p for p in sys.path if p and p != plugin_dir]
    launcher_code = (
      f"import sys, os; "
      f"sys.path[:0] = {[plugin_dir] + parent_paths!r}; "
      f"os.chdir({plugin_dir!r}); "
      f"os.environ['PWD'] = {plugin_dir!r}; "
      f"from setproctitle import setproctitle; "
      f"setproctitle('plugin_{name}'); "
      f"import importlib.util; "
      f"spec = importlib.util.spec_from_file_location('plugin_{name}', {module_file!r}); "
      f"mod = importlib.util.module_from_spec(spec); "
      f"spec.loader.exec_module(mod); "
      f"mod.main()"
    )

    log_path = self._log_file(name)
    log_fd = open(log_path, 'w')
    proc = subprocess.Popen(
      [sys.executable, '-c', launcher_code],
      stdin=subprocess.DEVNULL,
      stdout=log_fd,
      stderr=log_fd,
      preexec_fn=os.setpgrp,
    )
    log_fd.close()

    self._pids[name] = proc.pid
    with open(self._pid_file(name), 'w') as f:
      f.write(str(proc.pid))

    cloudlog.info(f"plugin process '{name}' spawned (pid={proc.pid})")

  def _stop(self, name: str):
    pid = self._is_running(name)
    if pid is not None:
      try:
        os.kill(pid, signal.SIGTERM)
        for _ in range(50):
          time.sleep(0.1)
          try:
            os.kill(pid, 0)
          except OSError:
            break
        else:
          os.kill(pid, signal.SIGKILL)
      except OSError:
        pass
      cloudlog.info(f"plugin process '{name}' stopped")

    self._pids.pop(name, None)
    try:
      os.unlink(self._pid_file(name))
    except FileNotFoundError:
      pass

  def stop_all(self):
    """Stop all managed plugin processes."""
    for name in list(self._pids):
      self._stop(name)
    # Also stop any orphaned PID files
    if os.path.isdir(PID_DIR):
      for f in os.listdir(PID_DIR):
        if f.endswith('.pid'):
          self._stop(f[:-4])


def _kill_ui():
  """Kill the UI process so manager restarts it with fresh code."""
  try:
    result = subprocess.run(['pgrep', '-f', 'selfdrive.ui.ui'], capture_output=True, text=True)
    for pid_str in result.stdout.strip().split('\n'):
      if pid_str:
        os.kill(int(pid_str), signal.SIGTERM)
        cloudlog.info(f"plugind: killed UI process (pid={pid_str}) for restart")
  except Exception:
    cloudlog.exception("plugind: failed to kill UI process")


def main():
  cloudlog.info("plugind starting")

  registry = PluginRegistry()
  proc_mgr = PluginProcessManager()

  # Initial discovery
  discovered = registry.discover()
  cloudlog.info(f"plugind discovered {len(discovered)} plugins: {discovered}")

  # Load enabled plugins
  registry.load_enabled()

  # Spawn standalone plugin processes
  standalone = registry.get_standalone_processes()
  if standalone:
    cloudlog.info(f"plugind spawning {len(standalone)} standalone processes: "
                  f"{[p['name'] for p in standalone]}")
    proc_mgr.sync(standalone)

  # Start REST API for COD
  set_registry(registry)
  api_server = start_api_server()

  # Subscribe to deviceState for onroad/offroad detection
  sm = messaging.SubMaster(['deviceState'])

  # Main loop: poll for config changes, health monitoring
  # Note: no finally/stop_all — plugin processes are independent (own process
  # group) and should survive plugind restarts. They die on reboot naturally.
  rk = Ratekeeper(1.0 / POLL_INTERVAL, print_delay_threshold=None)
  while True:
    try:
      sm.update(0)

      # Re-check enabled state (user may toggle via COD or Params)
      registry.load_enabled()

      # Sync standalone processes (restart crashed ones, stop disabled ones)
      proc_mgr.sync(registry.get_standalone_processes())

      # Handle restart marker (written by install.sh after plugin updates)
      # Only restart when offroad (deviceState.started == False) for safety
      if os.path.exists(RESTART_MARKER) and not sm['deviceState'].started:
        cloudlog.warning("plugind: restart marker detected, restarting plugin processes and UI")
        os.unlink(RESTART_MARKER)
        proc_mgr.stop_all()
        # sync() will respawn with fresh code on next poll
        _kill_ui()
    except Exception:
      cloudlog.exception("plugind poll error")

    rk.keep_time()


if __name__ == "__main__":
  main()
