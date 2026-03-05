#!/usr/bin/env python3
"""Plugin manager daemon — discovers, loads, and monitors plugins.

Runs as an always_run process. Responsibilities:
  1. Scan /data/plugins/ for installed plugins
  2. Validate compatibility (min/max openpilot version)
  3. Load enabled plugins, register hooks + processes
  4. Spawn and monitor standalone plugin processes
  5. Serve REST API on localhost for COD
  6. Poll for enable/disable changes
"""
import importlib
import importlib.util
import multiprocessing
import os
import sys

from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.plugins.registry import PluginRegistry
from openpilot.selfdrive.plugins.api import set_registry, start_api_server

POLL_INTERVAL = 5.0  # seconds between checking for config changes


def _run_plugin_process(plugin_dir: str, module_name: str, proc_name: str):
  """Entry point for spawned plugin processes."""
  if plugin_dir not in sys.path:
    sys.path.insert(0, plugin_dir)

  cloudlog.info(f"plugin process '{proc_name}' starting from {plugin_dir}/{module_name}")

  module_file = os.path.join(plugin_dir, *module_name.split('.')) + '.py'
  spec = importlib.util.spec_from_file_location(f"plugin_proc_{proc_name}", module_file)
  if spec is None or spec.loader is None:
    cloudlog.error(f"plugin process '{proc_name}': cannot find {module_file}")
    return

  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)

  main_fn = getattr(module, 'main', None)
  if main_fn is None:
    cloudlog.error(f"plugin process '{proc_name}': no main() function in {module_name}")
    return

  main_fn()


class PluginProcessManager:
  """Spawns and monitors standalone plugin processes."""

  def __init__(self):
    self._procs: dict[str, multiprocessing.Process] = {}

  def sync(self, desired: list[dict]):
    """Start/stop processes to match desired state."""
    desired_names = {p['name'] for p in desired}

    # Stop processes no longer needed
    for name in list(self._procs):
      if name not in desired_names:
        self._stop(name)

    # Start or restart processes
    for proc_def in desired:
      name = proc_def['name']
      proc = self._procs.get(name)
      if proc is not None and proc.is_alive():
        continue
      if proc is not None and not proc.is_alive():
        cloudlog.warning(f"plugin process '{name}' died (exit={proc.exitcode}), restarting")
      self._start(proc_def)

  def _start(self, proc_def: dict):
    name = proc_def['name']
    p = multiprocessing.Process(
      target=_run_plugin_process,
      args=(proc_def['plugin_dir'], proc_def['module'], name),
      name=f"plugin_{name}",
      daemon=True,
    )
    p.start()
    self._procs[name] = p
    cloudlog.info(f"plugin process '{name}' spawned (pid={p.pid})")

  def _stop(self, name: str):
    proc = self._procs.pop(name, None)
    if proc is not None and proc.is_alive():
      proc.terminate()
      proc.join(timeout=5)
      if proc.is_alive():
        proc.kill()
      cloudlog.info(f"plugin process '{name}' stopped")

  def stop_all(self):
    for name in list(self._procs):
      self._stop(name)


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

  # Main loop: poll for config changes, health monitoring
  rk = Ratekeeper(1.0 / POLL_INTERVAL, print_delay_threshold=None)
  try:
    while True:
      try:
        # Re-check enabled state (user may toggle via COD or Params)
        registry.load_enabled()

        # Sync standalone processes (restart crashed ones, stop disabled ones)
        proc_mgr.sync(registry.get_standalone_processes())
      except Exception:
        cloudlog.exception("plugind poll error")

      rk.keep_time()
  finally:
    proc_mgr.stop_all()


if __name__ == "__main__":
  main()
