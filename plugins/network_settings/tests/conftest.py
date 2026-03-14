"""Add plugin dir to sys.path so bare imports (params_helper, proxy, etc.) resolve.

On-device, the plugin runner adds each plugin's directory to sys.path.
In tests, we replicate that here.
"""
import sys
from pathlib import Path

plugin_dir = str(Path(__file__).resolve().parent.parent)
if plugin_dir not in sys.path:
  sys.path.insert(0, plugin_dir)
