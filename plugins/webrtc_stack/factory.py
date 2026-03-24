"""webrtc.session_factory hook — returns WebRTCSession as the session class."""

import os
import sys
import types

# The plugin registry loads individual hook modules without first registering
# the parent package, so relative imports fail.  Bootstrap a synthetic package
# so that `from .session import ...` resolves correctly regardless of how the
# loader registers this module.
_pkg = __name__.rsplit(".", 1)[0] if "." in __name__ else "plugins.webrtc_stack"
if _pkg not in sys.modules:
  _pkg_mod = types.ModuleType(_pkg)
  _pkg_mod.__path__ = [os.path.dirname(__file__)]
  _pkg_mod.__package__ = _pkg
  sys.modules[_pkg] = _pkg_mod

from .session import WebRTCSession


def provide_session_class(default_class):
  """Replace the default StreamSession with the plugin's WebRTCSession.

  Both are functionally equivalent; this hook exists so the canonical
  implementation lives in the plugins repo and can be installed on any
  openpilot fork that exposes the webrtc.session_factory hook point,
  without depending on teleoprtc.
  """
  # Lower webrtcd's scheduling priority so it doesn't compete with openpilot's
  # driving processes (controlsd at SCHED_FIFO, modeld, planners) under load.
  try:
    os.nice(10)
  except Exception:
    pass
  return WebRTCSession
