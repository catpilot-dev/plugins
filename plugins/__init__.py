# Deliberately present so `plugins` is a REGULAR package, not a PEP 420
# namespace package. Without this, any stray sys.path entry (e.g. a leaked
# PYTHONPATH pointing at another worktree) that contains a regular `plugins`
# package silently hijacks every `plugins.*` test import — the suite then
# tests that other checkout's code. Dev-machine only: install.sh copies
# plugins/*/ directories, so this file never reaches the device.
