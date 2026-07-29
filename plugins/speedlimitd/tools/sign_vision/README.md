# sign_vision

Offline proof-of-concept for a 2-stage (YOLO11n detector + YOLO11n
classifier) pipeline that reads static Chinese speed-limit signs from
recorded C3 camera footage. This is standalone tooling: it does not touch
`speedlimitd` or run on-device. See
`docs/superpowers/specs/2026-07-16-sign-vision-poc-design.md` in the plugins
repo for the full design and phase-gate criteria.

Heavy deps (ultralytics, onnxruntime, opencv-python, av) are isolated in this
package's own `pyproject.toml` so the rest of the plugins repo test suite
stays importable without them.

## Commands

```bash
# create/refresh the isolated env (installs ultralytics -> pulls torch; slow)
uv sync

# run this package's tests
uv run pytest tests/ -q
```
