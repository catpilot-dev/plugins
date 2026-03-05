#!/usr/bin/env bash
#
# install.sh — Install openpilot-plugins onto a device or local openpilot tree.
#
# Three operations:
#   1. Overlay overlays/selfdrive/ → openpilot tree (framework + UI modules)
#   2. Overlay overlays/cereal/ → openpilot tree (schema extensions)
#   3. Copy each plugins/*/ → /data/plugins/ (runtime plugin packages)
#
# Usage:
#   bash install.sh                  # Auto-detect openpilot location
#   bash install.sh --dry-run        # Preview actions without writing
#   bash install.sh --target /path   # Specify openpilot root
#   bash install.sh --help

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRY_RUN=false
OPENPILOT_ROOT=""
PLUGINS_DEST="/data/plugins"

usage() {
  echo "Usage: install.sh [OPTIONS]"
  echo ""
  echo "Install openpilot-plugins framework and plugin packages."
  echo ""
  echo "Options:"
  echo "  --dry-run        Preview actions without writing files"
  echo "  --target PATH    Specify openpilot root directory"
  echo "  --plugins-dir P  Specify plugins destination (default: /data/plugins)"
  echo "  --help           Show this help"
  exit 0
}

log()  { echo "[install] $*"; }
warn() { echo "[install] WARNING: $*" >&2; }
err()  { echo "[install] ERROR: $*" >&2; exit 1; }

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)     DRY_RUN=true; shift ;;
    --target)      OPENPILOT_ROOT="$2"; shift 2 ;;
    --plugins-dir) PLUGINS_DEST="$2"; shift 2 ;;
    --help|-h)     usage ;;
    *)             err "Unknown option: $1" ;;
  esac
done

# Detect openpilot root
if [[ -z "$OPENPILOT_ROOT" ]]; then
  if [[ -d /data/openpilot/selfdrive ]]; then
    OPENPILOT_ROOT="/data/openpilot"
  elif [[ -n "${OPENPILOT_ROOT:-}" ]]; then
    : # already set via env
  elif [[ -d "$HOME/openpilot/selfdrive" ]]; then
    OPENPILOT_ROOT="$HOME/openpilot"
  else
    err "Cannot find openpilot. Use --target /path/to/openpilot"
  fi
fi

if [[ ! -d "$OPENPILOT_ROOT/selfdrive" ]]; then
  err "Not a valid openpilot tree: $OPENPILOT_ROOT (missing selfdrive/)"
fi

log "openpilot root: $OPENPILOT_ROOT"
log "plugins dest:   $PLUGINS_DEST"
$DRY_RUN && log "DRY RUN — no files will be written"

# --- Hook-site verification ---
verify_hooks() {
  local missing=0

  # Check for hooks import in key control files
  local hooks_import="from openpilot.selfdrive.plugins.hooks import hooks"

  for file in \
    "$OPENPILOT_ROOT/selfdrive/controls/controlsd.py" \
    "$OPENPILOT_ROOT/selfdrive/controls/lib/longitudinal_planner.py" \
    "$OPENPILOT_ROOT/system/manager/manager.py"; do
    if [[ -f "$file" ]]; then
      if ! grep -q "selfdrive.plugins" "$file" 2>/dev/null; then
        warn "Hook call site missing in: ${file#$OPENPILOT_ROOT/}"
        missing=$((missing + 1))
      fi
    else
      warn "File not found: ${file#$OPENPILOT_ROOT/}"
      missing=$((missing + 1))
    fi
  done

  if [[ $missing -gt 0 ]]; then
    warn ""
    warn "Your openpilot tree is missing $missing hook call sites."
    warn "Plugins will install but won't activate until hook call sites"
    warn "are added to controlsd.py, longitudinal_planner.py, and manager.py."
    warn "See docs/HOOK_INTEGRATION_POINTS.md for details."
    warn ""
  fi
}

verify_hooks

# --- 1. Overlay framework modules ---
overlay_framework() {
  local src="$SCRIPT_DIR/overlays/selfdrive/plugins"
  local dst="$OPENPILOT_ROOT/selfdrive/plugins"

  log "Overlaying framework: selfdrive/plugins/"

  if $DRY_RUN; then
    find "$src" -type f -name '*.py' | while read -r f; do
      echo "  COPY ${f#$SCRIPT_DIR/} → ${dst#$OPENPILOT_ROOT/}/${f#$src/}"
    done
    return
  fi

  mkdir -p "$dst"
  mkdir -p "$dst/tests"
  cp -v "$src"/*.py "$dst/" 2>/dev/null || true
  if [[ -d "$src/tests" ]]; then
    cp -v "$src/tests"/*.py "$dst/tests/" 2>/dev/null || true
  fi
}

# --- 1b. Overlay UI modules ---
overlay_ui() {
  local src="$SCRIPT_DIR/overlays/selfdrive/ui"
  if [[ ! -d "$src" ]]; then return; fi
  local dst="$OPENPILOT_ROOT/selfdrive/ui"

  log "Overlaying UI: selfdrive/ui/"

  if $DRY_RUN; then
    find "$src" -type f -name '*.py' | while read -r f; do
      echo "  COPY ${f#$SCRIPT_DIR/} → ${dst#$OPENPILOT_ROOT/}/${f##$src/}"
    done
    return
  fi

  # Recursively copy all .py files, preserving directory structure
  find "$src" -type f -name '*.py' | while read -r f; do
    local rel="${f#$src/}"
    local target_dir="$dst/$(dirname "$rel")"
    mkdir -p "$target_dir"
    cp -v "$f" "$target_dir/" 2>/dev/null || true
  done
}

# --- 2. Overlay cereal schema + inject plugin schemas/services ---
overlay_cereal() {
  local cereal_dir="$OPENPILOT_ROOT/cereal"

  # Copy base custom.capnp overlay (stock template with reserved slots)
  local src="$SCRIPT_DIR/overlays/cereal/custom.capnp"
  if [[ -f "$src" ]] && [[ -d "$cereal_dir" ]]; then
    if $DRY_RUN; then
      echo "  COPY overlays/cereal/custom.capnp → cereal/custom.capnp"
    else
      cp -v "$src" "$cereal_dir/custom.capnp"
    fi
  fi

  $DRY_RUN && return

  # Inject plugin cereal schemas (custom.capnp + log.capnp)
  local capnp_script="$SCRIPT_DIR/plugins/custom_capnp.py"
  if [[ -f "$capnp_script" ]] && [[ -d "$cereal_dir" ]]; then
    python3 "$capnp_script" "$cereal_dir" "$PLUGINS_DEST" 2>&1 | while IFS= read -r line; do
      log "$line"
    done
  fi

  # Inject plugin services into cereal/services.py
  local svc_script="$SCRIPT_DIR/plugins/services.py"
  local svc_target="$cereal_dir/services.py"
  if [[ -f "$svc_script" ]] && [[ -f "$svc_target" ]]; then
    python3 "$svc_script" "$svc_target" "$PLUGINS_DEST" 2>&1 | while IFS= read -r line; do
      log "$line"
    done
  fi

  # Clear __pycache__ for patched cereal files
  find "$cereal_dir" -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
}

# --- 3. Copy plugin packages ---
install_plugins() {
  log "Installing plugins to: $PLUGINS_DEST"

  if $DRY_RUN; then
    for plugin_dir in "$SCRIPT_DIR"/plugins/*/; do
      local name
      name="$(basename "$plugin_dir")"
      if [[ -f "$plugin_dir/plugin.json" ]]; then
        echo "  COPY plugins/$name/ → $PLUGINS_DEST/$name/"
      fi
    done
    return
  fi

  mkdir -p "$PLUGINS_DEST"

  for plugin_dir in "$SCRIPT_DIR"/plugins/*/; do
    local name
    name="$(basename "$plugin_dir")"
    [[ "$name" == "docs" ]] && continue

    if [[ ! -f "$plugin_dir/plugin.json" ]]; then
      warn "Skipping $name — no plugin.json"
      continue
    fi

    local dest="$PLUGINS_DEST/$name"
    if [[ -d "$dest" ]]; then
      log "  Updating: $name"
      # Preserve runtime state across reinstalls:
      #   data/     — persistent plugin params (survive reboot/reinstall)
      #   .disabled — user preference marker
      local had_data=false
      if [[ -d "$dest/data" ]]; then
        mv "$dest/data" "/tmp/_plugin_data_$$"
        had_data=true
      fi
      local was_disabled=false
      [[ -f "$dest/.disabled" ]] && was_disabled=true
      rm -rf "$dest"
      cp -r "$plugin_dir" "$dest"
      if $had_data; then
        mv "/tmp/_plugin_data_$$" "$dest/data"
      fi
      if $was_disabled; then
        touch "$dest/.disabled"
      fi
    else
      log "  Installing: $name"
      cp -r "$plugin_dir" "$dest"
    fi
  done
}

overlay_framework
overlay_ui
install_plugins
overlay_cereal

if $DRY_RUN; then
  log "Dry run complete. Re-run without --dry-run to apply."
else
  log "Installation complete."
  log ""
  log "Next steps:"
  log "  1. Reboot device (or restart openpilot) to activate plugins"
  log "  2. Disable a plugin:  touch $PLUGINS_DEST/<name>/.disabled"
  log "  3. Re-enable:         rm $PLUGINS_DEST/<name>/.disabled"
fi
