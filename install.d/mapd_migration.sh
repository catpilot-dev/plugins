# shellcheck shell=bash
#
# install.d/mapd_migration.sh — one-way migration off the removed mapd plugin.
#
# Sourced by install.sh. Function definitions ONLY: sourcing this file has no
# side effects. Spec: docs/superpowers/plans/2026-11-21-remove-mapd.md (R5, R5h-j);
# end-to-end test: plugins/tests/test_mapd_migration.py.
#
# The mapd plugin was removed from the repo on 2026-11-21 (OSM data now comes only
# from speedlimitd's local tile reader). install_plugins never deletes a runtime
# plugin dir that vanished from the repo, and deleting it outright breaks cereal:
# catpilot's boot builder strips every appended standalone type and re-appends
# only those of enabled plugins, while slots 17-19 still reference mapd's types
# (`custom.capnp:73: Not defined: MapdDownloadProgress`). So:
#   a. disable mapd (touch .disabled, rm .enforced), so the cereal step's
#      custom_capnp.py reverts slots 17-19 to CustomReservedN stubs and drops
#      mapd's standalone types, and services.py stops injecting mapd services;
#   run the cereal step (install.sh: overlay_cereal) with errexit live inside it;
#   b. verify the revert; only then rm -rf the mapd runtime dir. If the cereal
#      step failed or the revert is not verified, undo (a) so the device boots
#      exactly as before (a .disabled mapd with unreverted slots crashes the same
#      way), re-run the cereal step best-effort, and warn;
#   c. once no mapd dir remains, drop the stale mapdOut / mapdExtendedOut lines
#      from cereal/services.py (nothing re-injects them, nothing removes them).
# An EXIT trap backstop undoes (a) if the shell dies while the migration is
# pending. The mapd binary, its backups and the tiles are left alone.
#
# Every step warns and continues and mapd_migrate always returns 0: install.sh
# runs under `set -euo pipefail`, and launch_chffrplus.sh swallows its exit code,
# so an abort here would silently skip the rest of the install.
#
# Inputs (install.sh globals): PLUGINS_DEST, OP_PKG (cereal dir = $OP_PKG/cereal),
# SCRIPT_DIR (repo root, for plugins/services.py), DRY_RUN (true/false).
#
# Call mapd_migrate as a plain statement, never as an if/||/&& condition: bash
# ignores errexit inside a condition, subshells included, so a failing cereal
# step could not be detected by its status (the verify step still catches an
# unreverted schema).

_mapd_log() {
  if declare -F log >/dev/null; then log "$@"; else echo "[install] $*"; fi
}

_mapd_warn() {
  if declare -F warn >/dev/null; then warn "$@"; else echo "[install] WARNING: $*" >&2; fi
}

_mapd_dry_run() {
  [[ "${DRY_RUN:-false}" == true ]]
}

# Run the cereal step in a subshell with errexit on, so a failure inside it
# stops it and its status is real. The status goes to _MAPD_CEREAL_RC; the
# function itself always returns 0 (a non-zero return would trip the caller's
# errexit, and testing it with || would disable errexit inside the step).
_mapd_run_cereal() {
  local cmd="$1" had_errexit=false
  [[ $- == *e* ]] && had_errexit=true
  set +e
  ( set -e; "$cmd" )
  _MAPD_CEREAL_RC=$?
  if $had_errexit; then set -e; fi
  return 0
}

# Step (a).
_mapd_disable() {
  local dir="$1"
  touch "$dir/.disabled" || _mapd_warn "mapd migration: could not touch $dir/.disabled"
  rm -f "$dir/.enforced" || _mapd_warn "mapd migration: could not remove $dir/.enforced"
}

# Undo step (a): enabled + enforced, the only state the old install.sh ever left
# mapd in (it re-enforced mapd and cleared .disabled on every run).
_mapd_undo() {
  local dir="$1"
  rm -f "$dir/.disabled" || _mapd_warn "mapd migration: could not remove $dir/.disabled"
  touch "$dir/.enforced" || _mapd_warn "mapd migration: could not touch $dir/.enforced"
}

# Step (b): the last check before an irreversible rm -rf.
_mapd_cereal_reverted() {
  local custom="$1/custom.capnp" logc="$1/log.capnp" n ev
  [[ -f "$custom" && -f "$logc" ]] || return 1
  if grep -qE '^[[:space:]]*struct MapdOut([^A-Za-z0-9_]|$)' "$custom"; then
    return 1
  fi
  for n in 17 18 19; do
    ev=$((n + 126))   # Event union ordinals: slot 17 -> @143, 18 -> @144, 19 -> @145
    grep -qE "^[[:space:]]*struct CustomReserved$n @0x[0-9a-f]+ \\{" "$custom" || return 1
    grep -qE "^[[:space:]]*customReserved$n @$ev :Custom\\.CustomReserved$n;" "$logc" || return 1
  done
  return 0
}

# Step (c).
_mapd_remove_stale_services() {
  local svc="$1" script="$2" out
  [[ -f "$svc" ]] || return 0
  grep -qE '^[[:space:]]*"(mapdOut|mapdExtendedOut)"[[:space:]]*:' "$svc" || return 0
  if _mapd_dry_run; then
    echo "  REMOVE stale mapdOut/mapdExtendedOut entries from ${svc##*/}"
    return 0
  fi
  if [[ ! -f "$script" ]]; then
    _mapd_warn "mapd migration: $script not found; stale mapd services stay in $svc"
    return 0
  fi
  if out="$(python3 "$script" --remove "$svc" mapdOut mapdExtendedOut 2>&1)"; then
    _mapd_log "$out"
  else
    _mapd_warn "mapd migration: removing stale mapd services failed: $out"
  fi
  return 0
}

# Prints the command inside a `trap -p EXIT` line (empty for an empty line).
_mapd_trap_spec_cmd() {
  [[ -n "${1:-}" ]] || return 0
  eval "set -- $1"   # trap -- '<cmd>' EXIT
  printf '%s' "$3"
}

_mapd_arm_backstop() {
  _MAPD_PREV_EXIT_TRAP="$(trap -p EXIT)"
  _MAPD_PREV_EXIT_CMD="$(_mapd_trap_spec_cmd "$_MAPD_PREV_EXIT_TRAP")"
  trap '_mapd_exit_backstop' EXIT
}

_mapd_disarm_backstop() {
  if [[ -n "${_MAPD_PREV_EXIT_TRAP:-}" ]]; then
    eval "$_MAPD_PREV_EXIT_TRAP"
  else
    trap - EXIT
  fi
  unset _MAPD_PREV_EXIT_TRAP _MAPD_PREV_EXIT_CMD _MAPD_MIGRATION_DIR
  _MAPD_MIGRATION_PENDING=false
}

_mapd_return() { return "$1"; }

# EXIT trap: the shell is going away with step (a) applied and (b) not done.
_mapd_exit_backstop() {
  local status=$?
  if [[ "${_MAPD_MIGRATION_PENDING:-false}" == true ]]; then
    _MAPD_MIGRATION_PENDING=false
    _mapd_warn "mapd migration interrupted: re-enabling mapd so the next boot still builds"
    _mapd_undo "${_MAPD_MIGRATION_DIR:?}"
  fi
  if [[ -n "${_MAPD_PREV_EXIT_CMD:-}" ]]; then
    # Chain install.sh's own EXIT trap (if any), with the exit status it expects.
    set +e
    _mapd_return "$status"
    eval "$_MAPD_PREV_EXIT_CMD"
  fi
}

# mapd_migrate <cereal_cmd>: run <cereal_cmd> (install.sh: overlay_cereal) with
# the mapd migration around it. Always returns 0.
mapd_migrate() {
  local cereal_cmd="${1:-}"
  if [[ -z "$cereal_cmd" ]]; then
    _mapd_warn "mapd_migrate: no cereal command given"
    return 0
  fi

  if [[ -z "${PLUGINS_DEST:-}" || -z "${OP_PKG:-}" ]]; then
    _mapd_warn "mapd migration skipped: PLUGINS_DEST or OP_PKG not set"
    _mapd_run_cereal "$cereal_cmd"
    [[ "$_MAPD_CEREAL_RC" -eq 0 ]] || _mapd_warn "cereal step failed (exit $_MAPD_CEREAL_RC)"
    return 0
  fi

  local dir="$PLUGINS_DEST/mapd"
  local cereal_dir="$OP_PKG/cereal"
  local repo="${SCRIPT_DIR:-}"
  if [[ -z "$repo" ]]; then
    repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || repo=""
  fi
  local svc_script="$repo/plugins/services.py"
  local undone=false

  if _mapd_dry_run; then
    if [[ -d "$dir" ]]; then
      echo "  MIGRATE mapd (plugin removed): touch .disabled + rm .enforced, revert cereal slots 17-19, verify, rm -rf $dir"
    fi
    _mapd_run_cereal "$cereal_cmd"   # overlay_cereal returns early under DRY_RUN
    _mapd_remove_stale_services "$cereal_dir/services.py" "$svc_script"
    return 0
  fi

  if [[ -d "$dir" ]]; then
    _mapd_log "mapd plugin was removed: disabling it so the cereal step reverts slots 17-19"
    _MAPD_MIGRATION_DIR="$dir"
    _MAPD_MIGRATION_PENDING=true
    _mapd_arm_backstop
    _mapd_disable "$dir"

    _mapd_run_cereal "$cereal_cmd"
    local rc="$_MAPD_CEREAL_RC"

    if [[ "$rc" -eq 0 ]] && _mapd_cereal_reverted "$cereal_dir"; then
      _MAPD_MIGRATION_PENDING=false
      # Unlink the manifest first: one atomic step after which no loader
      # (builder.py, custom_capnp.py, services.py) treats the dir as a plugin,
      # even if the rm -rf below is interrupted half-way.
      rm -f "${PLUGINS_DEST:?}/mapd/plugin.json" || _mapd_warn "mapd migration: could not remove mapd/plugin.json"
      if rm -rf "${PLUGINS_DEST:?}/mapd"; then
        _mapd_log "Removed mapd plugin dir $dir (cereal slots 17-19 reverted; mapd binary and tiles kept)"
      else
        _mapd_warn "mapd migration: rm -rf $dir failed; will retry next install"
      fi
    else
      if [[ "$rc" -ne 0 ]]; then
        _mapd_warn "mapd migration: cereal step failed (exit $rc); keeping mapd enabled"
      else
        _mapd_warn "mapd migration: cereal slots 17-19 not reverted; keeping mapd enabled"
      fi
      _mapd_undo "$dir"
      _MAPD_MIGRATION_PENDING=false
      undone=true
      _mapd_run_cereal "$cereal_cmd"
      if [[ "$_MAPD_CEREAL_RC" -ne 0 ]]; then
        _mapd_warn "mapd migration: cereal step failed again after restoring mapd (exit $_MAPD_CEREAL_RC)"
      fi
    fi
    _mapd_disarm_backstop
  else
    _mapd_run_cereal "$cereal_cmd"
    [[ "$_MAPD_CEREAL_RC" -eq 0 ]] || _mapd_warn "cereal step failed (exit $_MAPD_CEREAL_RC)"
  fi

  if ! $undone && [[ ! -d "$dir" ]]; then
    _mapd_remove_stale_services "$cereal_dir/services.py" "$svc_script"
  fi
  return 0
}
