#!/bin/bash
# c3_compat vitals watchdog — saves system state every 60s
# Run: setsid /data/plugins-runtime/c3_compat/watchdog.sh &
DIAG_DIR=/data/crash_diag
mkdir -p "$DIAG_DIR"
TICK=0
while true; do
  sleep 60
  TICK=$((TICK + 1))
  {
    echo "=== Vitals $(date '+%Y-%m-%d %H:%M:%S') ==="
    echo "--- uptime ---"
    uptime
    echo "--- memory ---"
    free -m
    echo "--- top CPU ---"
    ps aux --sort=-%cpu | head -8
    echo "--- top MEM ---"
    ps aux --sort=-%mem | head -8
    echo "--- thermal ---"
    cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -10
    echo "--- GPU ---"
    cat /sys/class/kgsl/kgsl-3d0/gpu_busy_percentage 2>/dev/null || echo "n/a"
    echo "--- panda USB ---"
    lsusb 2>/dev/null | grep -i 'panda\|3801\|bbaa' || echo "no panda"
    echo "--- dmesg tail (filtered) ---"
    dmesg | grep -v 'rcpi_applicable\|hdd_is_rcpi' | tail -30
  } > "$DIAG_DIR/vitals.log" 2>/dev/null

  # Every 10 min: save filtered dmesg (WiFi RCPI spam fills ring buffer)
  if [ $((TICK % 10)) -eq 0 ]; then
    dmesg -T | grep -v 'rcpi_applicable\|hdd_is_rcpi' > "$DIAG_DIR/dmesg_current.log" 2>/dev/null
  fi
done
