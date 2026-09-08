#!/usr/bin/env bash
# Install BonicBot A2 udev rules and verify the resulting device symlinks.
#
# Run once per robot:  sudo ./scripts/setup_udev.sh
set -euo pipefail

RULES_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/udev/99-bonicbot.rules"
RULES_DST="/etc/udev/rules.d/99-bonicbot.rules"

if [[ $EUID -ne 0 ]]; then
  echo "Must run as root: sudo $0" >&2
  exit 1
fi

if [[ ! -f "$RULES_SRC" ]]; then
  echo "Rules file not found: $RULES_SRC" >&2
  exit 1
fi

echo "Installing $RULES_SRC -> $RULES_DST"
install -m 0644 "$RULES_SRC" "$RULES_DST"

udevadm control --reload-rules
udevadm trigger
sleep 2

echo
echo "Device check:"
status=0
for dev in /dev/esp /dev/lidar /dev/video0; do
  if [[ -e "$dev" ]]; then
    printf '  %-12s OK   -> %s\n' "$dev" "$(readlink -f "$dev")"
  else
    printf '  %-12s MISSING\n' "$dev"
    status=1
  fi
done

if [[ $status -ne 0 ]]; then
  cat <<'EOF'

Some devices are missing. Check that the hardware is plugged in, then find the
real vendor/product IDs and update config/udev/99-bonicbot.rules:

    udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"
    udevadm info /dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"

The IDs in the shipped rules file are the common defaults, not guaranteed for
this unit. If /dev/video0 is missing and the camera is a Module 3 (IMX708), it
is libcamera-only and will not enumerate as a v4l2 device at all.
EOF
fi

exit $status
