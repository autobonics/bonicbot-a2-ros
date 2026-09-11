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
    printf '  %-14s OK   -> %s\n' "$dev" "$(readlink -f "$dev")"
  else
    printf '  %-14s MISSING\n' "$dev"
    status=1
  fi
done

# Optional — a gamepad is not always plugged in, so absence is not a failure.
if [[ -e /dev/input/js0 ]]; then
  printf '  %-14s OK   -> %s\n' "/dev/input/js0" "$(stat -c '%U:%G %a' /dev/input/js0)"
  # Group membership, not the symlink, is what lets joy_node open the device.
  if ! id -nG "${SUDO_USER:-$USER}" | tr ' ' '\n' | grep -qx input; then
    printf '  %-14s WARNING: %s is not in the "input" group — joy_node cannot\n' \
      "" "${SUDO_USER:-$USER}"
    printf '  %-14s          open the pad. Fix: sudo usermod -aG input %s\n' \
      "" "${SUDO_USER:-$USER}"
  fi
else
  printf '  %-14s absent (no gamepad connected — not an error)\n' "/dev/input/js0"
fi

if [[ $status -ne 0 ]]; then
  cat <<'EOF'

Some devices are missing. Check that the hardware is plugged in, then find the
real vendor/product IDs and update config/udev/99-bonicbot.rules:

    udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"
    udevadm info /dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL"

The IDs in the shipped rules file are the common defaults, not guaranteed for
this unit.

/dev/video0 needs no rule and is NOT a sign the camera is usable: on Ubuntu
22.04's libcamera stack it is the raw `unicam` CSI receiver, which streams only
the sensor's native Bayer format (GB10 on the ov5647) and nothing else. The
camera is driven through libcamera (camera_ros), not v4l2_camera. To check the
camera itself, launch it and watch the topic rather than trusting this symlink:

    ros2 launch bonicbot_a2_hardware camera.launch.py
    ros2 topic hz /face_camera/image_raw
EOF
fi

exit $status
