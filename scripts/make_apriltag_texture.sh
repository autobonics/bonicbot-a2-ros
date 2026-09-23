#!/usr/bin/env bash
#
# Fetch an AprilTag image and upscale it for use as a Gazebo texture.
#
# The tag36h11 family's images are a published dataset, not something to
# generate by hand — the 36-bit codeword per id comes out of the family
# definition, and an invented pattern is simply not a tag. This pulls the
# canonical PNG from AprilRobotics/apriltag-imgs, the family's own repository.
#
#   ./scripts/make_apriltag_texture.sh [tag_id]     # default: 0
#
# The source images are 10x10 PIXELS — one pixel per tag cell, which is correct
# and complete. They are upscaled here only because renderers filter small
# textures into mush; the upscale MUST be nearest-neighbour or the cell edges
# blur and detection degrades at range.
#
# Output lands next to apriltag_plate.obj, because its .mtl resolves map_Kd
# relative to itself. Not committed to git: it is a generated build asset, and
# the dataset it comes from is upstream and stable.
set -euo pipefail

TAG_ID="${1:-0}"
SCALE=100                       # 10x10 px -> 1000x1000 px

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$HERE/src/sim/bonicbot_a2_sim/meshes"
NAME="$(printf 'tag36_11_%05d.png' "$TAG_ID")"
OUT="$OUT_DIR/$(printf 'tag36h11_%05d.png' "$TAG_ID")"
URL="https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/$NAME"

mkdir -p "$OUT_DIR"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "fetching $URL"
curl -fsSL "$URL" -o "$TMP/small.png"

# Nearest-neighbour upscale. Python/Pillow first because it states the filter
# explicitly (Image.NEAREST); ImageMagick's -filter point is the equivalent.
# Do NOT substitute a plain `convert -resize`: its default filter interpolates,
# and soft-edged tag cells are a detection-range problem, not a cosmetic one.
if python3 -c 'import PIL' 2>/dev/null; then
    python3 - "$TMP/small.png" "$OUT" "$SCALE" <<'PY'
import sys
from PIL import Image
src, dst, scale = sys.argv[1], sys.argv[2], int(sys.argv[3])
img = Image.open(src).convert('RGB')
w, h = img.size
img.resize((w * scale, h * scale), Image.NEAREST).save(dst)
print(f'{src} {w}x{h} -> {dst} {w*scale}x{h*scale} (nearest)')
PY
elif command -v convert >/dev/null 2>&1; then
    SIZE=$(( 10 * SCALE ))
    # -alpha off: the source PNGs are RGBA (fully opaque), and an alpha channel
    # surviving into an Ogre texture is a needless way for the tag's white quiet
    # zone to end up transparent. The Pillow path drops it via convert('RGB').
    convert "$TMP/small.png" -filter point -resize "${SIZE}x${SIZE}" -alpha off "$OUT"
    echo "$OUT written (${SIZE}x${SIZE}, ImageMagick -filter point)"
else
    echo "need python3-pil or imagemagick:" >&2
    echo "  sudo apt install python3-pil     # or: sudo apt install imagemagick" >&2
    exit 1
fi

cat <<EOF

Done: $OUT

Rebuild so the texture is installed alongside the mesh:
    colcon build --packages-select bonicbot_a2_sim

If the tag is visible in Gazebo but apriltag_node never reports a detection,
the texture is mirrored (a mirrored AprilTag does not decode at all). Fix:
    convert "$OUT" -flop "$OUT"
EOF
