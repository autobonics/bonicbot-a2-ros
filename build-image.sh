#!/usr/bin/env bash
# Build the bonicbot-a2-ros image with a version derived from git.
#
#   ./build-image.sh              # build, tag, print the ref
#   ./build-image.sh --push       # and push it
#
# WHY THIS SCRIPT EXISTS AT ALL. The image tag used to be typed by hand, and
# `2.0.0` ended up meaning nothing: no git tag, no recorded commit, no way to
# answer "what source is in this image?" without opening it. That cost a day —
# a robot whose camera would not stream, because 2.0.0 turned out to predate
# the v4l2_camera -> camera_ros switch, and nothing said so.
#
# So the version comes from `git describe` and the commit is baked in. Same
# rules as bonicOS-host's build-deb.sh, deliberately: one versioning idea
# across the whole stack rather than one per repo.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${BONICBOT_ROS_IMAGE:-autobonics/bonicbot-a2-ros}"
PUSH=false
PLATFORM="${PLATFORM:-linux/arm64}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --push)     PUSH=true; shift ;;
    --image)    IMAGE="$2"; shift 2 ;;
    --platform) PLATFORM="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

cd "$REPO"

# ── version, derived from git (never typed) ───────────────────────
#   at a tag            -> 2.1.0
#   N commits past it   -> 2.1.0+7.gabc1234   (sorts after the tag)
#   no tags at all      -> 0.0.0+g<sha>       (sorts below every release)
#   dirty tree          -> +dirty appended, loudly
dirty=false
git diff --quiet -- . || dirty=true
git diff --cached --quiet -- . || dirty=true

tag=""
if tag="$(git describe --tags --exact-match 2>/dev/null)"; then
  version="${tag#v}"
elif describe="$(git describe --tags 2>/dev/null)"; then
  if [[ "$describe" =~ ^(.*)-([0-9]+)-g([0-9a-fA-F]+)$ ]]; then
    version="${BASH_REMATCH[1]#v}+${BASH_REMATCH[2]}.g${BASH_REMATCH[3]}"
  else
    echo "error: unrecognised 'git describe' output: $describe" >&2; exit 1
  fi
else
  # No tags reachable. This is the state the repo was in when the camera bug
  # happened — do not fail on it, but make it obvious the image is untagged.
  version="0.0.0+g$(git rev-parse --short HEAD)"
  echo "NOTE: no git tags found. Building ${version}." >&2
  echo "      Tag a release so the image version means something:" >&2
  echo "        git tag v2.1.0 && git push origin v2.1.0" >&2
fi

commit="$(git rev-parse HEAD)"

if [[ "$dirty" == true ]]; then
  version="${version}+dirty"
  echo "WARNING: working tree is dirty — building ${version}." >&2
  echo "         The commit recorded in the image does NOT describe what is" >&2
  echo "         actually in it. Do not ship this." >&2
fi

ref="${IMAGE}:${version}"
built="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

echo "==> building $ref"
echo "    commit:   ${commit:0:12}"
echo "    platform: $PLATFORM"

# --load so the image lands in the local daemon where bonicOS-robot-app's build
# can stack on it. --push instead when publishing, because buildx cannot do
# both in one invocation for a single-platform build.
out=(--load)
$PUSH && out=(--push)

docker buildx build \
  --platform "$PLATFORM" \
  -f Dockerfile.ros \
  --build-arg "VERSION=${version}" \
  --build-arg "GIT_COMMIT=${commit}" \
  --build-arg "BUILD_DATE=${built}" \
  -t "$ref" \
  "${out[@]}" \
  .

echo
echo "  image:  $ref"
echo "  commit: $commit"
echo
echo "  stack robot_app on it:"
echo "    cd ../bonicOS-robot-app && docker buildx build --platform $PLATFORM \\"
echo "      --build-arg ROS_IMAGE=$ref --build-arg VERSION=<bonicos version> ..."
echo
echo "  read it back from any image or running container:"
echo "    docker run --rm $ref cat /ws/version.txt"
