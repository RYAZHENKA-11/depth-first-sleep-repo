#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
IMAGE="go2-sim-lyrical"

docker build -q -t "$IMAGE" "$REPO/src/go2_sim/docker" > /dev/null
echo "RViz: http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
exec docker run --rm -it -p 6080:6080 -v "$REPO":/ws:ro --name go2_sim "$IMAGE" "$@"
