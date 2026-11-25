#!/bin/bash
# Biên dịch lõi SLAM 2D (pi/slam -> pi/slam/build/slam_node). Trên Pi 4 mất ~3-5 phút.
#   bash ~/iot-ck/pi/scripts/build_slam.sh [số_luồng] [--test]
#   --test: build + chạy GoogleTest và kịch bản giả lập (cần libgtest-dev, chậm hơn)
set -euo pipefail
cd "$(dirname "$0")/../slam"
JOBS=2
TEST=OFF
for a in "$@"; do
  case "$a" in
    --test) TEST=ON ;;
    *) JOBS="$a" ;;
  esac
done
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSLAM_TESTS=$TEST
cmake --build build -j"$JOBS"
sync   # mất điện ngay sau build -> file rỗng 0 byte (đã gặp trên xe này)
[ -s build/slam_node ] || { echo "slam_node rỗng: xoá pi/slam/build rồi chạy lại" >&2; exit 1; }
if [ "$TEST" = ON ]; then
  (cd build && ctest --output-on-failure)
  mkdir -p /tmp/slam_sim_maps && build/slam_sim --config ../config/slam.yaml --maps /tmp/slam_sim_maps
fi
ls -la build/slam_node
echo "XONG: $(pwd)/build/slam_node"
