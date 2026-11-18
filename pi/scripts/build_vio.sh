#!/bin/bash
# Tải OpenVINS và biên dịch ovw_node (không ROS). Trên Pi 4 mất ~15-25 phút.
#   bash ~/iot-ck/pi/scripts/build_vio.sh [số_luồng]
set -euo pipefail
cd "$(dirname "$0")/../vio"
OV_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b   # đã thử nghiệm với commit này
if [ ! -d third_party/open_vins/.git ]; then
  mkdir -p third_party
  git clone https://github.com/rpng/open_vins.git third_party/open_vins
fi
git -C third_party/open_vins checkout -q "$OV_COMMIT"
JOBS=${1:-3}   # 3 luồng vừa đủ RAM 4 GB (có swap 2 GB)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$JOBS"
ls -la build/ovw_node
echo "XONG: $(pwd)/build/ovw_node"
