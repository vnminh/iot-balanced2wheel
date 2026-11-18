#!/bin/bash
# Tạo môi trường Python (dùng chung picamera2/numpy/yaml của hệ thống) + aiohttp.
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -r pi/requirements.txt
.venv/bin/python -c "import aiohttp, serial, yaml, numpy; print('python OK, aiohttp', aiohttp.__version__)"
