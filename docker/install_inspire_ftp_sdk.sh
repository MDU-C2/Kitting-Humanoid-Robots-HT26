#!/usr/bin/env bash
# Install only the Python Inspire FTP DDS SDK in XR's own environment.
# Does not run any hand examples, ROS nodes, or motor command publishers.
set -Eeuo pipefail

export MAMBA_ROOT_PREFIX=/opt/mamba

# Micromamba's Python _ssl extension must resolve the OpenSSL libraries that
# belong to the same environment.  Ubuntu's system libcrypto.so.3 is older
# than the version required by some conda-forge Python builds.  Scope the
# linker search path to XR Python invocations; never change ROS process
# library resolution or globally export this for Git and system utilities.
XR_LIB=/opt/mamba/envs/xr/lib
test -f "$XR_LIB/libcrypto.so.3" || {
  echo "Missing XR OpenSSL: $XR_LIB/libcrypto.so.3" >&2
  exit 1
}

SDK_ROOT=/opt/inspire_hand_ws
SDK_REPOSITORY=https://github.com/NaCl-1374/inspire_hand_ws.git
SDK_REF="${INSPIRE_HAND_SDK_REF:-master}"

if [[ -e "$SDK_ROOT" ]]; then
  echo "Refusing to replace an existing SDK source directory: $SDK_ROOT" >&2
  exit 1
fi

# Sparse checkout avoids downloading the project's bundled Python environment
# and other unrelated files. Do not initialize the unitree SDK submodule:
# the Docker XR environment already has its own compatible unitree_sdk2py.
git clone --quiet --depth 1 --filter=blob:none --sparse \
  "$SDK_REPOSITORY" "$SDK_ROOT"
git -C "$SDK_ROOT" sparse-checkout set inspire_hand_sdk

# For stable rebuilds, set INSPIRE_HAND_SDK_REF to a reviewed commit SHA.
# The default 'master' is provided for initial compatibility testing only.
if [[ "$SDK_REF" != master ]]; then
  git -C "$SDK_ROOT" fetch --quiet --depth 1 origin "$SDK_REF"
  git -C "$SDK_ROOT" checkout --quiet --detach FETCH_HEAD
fi

SDK_SHA="$(git -C "$SDK_ROOT" rev-parse HEAD)"
echo "Inspire FTP SDK source revision: $SDK_SHA"

test -f "$SDK_ROOT/inspire_hand_sdk/setup.py" || {
  echo 'Inspire FTP SDK setup.py missing: refusing installation' >&2
  exit 1
}

# The upstream package imports Modbus and Qt modules from __init__.py even
# when only the DDS message classes are requested. Install the declared runtime
# dependencies in XR's isolated Python environment; do NOT alter the upstream
# package initializer or the working xr_teleoperator source.
# NumPy remains pinned to the version used by XR Pinocchio.
env -u PYTHONPATH LD_LIBRARY_PATH="$XR_LIB" \
  micromamba run -n xr python -m pip install --no-cache-dir \
  'numpy==1.26.4' \
  'pymodbus==3.6.9' \
  'pyserial==3.5' \
  'PyQt5==5.15.11' \
  'pyqtgraph==0.13.7' \
  'colorcet==3.1.0'

# Install the SDK itself without pulling or upgrading any further packages.
env -u PYTHONPATH LD_LIBRARY_PATH="$XR_LIB" \
  micromamba run -n xr python -m pip install \
  --no-cache-dir --no-deps --no-build-isolation \
  "$SDK_ROOT/inspire_hand_sdk"

env -u PYTHONPATH LD_LIBRARY_PATH="$XR_LIB" \
  micromamba run -n xr python - <<'PY'
import numpy
import ssl
import pinocchio
import cyclonedds
from inspire_sdkpy import inspire_dds
from inspire_sdkpy import ModbusDataHandler, ImageTab
assert numpy.__version__ == '1.26.4', numpy.__version__
assert pinocchio.__version__ == '3.1.0', pinocchio.__version__
print('XR OpenSSL:', ssl.OPENSSL_VERSION)
assert hasattr(inspire_dds, 'inspire_hand_state'), \
    'FTP DDS message type inspire_hand_state missing'
print('Inspire FTP SDK Python import OK (NO DDS initialization or commands)')
PY
