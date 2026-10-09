#!/usr/bin/env bash
# Build-time ONLY: self-contained XR Python stack, separate from ROS Humble.
# Called by the root Dockerfile as root. Does not communicate with a robot.
set -Eeuo pipefail
export DEBIAN_FRONTEND=noninteractive
export MAMBA_ROOT_PREFIX=/opt/mamba

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl bzip2
rm -rf /var/lib/apt/lists/*

# Same official Micromamba distribution channel used for the validated dev test.
# For fully bit-for-bit reproducible builds, additionally pin the tarball checksum.
curl --fail --location --retry 3 --silent --show-error \
  https://micro.mamba.pm/api/micromamba/linux-64/latest \
  | tar -xj -C /usr/local/bin --strip-components=1 bin/micromamba

micromamba create -y -n xr -c conda-forge --override-channels \
  python=3.10 pinocchio=3.1.0 numpy=1.26.4 casadi=3.6.7 pip

# Installed and confirmed by the user in the temporary XR environment.
# XR repo's own requirements also call for sshkeyboard and rerun-sdk.
micromamba run -n xr python -m pip install --no-cache-dir \
  'meshcat==0.3.2' \
  'matplotlib==3.8.4' \
  'logging-mp==0.2.5' \
  'sshkeyboard==2.3.1' \
  'rerun-sdk==0.20.1'

# Fail the image build if ROS Pinocchio shadows the XR bindings.
env -u PYTHONPATH -u LD_LIBRARY_PATH micromamba run -n xr python - <<'PY'
import numpy, casadi, pinocchio, meshcat.geometry, matplotlib, logging_mp
from pinocchio import casadi as cpin
assert numpy.__version__ == '1.26.4', numpy.__version__
assert pinocchio.__version__ == '3.1.0', pinocchio.__version__
assert casadi.__version__ == '3.6.7', casadi.__version__
print('XR Micromamba Python dependencies: OK')
PY

# Build XR CycloneDDS bindings against the ROS Humble native library.
# Do not change the validated XR numerical dependencies.
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  CYCLONEDDS_HOME=/opt/cyclonedds-prefix \
  /usr/local/bin/micromamba run -n xr \
  python -m pip install --no-cache-dir \
    --no-binary cyclonedds \
    --no-build-isolation \
    "cyclonedds==0.10.2"

# Required SDK checkout is pinned by Dockerfile; fail instead of silently skipping.
test -d /opt/unitree_sdk2_python/unitree_sdk2py || {
  echo "Missing pinned Unitree Python SDK checkout" >&2; exit 1;
}
env -u PYTHONPATH -u LD_LIBRARY_PATH micromamba run -n xr \
  python -m pip install --no-build-isolation --no-deps -e /opt/unitree_sdk2_python
env -u PYTHONPATH -u LD_LIBRARY_PATH micromamba run -n xr \
  python -c 'import unitree_sdk2py; print("XR Unitree SDK import: OK")' 

micromamba clean --all --yes
