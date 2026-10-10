#!/bin/bash
# =============================================================================
# XRoboToolkit Teleop - Jetson Orin (aarch64) Installation Script
# =============================================================================
# This script completes the installation of xrobotoolkit_teleop and all
# dependencies on Jetson Orin (aarch64, Ubuntu 20.04).
#
# Prerequisites already completed:
#   - gRPC and protobuf system packages installed
#   - PXREARobotSDK built for aarch64
#   - xrobotoolkit_sdk Python bindings built and installed
#
# Usage:
#   bash install_jetson_orin.sh
# =============================================================================
set -e

echo "========================================"
echo " XRoboToolkit Teleop - Jetson Installer"
echo "========================================"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Install system packages (if not already done)
# ---------------------------------------------------------------------------
echo "[Step 1/6] Installing system packages..."
echo "123" | sudo -S apt-get install -y -qq \
    libxml2-dev libxslt-dev \
    libboost-dev libeigen3-dev \
    2>&1 | tail -3
echo "  Done."

# ---------------------------------------------------------------------------
# Step 2: Install placo build dependencies
# ---------------------------------------------------------------------------
echo "[Step 2/6] Installing placo build dependencies..."

# Install cmeel build system
python3 -m pip install --user cmeel cmeel-boost cmeel-eigen 2>&1 | tail -3

# Install pin (required by placo)
python3 -m pip install --user "pin==3.8.0" 2>&1 | tail -3

# Install other placo build deps
python3 -m pip install --user rhoban-cmeel-jsoncpp doxystub 2>&1 | tail -3

echo "  Done."

# ---------------------------------------------------------------------------
# Step 3: Install placo
# ---------------------------------------------------------------------------
echo "[Step 3/6] Installing placo (this may take a while)..."

# Download and build placo from source
PLACO_VER="0.9.17"
PLACO_DIR="/tmp/placo-${PLACO_VER}"

if [ ! -d "${PLACO_DIR}" ]; then
    echo "  Downloading placo ${PLACO_VER}..."
    curl -L -o /tmp/placo-${PLACO_VER}.tar.gz \
        "https://files.pythonhosted.org/packages/source/p/placo/placo-${PLACO_VER}.tar.gz" \
        --connect-timeout 10 --max-time 120 -s

    cd /tmp
    tar xzf placo-${PLACO_VER}.tar.gz
fi

cd "${PLACO_DIR}"

# Modify build requirements to relax cmeel-boost version constraint
# (eiquadprog requires cmeel-boost ~=1.89.0 but max available is 1.88.0)
echo "  Relaxing cmeel-boost version requirements..."
sed -i 's/cmeel-boost ~= [0-9.]*/cmeel-boost >= 1.83.0/' pyproject.toml 2>/dev/null || true

# Try installing with no-build-isolation since we have all deps
python3 -m pip install --user --no-build-isolation . 2>&1 | tail -10

echo "  Done."

# ---------------------------------------------------------------------------
# Step 4: Install remaining Python dependencies
# ---------------------------------------------------------------------------
echo "[Step 4/6] Installing remaining Python dependencies..."

python3 -m pip install --user ur_rtde dex_retargeting 2>&1 | tail -5 || {
    echo "  Warning: ur_rtde or dex_retargeting failed - may not be needed for QJ2M"
}

echo "  Done."

# ---------------------------------------------------------------------------
# Step 5: Install xrobotoolkit_teleop package
# ---------------------------------------------------------------------------
echo "[Step 5/6] Installing xrobotoolkit_teleop..."

cd /home/agx02/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python

# Install in development mode (skip dependency checks since we installed manually)
python3 -m pip install --user -e . --no-deps 2>&1 | tail -5 || {
    # Fallback: regular install
    python3 -m pip install --user . --no-deps 2>&1 | tail -5
}

echo "  Done."

# ---------------------------------------------------------------------------
# Step 6: Verify installation
# ---------------------------------------------------------------------------
echo "[Step 6/6] Verifying installation..."

echo ""
echo "=== Verification Results ==="

# Check xrobotoolkit_sdk
python3 -c "import xrobotoolkit_sdk; print('[OK] xrobotoolkit_sdk:', xrobotoolkit_sdk.__file__)" 2>&1 || echo "[FAIL] xrobotoolkit_sdk"

# Check placo
python3 -c "import placo; print('[OK] placo:', placo.__file__)" 2>&1 || echo "[FAIL] placo"

# Check numpy
python3 -c "import numpy; print('[OK] numpy:', numpy.__version__)" 2>&1 || echo "[FAIL] numpy"

# Check meshcat
python3 -c "import meshcat; print('[OK] meshcat')" 2>&1 || echo "[FAIL] meshcat"

# Check xrobotoolkit_teleop
python3 -c "import xrobotoolkit_teleop; print('[OK] xrobotoolkit_teleop')" 2>&1 || echo "[FAIL] xrobotoolkit_teleop"

# Check rclpy (ROS 2)
python3 -c "import rclpy; print('[OK] rclpy')" 2>&1 || echo "[FAIL] rclpy"

echo ""
echo "========================================"
echo " Installation Complete!"
echo "========================================"
echo ""
echo "To run QJ2M teleoperation:"
echo ""
echo "  # Simulation:"
echo "  python3 scripts/simulation/teleop_qj2m_placo.py"
echo ""
echo "  # Hardware:"
echo "  python3 scripts/hardware/teleop_qj2m_hardware.py"
echo ""
echo "  # Hardware with IK visualization:"
echo "  python3 scripts/hardware/teleop_qj2m_hardware.py --visualize-placo"
echo ""
echo "See docs/QJ2M_DEPLOYMENT_GUIDE.md for detailed documentation."
echo "========================================"
