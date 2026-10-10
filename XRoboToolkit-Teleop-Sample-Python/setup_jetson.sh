#!/bin/bash
# =============================================================================
# XRoboToolkit Teleop - Jetson Orin (aarch64) 一键安装脚本
# =============================================================================
# 运行: bash setup_jetson.sh
# 密码: 123
# =============================================================================
set -e

PASS="123"
PROJ_DIR="/home/agx02/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python"
VENV_DIR="/home/agx02/xr_env"

echo "========================================"
echo " XRoboToolkit Teleop - Jetson Setup"
echo "========================================"
echo ""

# ---------------------------------------------------------------------------
# Step 1: System dependencies
# ---------------------------------------------------------------------------
echo "[1/6] Installing system packages..."
echo "$PASS" | sudo -S apt-get update -qq 2>/dev/null
echo "$PASS" | sudo -S apt-get install -y -qq \
    python3.8-venv python3-venv \
    libxml2-dev libxslt-dev \
    cmake build-essential \
    libboost-dev libeigen3-dev \
    2>&1 | tail -3
echo "  Done."

# ---------------------------------------------------------------------------
# Step 2: Create Python virtual environment
# ---------------------------------------------------------------------------
echo "[2/6] Creating Python virtual environment..."
rm -rf "$VENV_DIR"
python3 -m venv "$VENV_DIR" --system-site-packages
source "$VENV_DIR/bin/activate"
pip install --upgrade pip uv 2>&1 | tail -3
echo "  Done."

# ---------------------------------------------------------------------------
# Step 3: Install xrobotoolkit_sdk (already built)
# ---------------------------------------------------------------------------
echo "[3/6] Installing xrobotoolkit_sdk..."
cd "${PROJ_DIR}/dependencies/XRoboToolkit-PC-Service-Pybind"
export pybind11_DIR="/home/agx02/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11"
rm -rf build
python setup.py install 2>&1 | tail -5
echo "  Done."

# ---------------------------------------------------------------------------
# Step 4: Install placo + Python deps via uv
# ---------------------------------------------------------------------------
echo "[4/6] Installing placo and Python dependencies (may take a while)..."
source "$VENV_DIR/bin/activate"
uv pip install \
    placo \
    numpy \
    meshcat \
    opencv-python \
    tyro \
    dynamixel-sdk \
    ur-rtde \
    2>&1 | tail -20
echo "  Done."

# ---------------------------------------------------------------------------
# Step 5: Install xrobotoolkit_teleop
# ---------------------------------------------------------------------------
echo "[5/6] Installing xrobotoolkit_teleop..."
cd "$PROJ_DIR"
source "$VENV_DIR/bin/activate"
uv pip install -e . 2>&1 | tail -5
echo "  Done."

# ---------------------------------------------------------------------------
# Step 6: Verify
# ---------------------------------------------------------------------------
echo "[6/6] Verifying installation..."
source "$VENV_DIR/bin/activate"

echo ""
echo "=== Verification ==="

for mod in "xrobotoolkit_sdk" "placo" "numpy" "meshcat" "xrobotoolkit_teleop" "rclpy"; do
    python -c "import ${mod}; print('[OK]', '${mod}')" 2>/dev/null && echo "" || echo "[FAIL] ${mod}"
done

echo ""
echo "========================================"
echo " To activate environment:"
echo "   source ${VENV_DIR}/bin/activate"
echo ""
echo " To run QJ2M simulation:"
echo "   cd ${PROJ_DIR}"
echo "   python scripts/simulation/teleop_qj2m_placo.py"
echo ""
echo " To run QJ2M hardware:"
echo "   cd ${PROJ_DIR}"
echo "   python scripts/hardware/teleop_qj2m_hardware.py"
echo "========================================"
