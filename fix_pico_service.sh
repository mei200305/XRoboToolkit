#!/bin/bash
# Fix PICO RoboticsServiceProcess for Ubuntu 20.04 (Jetson Orin)
# The official binary needs GLIBC 2.34 + ICU 70 which are from Ubuntu 22.04
set -e

PASS="123"
echo "=== Fixing PICO Service for Ubuntu 20.04 ==="

# Step 1: Install ICU 70 libraries from Ubuntu 22.04 package
echo "[1/3] Installing ICU 70 libraries..."
cd /tmp
if [ ! -f libicu70_arm64.deb ]; then
    wget -q "http://ports.ubuntu.com/pool/main/i/icu/libicu70_70.1-2_arm64.deb" --timeout=60 -O libicu70_arm64.deb
fi
mkdir -p icu70_extract
dpkg-deb -x libicu70_arm64.deb icu70_extract/
echo "$PASS" | sudo -S cp icu70_extract/usr/lib/aarch64-linux-gnu/libicu*.so.70* /usr/lib/aarch64-linux-gnu/ 2>/dev/null
echo "$PASS" | sudo -S ldconfig 2>/dev/null

# Step 2: Replace SDK with our GLIBC 2.31-compatible version
echo "[2/3] Installing compatible SDK library..."
SDK_SRC="/home/agx02/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind/lib/libPXREARobotSDK.so"
echo "$PASS" | sudo -S cp "$SDK_SRC" /opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so

# Step 3: Create wrapper using snap core22's GLIBC 2.35
echo "[3/3] Creating PICO service wrapper..."
SNAP_LD="/snap/core22/2412/lib/ld-linux-aarch64.so.1"

if [ ! -f "$SNAP_LD" ]; then
    echo "ERROR: snap core22 not found. Install with: sudo snap install core22"
    exit 1
fi

cat > /tmp/run_pico.sh << 'RUNEOF'
#!/bin/bash
DIR="/opt/apps/roboticsservice"
SNAP_LIBC="/snap/core22/2412/usr/lib/aarch64-linux-gnu"
ICU_DIR="/usr/lib/aarch64-linux-gnu"

export LD_LIBRARY_PATH="${DIR}:${DIR}/lib:${DIR}/SDK/arm64:${SNAP_LIBC}:${ICU_DIR}"
export QT_PLUGIN_PATH="${DIR}/plugins/"
export QT_QML_PATH="${DIR}/qml/"

cd "${DIR}"
exec /snap/core22/2412/lib/ld-linux-aarch64.so.1 --library-path "${LD_LIBRARY_PATH}" ./RoboticsServiceProcess "$@"
RUNEOF

chmod +x /tmp/run_pico.sh

# Test
echo "Testing PICO service..."
/tmp/run_pico.sh &
PID=$!
sleep 3
if kill -0 $PID 2>/dev/null; then
    echo "SUCCESS: PICO service running (PID=$PID)"
    echo "To start in the future: /tmp/run_pico.sh &"
    kill $PID 2>/dev/null
else
    echo "Status: Check for errors above"
    echo "To debug: ldd /opt/apps/roboticsservice/RoboticsServiceProcess"
fi
