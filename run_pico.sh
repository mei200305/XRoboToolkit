#!/bin/bash
# Wrapper to start PICO RoboticsServiceProcess on Ubuntu 20.04 Jetson Orin
# Uses snap core22's GLIBC 2.35 to satisfy the binary's GLIBC 2.34 requirement
DIR="/opt/apps/roboticsservice"
SNAP_LIBC="/snap/core22/2412/usr/lib/aarch64-linux-gnu"
ICU_DIR="/usr/lib/aarch64-linux-gnu"

export LD_LIBRARY_PATH="${DIR}:${DIR}/lib:${DIR}/SDK/arm64:${SNAP_LIBC}:${ICU_DIR}"
export QT_PLUGIN_PATH="${DIR}/plugins/"
export QT_QML_PATH="${DIR}/qml/"

cd "${DIR}"
exec /snap/core22/2412/lib/ld-linux-aarch64.so.1 --library-path "${LD_LIBRARY_PATH}" ./RoboticsServiceProcess "$@"
