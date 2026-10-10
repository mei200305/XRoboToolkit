from pathlib import Path
PROJ_DIR = Path(__file__).resolve().parent

import os
import sys
from distutils.util import strtobool
from .logging import logger
from ..ezros import xbin

def getEnvBool(env_var, default=False) -> bool:
    return strtobool(os.environ.get(env_var, str(default)))

def getEnvStr(env_var, default='') -> str:
    return os.environ.get(env_var, default)

# env
from .plat import detectMachineType, detectSystemType, detectArchType

ARCH_TYPE = detectArchType()  # "x86_64", "aarch64", etc.
SYSTEM_TYPE = detectSystemType()  # "windows", "linux", "darwin", etc.
MACHINE_TYPE = detectMachineType()  # "RK3588", "RK3576", "RDK_X5", etc.

# 设置 1ms 的定时器精度，减少延迟（仅 Windows）
if sys.platform.startswith("win"):
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)