import cypack; cypack.init(__name__, set([]))
from .xense_gripper import XenseGripper
from .node.camera_client import CameraNode as XenseCamera
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent

from ._version import __version__
__all__ = ["__version__"]