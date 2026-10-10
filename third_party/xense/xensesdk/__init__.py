import cypack; cypack.init(__name__, set(['_plugin', 'ezgl', 'retiles']))
from typing import Any, TYPE_CHECKING
from pathlib import Path

PROJ_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PROJ_DIR / "config"
CONFIG_CACHE_DIR = Path.home() / ".xensesdk" / "config"

if TYPE_CHECKING:
    from .api.api import Sensor
    from .viz import ExampleView
    # from .ezros import call_service

from .core.ctx_builders import find_xpack_path
from .core.hooks import get_hooks

def __getattr__(name: str) -> Any:
    """
    延迟导入重依赖模块（如 av/Qt 等），避免仅使用配置/工具代码时被强依赖阻塞。
    """
    if name == "find_xpack_path":
        from .core.ctx_builders import find_xpack_path as find_xpack_path
        return find_xpack_path

    if name == "Sensor":
        from xensesdk.api.api import Sensor as Sensor
        return Sensor

    if name == "ExampleView":
        from xensesdk.viz import ExampleView as ExampleView
        return ExampleView

    if name == "call_service":
        from xensesdk.ezros import call_service as call_service
        return call_service

    raise AttributeError(f"module 'xensesdk' has no attribute '{name}'")
