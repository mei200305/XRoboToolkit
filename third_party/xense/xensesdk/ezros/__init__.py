"""
EzROS - 基于 CycloneDDS 的 ROS2 风格通信框架
"""

from pathlib import Path

from . import xbin


EZROS_DIR = Path(__file__).resolve().parent
CYCLONEDDS_CONFIG = EZROS_DIR / "CycloneDDS.xml"

__version__ = "0.1.0"
__author__ = "Liu Jin"
__description__ = "基于 CycloneDDS 的 ROS2 风格通信框架"

_NODE_EXPORTS = (
    "Node",
    "Publisher",
    "Subscriber",
    "ServiceServer",
    "ServiceClient",
    "call_service",
)
_MESSAGE_EXPORTS = (
    "Message",
    "ServiceRequest",
    "ServiceResponse",
    "BytesMessage",
    "StringMessage",
    "PureBytesMessage",
)
_LOGGER_EXPORTS = (
    "Logger",
    "get_logger",
    "init_logging",
)
_CONTEXT_EXPORTS = (
    "Rate",
    "Timer",
    "Context",
    "init",
    "shutdown",
    "ok",
    "spin",
    "create_rate",
    "wait_for_condition",
    "setup_multi_net_interface",
)
_MONITOR_EXPORTS = (
    "PublisherMonitor",
    "monitor_publisher",
    "unmonitor_publisher",
    "get_global_monitor",
    "get_topic_type",
    "scan_node",
)

__all__ = [
    *_NODE_EXPORTS,
    *_MESSAGE_EXPORTS,
    *_LOGGER_EXPORTS,
    *_CONTEXT_EXPORTS,
    *_MONITOR_EXPORTS,
    "xbin",
    "EZROS_DIR",
    "CYCLONEDDS_CONFIG",
    "__version__",
    "__author__",
    "__description__",
]

_network_ready = False


def __getattr__(name):
    if name in _NODE_EXPORTS:
        _setup_network_if_needed()
        from . import node

        value = getattr(node, name)
    elif name in _MESSAGE_EXPORTS:
        _setup_network_if_needed()
        from . import message

        value = getattr(message, name)
    elif name in _LOGGER_EXPORTS:
        from . import logger

        value = getattr(logger, name)
    elif name in _CONTEXT_EXPORTS:
        if name != "setup_multi_net_interface":
            _setup_network_if_needed()
        from . import context

        value = getattr(context, name)
    elif name in _MONITOR_EXPORTS:
        _setup_network_if_needed()
        from .cli import publisher_monitor

        value = getattr(publisher_monitor, name)
    else:
        raise AttributeError(name)
    globals()[name] = value
    return value


def _setup_network_if_needed():
    global _network_ready
    if _network_ready:
        return
    from .context import setup_multi_net_interface

    if _machine_type() not in ("rk3588", "rk3576"):
        setup_multi_net_interface()
    _network_ready = True


def _machine_type():
    compatible = Path("/sys/firmware/devicetree/base/compatible")
    if not compatible.exists():
        return "x86"
    text = compatible.read_text(errors="ignore").lower()
    if "3588" in text:
        return "rk3588"
    if "3576" in text:
        return "rk3576"
    return "arm"
