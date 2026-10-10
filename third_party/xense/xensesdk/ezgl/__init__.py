import cypack; cypack.init(__name__, set([]))

import importlib
import sys
import warnings


def _is_retiles_glfw(module) -> bool:
    if module is None:
        return False
    return getattr(module, "__name__", "") in {
        "xensesdk.retiles.integrations.ezgl",
        "retiles.integrations.ezgl",
    }


def _ensure_retiles_glfw() -> None:
    glfw_module = sys.modules.get("glfw")
    if _is_retiles_glfw(glfw_module):
        return

    try:
        importlib.import_module("xensesdk.retiles.integrations.ezgl")
    except Exception as exc:
        warnings.warn(f"Failed to install retiles glfw shim before ezgl import: {exc}")
        return
    
_ensure_retiles_glfw()
