import cypack; cypack.init(__name__, set([]))

import os
import sys
from pathlib import Path
from typing import Any, TYPE_CHECKING

PACKAGE_DIR = _VENDOR_ROOT if "_VENDOR_ROOT" in globals() else Path(__file__).resolve().parent
ASSETS_DIR = PACKAGE_DIR / "assets"
FONTS_DIR = ASSETS_DIR / "fonts"

__all__ = ["App", "AppConfig", "imgui_ctx", "PACKAGE_DIR", "ASSETS_DIR", "FONTS_DIR"]

# 保持 imgui_ctx 是真实模块属性，兼容 `from xensesdk.retiles import imgui_ctx as ix`。
from . import imgui_ctx as imgui_ctx  # noqa: E402

if TYPE_CHECKING:
    from .app import App, AppConfig


def _force_redux_imgui() -> None:  # pragma: no cover
    """
    优先加载 py-imgui-redux 扩展模块，避免环境里命中其他同名 imgui 包。
    """
    import importlib.machinery
    import importlib.util

    mode = os.environ.get("RETILES_IMGUI", "redux").strip().lower()
    if mode in ("pyimgui", "default", "auto"):
        return

    if "imgui" in sys.modules:
        # Somebody imported an `imgui` already; don't try to swap it out.
        return

    exts: tuple[str, ...] = (".pyd", ".so", ".dylib")
    candidates: list[Path] = []
    for entry in sys.path:
        try:
            base = Path(entry)
        except Exception:
            continue
        if not base.exists() or not base.is_dir():
            continue
        for ext in exts:
            candidates.extend(sorted(base.glob(f"imgui*{ext}")))

    if not candidates:
        return

    def _score(path: Path) -> tuple[int, int]:
        name = path.name.lower()
        return (0 if ("cpython" in name or "cp" in name) else 1, len(name))

    best = sorted(candidates, key=_score)[0]

    loader = importlib.machinery.ExtensionFileLoader("imgui", str(best))
    spec = importlib.util.spec_from_file_location("imgui", str(best), loader=loader)
    if spec is None:
        return
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    sys.modules["imgui"] = module


def __getattr__(name: str) -> Any:  # pragma: no cover
    if name in ("App", "AppConfig"):
        from .app import App, AppConfig

        return App if name == "App" else AppConfig
    raise AttributeError(name)


def __dir__() -> list[str]:  # pragma: no cover
    return sorted(__all__)


_force_redux_imgui()
import imgui