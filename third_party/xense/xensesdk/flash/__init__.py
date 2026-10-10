from pathlib import Path

FLASH_DIR = Path(__file__).resolve().parent

from .client import create_flash_client

__all__ = ["create_flash_client", "FLASH_DIR"]
