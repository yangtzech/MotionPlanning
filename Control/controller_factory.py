"""
简单的控制器注册与工厂
"""

from typing import Callable, Dict

_REGISTRY: Dict[str, Callable] = {}


def register_controller(name: str):
    def _decorator(fn: Callable):
        _REGISTRY[name] = fn
        return fn

    return _decorator


def get_controller(name: str):
    """返回已注册的控制器构造器或适配函数

    如果未找到则抛出 KeyError
    """
    # lazy import adapters to populate registry if empty
    if not _REGISTRY:
        try:
            # import the adapters module which registers controllers via decorator
            import importlib

            importlib.import_module("Control.controllers_adapters")
        except Exception:
            # ignore import error here; if adapters can't be imported, we'll raise below
            pass

    if name not in _REGISTRY:
        raise KeyError(f"Controller '{name}' is not registered")

    return _REGISTRY[name]


def list_controllers():
    return list(_REGISTRY.keys())
