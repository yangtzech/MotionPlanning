"""Controllers subpackage for pypnc.

This module purposefully re-exports the commonly used controller symbols
so downstream code can import from ``pypnc.controllers`` directly.
"""

from . import pure_pursuit as _pp

# Re-export the controller API we want to be stable. Keep these names small
# and focused - callers can still import deeper symbols if they need to.
Node = _pp.Node
PATH = _pp.PATH
pure_pursuit = _pp.pure_pursuit
pid_control = _pp.pid_control
generate_path = _pp.generate_path

__all__ = [
    "Node",
    "PATH",
    "pure_pursuit",
    "pid_control",
    "generate_path",
]
