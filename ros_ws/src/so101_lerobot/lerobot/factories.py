from __future__ import annotations

"""Factory helpers for on-demand construction of control and robot interfaces.

At runtime we only import the concrete implementation that is requested. This
keeps optional heavy dependencies (e.g. VR SDKs, haptic drivers) out of the
critical path for users that do not need them.
"""

from typing import TYPE_CHECKING

# -----------------------------------------------------------------------------
# Control interfaces
# -----------------------------------------------------------------------------

def _lazy_import_vr() -> "ControlInterface":
    from .control.vr_control import VRControlInterface  # noqa: F401
    return VRControlInterface()

def _lazy_import_keyboard() -> "ControlInterface":
    from .control.keyboard_control import KeyboardControlInterface  # noqa: F401
    return KeyboardControlInterface()

def _lazy_import_gamepad() -> "ControlInterface":
    from .control.gamepad_control import GamepadControlInterface  # noqa: F401
    return GamepadControlInterface()

def _lazy_import_leader_follower() -> "ControlInterface":
    from .control.leader_follower_control import LeaderFollowerControlInterface  # noqa: F401
    return LeaderFollowerControlInterface()


class ControlFactory:
    """Dynamically construct *ControlInterface* implementations."""

    _CREATORS = {
        "vr": _lazy_import_vr,
        "keyboard": _lazy_import_keyboard,
        "gamepad": _lazy_import_gamepad,
        "leader_follower": lambda: _lazy_import_leader_follower(),
    }

    @classmethod
    def create(cls, control_type: str):
        control_type = control_type.lower()
        try:
            return cls._CREATORS[control_type]()
        except KeyError as exc:
            raise ValueError(f"Unknown control interface '{control_type}'.") from exc


# -----------------------------------------------------------------------------
# Robot interfaces
# -----------------------------------------------------------------------------

def _lazy_import_so101() -> "RobotInterface":
    from .robots.so101_interface import SO101Interface  # noqa: F401
    return SO101Interface()

def _lazy_import_ur5() -> "RobotInterface":
    from .robots.ur5_interface import UR5Interface  # noqa: F401
    return UR5Interface()

def _lazy_import_franka() -> "RobotInterface":
    from .robots.franka_interface import FrankaInterface  # noqa: F401
    return FrankaInterface()


class RobotFactory:
    _CREATORS = {
        "so101": _lazy_import_so101,
        "ur5": _lazy_import_ur5,
        "franka": _lazy_import_franka,
    }

    @classmethod
    def create(cls, robot_type: str):
        robot_type = robot_type.lower()
        try:
            return cls._CREATORS[robot_type]()
        except KeyError as exc:
            raise ValueError(f"Unknown robot type '{robot_type}'.") from exc 