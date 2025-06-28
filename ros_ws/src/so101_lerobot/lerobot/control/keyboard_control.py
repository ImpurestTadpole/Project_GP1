from __future__ import annotations

import pygame
from typing import Dict, Any

from ..control_interface import ControlInterface

# ----------------------------------------------------------------------------
# Helper dataclass for holding 6-DoF pose information -------------------------
# ----------------------------------------------------------------------------

class Pose6D:
    """Simple container for position (x,y,z) and orientation (rx,ry,rz)."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 rx: float = 0.0, ry: float = 0.0, rz: float = 0.0):
        self.x, self.y, self.z = x, y, z
        self.rx, self.ry, self.rz = rx, ry, rz

    # Factory helper ----------------------------------------------------------
    @classmethod
    def identity(cls) -> "Pose6D":
        return cls()

    def as_tuple(self):
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)

    # Arithmetic helpers ------------------------------------------------------
    def __add__(self, other: "Pose6D") -> "Pose6D":
        return Pose6D(*(a + b for a, b in zip(self.as_tuple(), other.as_tuple())))

    def __repr__(self) -> str:  # pragma: no cover
        return (
            f"Pose6D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
            f"rx={self.rx:.3f}, ry={self.ry:.3f}, rz={self.rz:.3f})"
        )


# ----------------------------------------------------------------------------
# Keyboard implementation ----------------------------------------------------
# ----------------------------------------------------------------------------

class KeyboardControlInterface(ControlInterface):
    """Very simple keyboard teleoperation backend.

    WASD keys move the end-effector in the XY-plane, QE move along Z. The arrow
    keys can optionally be mapped to orientation control later.
    """

    _STEP_TRANSL = 0.01  # metre per key press

    def __init__(self) -> None:
        pygame.init()
        # We need a window for pygame to capture keyboard events.
        pygame.display.set_mode((200, 200))
        pygame.display.set_caption("LeRobot Keyboard Teleop")

        self._pose = Pose6D.identity()
        self._gripper = 0.0

    # ---------------------------------------------------------------------
    # Required ControlInterface API --------------------------------------
    # ---------------------------------------------------------------------

    def get_pose(self) -> Pose6D:
        self._process_events()
        return self._pose

    def get_gripper_state(self) -> float:
        return self._gripper

    def get_user_commands(self) -> Dict[str, Any]:
        # For this simple backend we do not expose any extra commands.
        return {}

    def provide_feedback(self, feedback: Any):
        # No feedback mechanism for basic keyboard backend.
        pass

    # ------------------------------------------------------------------
    # Internal helpers --------------------------------------------------
    # ------------------------------------------------------------------

    def _process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit
            elif event.type == pygame.KEYDOWN:
                self._handle_key(event.key)

    def _handle_key(self, key: int):
        if key == pygame.K_w:
            self._pose.y += self._STEP_TRANSL
        elif key == pygame.K_s:
            self._pose.y -= self._STEP_TRANSL
        elif key == pygame.K_a:
            self._pose.x -= self._STEP_TRANSL
        elif key == pygame.K_d:
            self._pose.x += self._STEP_TRANSL
        elif key == pygame.K_q:
            self._pose.z += self._STEP_TRANSL
        elif key == pygame.K_e:
            self._pose.z -= self._STEP_TRANSL
        elif key == pygame.K_o:  # open gripper
            self._gripper = min(1.0, self._gripper + 0.05)
        elif key == pygame.K_p:  # close gripper
            self._gripper = max(0.0, self._gripper - 0.05) 