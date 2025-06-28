from __future__ import annotations

import pygame
from typing import Dict, Any

from ..control_interface import ControlInterface
from .keyboard_control import Pose6D  # Reuse simple Pose6D helper


class GamepadControlInterface(ControlInterface):
    """Basic gamepad interface using pygame's joystick module."""

    _AXIS_DEADZONE: float = 0.1
    _SCALE_TRANSL: float = 0.05  # metres per full axis deflection

    def __init__(self, device_index: int = 0):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected. Make sure it is plugged in and recognised by the OS.")
        self._joy = pygame.joystick.Joystick(device_index)
        self._joy.init()

        self._pose = Pose6D.identity()
        self._gripper = 0.0

    # ------------------------------------------------------------------
    # Required ControlInterface API ------------------------------------
    # ------------------------------------------------------------------

    def get_pose(self) -> Pose6D:
        self._process_events()
        return self._pose

    def get_gripper_state(self) -> float:
        return self._gripper

    def get_user_commands(self) -> Dict[str, Any]:
        return {}

    def provide_feedback(self, feedback: Any):
        pass  # Most consumer gamepads have no force feedback accessible from pygame

    # ------------------------------------------------------------------
    # Internal helpers --------------------------------------------------
    # ------------------------------------------------------------------

    def _process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit

        # Continuous axes readout
        lx = self._apply_deadzone(self._joy.get_axis(0))  # Left stick X
        ly = self._apply_deadzone(-self._joy.get_axis(1))  # Left stick Y (invert)
        rz = self._apply_deadzone(self._joy.get_axis(3))  # Right stick X (yaw)
        ry = self._apply_deadzone(-self._joy.get_axis(4))  # Right stick Y (pitch)
        lt = (self._joy.get_axis(2) + 1.0) / 2.0  # Trigger in [0,1]
        rt = (self._joy.get_axis(5) + 1.0) / 2.0

        self._pose.x += lx * self._SCALE_TRANSL
        self._pose.y += ly * self._SCALE_TRANSL
        self._pose.z += (rt - lt) * self._SCALE_TRANSL

        # Buttons for gripper control
        if self._joy.get_button(4):  # LB
            self._gripper = min(1.0, self._gripper + 0.02)
        if self._joy.get_button(5):  # RB
            self._gripper = max(0.0, self._gripper - 0.02)

    def _apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self._AXIS_DEADZONE else v 