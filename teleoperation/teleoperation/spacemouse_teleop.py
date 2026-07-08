#!/usr/bin/env python

"""
SpaceMouse teleoperation implementation.
"""

import logging
from typing import Any, Dict

from lerobot.teleoperators.teleoperator import Teleoperator
from .config_teleop import SpacemouseTeleopConfig
from .spacemouse.spacemouse_robot import SpaceMouseRobot

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SpacemouseTeleop(Teleoperator):
    """
    Teleoperation using SpaceMouse.

    Controls the robot's end-effector in Cartesian space; the output is a
    delta pose (position and orientation changes) plus a binary gripper command.
    """

    config_class = SpacemouseTeleopConfig
    name = "SpacemouseTeleop"

    def __init__(self, config: SpacemouseTeleopConfig):
        super().__init__(config)
        self.cfg = config
        self._is_connected = False
        self.spacemouse_robot: SpaceMouseRobot = None

    @property
    def action_features(self) -> dict:
        """Return action features (delta ee pose + binary gripper)."""
        features = {}
        for axis in ["x", "y", "z", "rx", "ry", "rz"]:
            features[f"delta_ee_pose.{axis}"] = float
        if self.cfg.use_gripper:
            features["gripper_cmd_bin"] = float
        return features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        """Connect to the SpaceMouse."""
        if self._is_connected:
            logger.warning(f"{self.name} is already connected.")
            return

        logger.info(f"\n===== [TELEOP] Connecting to {self.name} =====")
        self.spacemouse_robot = SpaceMouseRobot(
            use_gripper=self.cfg.use_gripper,
            pose_scaler=self.cfg.pose_scaler,
            channel_signs=self.cfg.channel_signs,
        )
        actions = self.spacemouse_robot.get_action()
        formatted_actions = [round(float(j), 4) for j in actions]
        logger.info(f"[TELEOP] Current ee pose actions: {formatted_actions}")
        self._is_connected = True
        logger.info(f"===== [TELEOP] {self.name} connected successfully =====\n")

    def disconnect(self) -> None:
        """Disconnect from the SpaceMouse."""
        if not self._is_connected:
            return
        if self.spacemouse_robot is not None:
            self.spacemouse_robot._expert.close()
        self._is_connected = False
        logger.info(f"[INFO] ===== {self.name} disconnected =====")

    def get_action(self) -> Dict[str, Any]:
        """Get the current delta pose action from the SpaceMouse."""
        if not self._is_connected:
            raise RuntimeError(f"{self.name} is not connected.")
        return self.spacemouse_robot.get_observations()

    def calibrate(self) -> None:
        """Calibrate the device. Default: no-op."""
        pass

    def configure(self) -> None:
        """Configure the device. Default: no-op."""
        pass

    def send_feedback(self, feedback: Dict[str, Any]) -> None:
        """Send feedback to the device. Default: no-op."""
        pass
