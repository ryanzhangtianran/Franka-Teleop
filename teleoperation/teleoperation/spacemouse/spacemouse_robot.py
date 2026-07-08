from typing import Dict, Sequence
import numpy as np

from .spacemouse_expert import SpaceMouseExpert
from .robot import Robot


class SpaceMouseRobot(Robot):
    """A class representing a SpaceMouse robot."""

    def __init__(
        self,
        use_gripper: bool = True,
        pose_scaler: Sequence[float] = [1.0, 1.0],
        channel_signs: Sequence[bool] = [1, 1, 1, 1, 1, 1],
    ):  
        
        self._use_gripper = use_gripper
        self._pose_scaler = pose_scaler
        self._channel_signs = channel_signs
        self._expert = SpaceMouseExpert()
        self._last_gripper_position = 1.0  # 默认夹爪张开状态

        
    def num_dofs(self) -> int:
        if self._use_gripper:
            return 7
        else:
            return 6

    def get_action(self) -> np.ndarray:
        """
        Return the current robot actions including gripper control.
        """
        action, buttons = self._expert.get_action()
        
        if len(action) >= 6:
            delta_ee_pose = action[:6]  # [x, y, z, rx, ry, rz]
        else:
            delta_ee_pose = np.zeros(6) 

        if len(self._pose_scaler) >= 2:
            position_scale, orientation_scale = self._pose_scaler[0], self._pose_scaler[1]
        else:
            raise ValueError(
                f"pose_scaler must have [position, orientation] entries, got: {self._pose_scaler}"
            )

        scales = [position_scale] * 3 + [orientation_scale] * 3
        for i in range(6):
            delta_ee_pose[i] = delta_ee_pose[i] * scales[i] * self._channel_signs[i]

        if self._use_gripper and len(buttons) >= 2:
            left_button = buttons[0]  
            right_button = buttons[1]  
            
            if left_button:
                gripper_position = 1.0  
            elif right_button:
                gripper_position = 0.0  
            else:
                gripper_position = self._last_gripper_position  
            
            self._last_gripper_position = gripper_position
        else:
            gripper_position = self._last_gripper_position  

        if self._use_gripper:
            return np.concatenate([delta_ee_pose, [gripper_position]])
        else:
            return delta_ee_pose

    def get_observations(self) -> Dict[str, np.ndarray]:
        """
        Return the current robot observations by formatting the action data.
        """
        action_data = self.get_action()
        
        obs_dict = {}
        axes = ["x", "y", "z", "rx", "ry", "rz"]
        
        if len(action_data) >= 6:
            for i, axis in enumerate(axes):
                obs_dict[f"delta_ee_pose.{axis}"] = float(action_data[i])
        else:
            for axis in axes:
                obs_dict[f"delta_ee_pose.{axis}"] = float(0.0)
        
        # Only emit the gripper key when it is a real value; a None here would
        # crash the robot-side gripper handling and break the dataset schema.
        if self._use_gripper and len(action_data) >= 7:
            obs_dict["gripper_cmd_bin"] = float(action_data[6])

        return obs_dict