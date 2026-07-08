from dataclasses import dataclass, field
from typing import List

from lerobot.teleoperators.config import TeleoperatorConfig


# Both registration names map to the single teleop config, so any externally
# serialized config referencing either name still deserializes.
@TeleoperatorConfig.register_subclass("lerobot_teleoperator_franka")
@TeleoperatorConfig.register_subclass("spacemouse_teleop")
@dataclass
class SpacemouseTeleopConfig(TeleoperatorConfig):
    """Configuration for SpaceMouse teleoperation."""
    use_gripper: bool = True
    pose_scaler: List[float] = field(default_factory=lambda: [1.0, 1.0])  # [position_scale, orientation_scale]
    channel_signs: List[int] = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])  # [x, y, z, rx, ry, rz]
