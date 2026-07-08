import time
import yaml
import logging
logging.basicConfig(level=logging.WARNING, format="%(message)s")
from pathlib import Path
from typing import Dict, Any
from interface import FrankaConfig, Franka
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say

class ReplayConfig:
    def __init__(self, cfg: Dict[str, Any]):
        robot = cfg["robot"]

        # global config
        self.dataset_name: str = cfg["dataset_name"]
        self.episode_idx: int = cfg.get("episode_idx", 0)

        # robot config
        self.robot_ip: str = robot["ip"]

def run_replay(replay_cfg: ReplayConfig):
    episode_idx = replay_cfg.episode_idx

    robot_config = FrankaConfig(
        robot_ip=replay_cfg.robot_ip,
        debug = False,
        gripper_reverse = False,
    )
    
    robot = Franka(robot_config)
    robot.connect()
    dataset = LeRobotDataset(replay_cfg.dataset_name, episodes=[episode_idx])
    # standard datasets store the vector under "action"; legacy-schema ones under "actions"
    action_key = "action" if "action" in dataset.features else "actions"
    actions = dataset.hf_dataset.select_columns(action_key)
    log_say(f"Replaying episode {episode_idx}")
    for idx in range(dataset.num_frames):
        t0 = time.perf_counter()
        action = {
            name: float(actions[idx][action_key][i]) for i, name in enumerate(dataset.features[action_key]["names"])
        }
        robot.send_action(action)

        precise_sleep(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))

    robot.disconnect()

def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "record_config.yaml"
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    replay_cfg = ReplayConfig(cfg["replay"])

    run_replay(replay_cfg)