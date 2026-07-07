from setuptools import setup, find_packages
from pathlib import Path

# ====== Project root ======
ROOT = Path(__file__).parent.resolve()

setup(
    name="franka_teleop",
    version="0.1.0",
    description="Franka teleoperation and dataset collection utilities",
    author="Zhaolong Shen, Ryan Zhang",
    author_email="shenzhaolong@buaa.edu.cn, ryanzhangtianran@gmail.com",
    python_requires=">=3.10",
    packages=find_packages(where=".", include=["scripts*", "scripts.*"]),
    include_package_data=True,
    install_requires=[
        "send2trash",
        f"interface @ {(ROOT / 'interface').as_uri()}",
        f"teleoperation @ {(ROOT / 'teleoperation').as_uri()}",
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
    ],
    entry_points={
        "console_scripts": [
            # core commands
            "franka-record = scripts.core.record:main",
            "franka-replay = scripts.core.replay:main",
            "franka-visualize = scripts.core.visualize:main",
            "franka-reset = scripts.core.reset:main",
            "franka-train = scripts.core.train:main",

            # tools commands (helper tools)
            "tools-check-dataset = scripts.tools.check_dataset_info:main",
            "tools-check-rs = scripts.tools.rs_devices:main",

            # test commands (testing scripts)
            "test-gripper-ctrl = scripts.test.gripper_ctrl:main",
            # unified help command
            "franka-help = scripts.help.help:main",
        ]
    },
)
