# Franka Teleoperation Data Collection with Isomorphic Joint Mapping and End-Effector Pose

## Introduction 
This project is a teleoperation data collection tool for Franka Research 3 robot (should compatible with Franka Panda), embeded in [LeRobot](https://github.com/huggingface/lerobot.git).

### Support

Now we support the end-effector pose control using [SpaceMouse](https://3dconnexion.com/) devices.

<p align="center">
  <img src="assets/spacemouse.png" alt="Record" width="600">
  <br>
</p>

 The SpaceMouse is a 6-axis input device that can be used to control the end pose of the robot. The left button is used to open the gripper, and the right button is used to close the gripper.

## Quick Start — 本机使用说明(SpaceMouse 采数)

> 本节是当前部署机器的日常操作速查(环境已于 2026-06-12 搭建并全链路验证)。首次部署或换机器请从第 0 节开始。

### 本机环境一览

| 项目 | 值 |
|---|---|
| conda 环境 | `franka`(Python 3.10,lerobot 0.3.4 Orbbec fork + franka_teleop) |
| NUC(机器人主机) | `192.168.50.10`,运行 Polymetis + `server.py`(zerorpc 端口 4242) |
| 外部相机 | Orbbec Gemini 336L,序列号 `CPC936300041` |
| 腕部相机 | Orbbec Gemini 336L,序列号 `CPC93630001C` |
| 遥操作设备 | 3Dconnexion SpaceMouse Compact |
| 数据集存储 | `~/.cache/huggingface/lerobot/<repo_id>_<日期>_<版本>` |

### 采数前准备(每次)

1. **NUC 端**:依次启动 Polymetis 三件套和 zerorpc 桥接(见第 0.2 节):
   `launch_robot.py` → `launch_gripper.py` → `launch_server.py` → `server.py`
2. **机器人**:上电、急停释放、关节解锁,在 Franka Desk 中激活 **FCI**。
3. **本机**:编辑 `scripts/config/record_config.yaml`,设置本次任务:
   - `repo_id`:按第 9 节命名规范(`<user>/<Verb_Object_prep_Target>`)
   - `task.description`:任务的自然语言描述(训练时作为 language instruction)
   - `task.num_episodes`:本次计划录制条数

### 启动采集

```bash
/data2/franka/franka_teleop/scripts/start_record.sh          # 预检(5项)通过后自动进入录制
/data2/franka/franka_teleop/scripts/start_record.sh --check  # 只做预检不启动
```

脚本自动完成:激活 conda 环境 → 挂载图形会话(`DISPLAY=:2`,rerun 画面显示在物理屏幕)→ 预检 NUC/SpaceMouse/相机/机械臂 → 启动 `franka-record`。
**必须在能交互的终端里运行**(SSH 或物理机均可),回合间复位需要在该终端按 Enter。

### 操作方法

| 操作 | 效果 |
|---|---|
| 推/转 SpaceMouse | 控制末端 6 自由度。**注意:存在动作死区,推杆需过半行程机械臂才响应**(轻推会被静默忽略,属设计行为) |
| SpaceMouse 左键 | 打开夹爪 |
| SpaceMouse 右键 | 闭合夹爪 |
| **→** 右方向键(第 1 次) | 结束当前 episode 并保存,程序暂停 |
| **→**(第 2 次) | 机械臂自动回 Home 复位(期间可摆放物体) |
| **→**(第 3 次) | 结束复位,开始录下一条 |
| **←** 左方向键 | 放弃并重录当前 episode |
| **Esc** | 结束会话,保存全部已录数据 |
| Ctrl+C / 异常 | 进入清理流程,询问是否删除不完整数据集(误删可在回收站找回) |

> 所有按键都在**物理机键盘**上按(全局监听),录数据全程无需回到启动终端。

### 录完之后

```bash
# 回放 / 可视化(先在 record_config.yaml 的 replay/visualize 段填入实际数据集名,含日期版本后缀)
franka-replay
franka-visualize
tools-check-dataset   # 手动删过数据集后,刷新 dataset_info.txt
```

### 常见问题速查

| 现象 | 原因与处理 |
|---|---|
| 推 SpaceMouse 机械臂不动 | ① 推力不够:死区要求推杆过半(见上表);② 看终端日志有无 `[ROBOT] zerorpc error`(执行失败会被吞成 warning,表面无异常);③ 跑 `start_record.sh --check` 复查链路 |
| 预检报 NUC 不可达 | NUC 上四个服务没起全;本机验证:`nc -z 192.168.50.10 4242` |
| 外部相机枚举时报 `libusb ... timed out` | 该相机在 USB2 口,告警无害(15fps 实测不掉帧);介意可换到 USB3 口 |
| 提示 X display 不可用 | 物理机图形会话(`:2`)不在了,需有人在物理机登录 franka 用户;或改用 VNC |
| 给环境装新包后各种报错 | 大概率 numpy 被降级。本环境必须保持 `numpy==2.2.6`/`av==15.x`,严禁安装 open3d(详见维护者) |
| 远程操作 | 笔记本上 `ssh -L 5900:localhost:5900 franka@<本机>`,本机 `x11vnc -display :2 -auth /run/user/1007/gdm/Xauthority -localhost -nopw`,VNC 连 `localhost:5900` |

## 0. Franka Control via Polymetis

### 0.1 Install [Polymetis](https://polymetis-docs.github.io/) on NUC

1. Requirement of libfranka:
```bash
wget https://github.com/frankarobotics/libfranka/releases/download/<libfranka_version>/libfranka_<libfranka_version>_<ubuntu_codename>_amd64.deb
sudo dpkg -i libfranka_<libfranka_version>_<ubuntu_codename>_amd64.deb
```

Choose a specific version of libfranka according to your machine.

The following example installs **libfranka 0.19.0** on **Ubuntu 22.04 (Jammy)**:
```bash
wget https://github.com/frankarobotics/libfranka/releases/download/0.19.0/libfranka_0.19.0_jammy_amd64.deb
sudo dpkg -i libfranka_0.19.0_jammy_amd64.deb
```

2. Clone repo:
```bash
git clone https://github.com/ryanzhangtianran/polymetis.git
cd polymetis
```

3. Create an environment:
```bash
conda env create -f ./polymetis/environment.yml
conda activate polymetis
```

4. Install
```bash
cd polymetis
pip install -e .

mkdir build && cd build
cmake -DBUILD_FRANKA=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=ON ..
make -j32
```

### 0.2. Usage
If you want to use the interface to control the Franka robot, follow the steps below:

```bash
cd polymetis/polymetis/python/scripts

python launch_robot.py robot_client=franka_hardware
python launch_gripper.py gripper=franka_hand
python launch_server.py
```

And run `sudo pkill -9 run_server` after launching the robot.

## 1. Environment Setup
### 1.1 Create and Activate Conda Environment
```bash
conda create -n franka python=3.10
conda activate franka
```

### 1.2 Install lerobot
```bash
# Install specific version 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
pip install -e .
```

If you want to use Orbbec series cameras as 3-view and wrist cameras, please use the following specific version of Lerobot:
```bash
git clone https://github.com/ryanzhangtianran/lerobot_0.3.4.git
cd lerobot
pip install -e .
```

### 1.3 Clone and Install Franka Teleoperation Control Source
```bash
git clone https://github.com/ryanzhangtianran/franka_teleop.git
cd franka_teleop
pip install -e .
```

### 1.4 View Available Commands
Run the following command to display all available scrtips:
```bash
franka-help
# ==================================================
#  Franka Teleoperation Utilities - Command Reference
# ==================================================

# Core Commands:
#   franka-record           Record teleoperation dataset
#   franka-replay           Replay a recorded dataset
#   franka-visualize        Visualize recorded dataset
#   franka-reset           Reset the robot to initial state
#   franka-train          Train a policy on the recorded dataset

# Tool Commands:
#   tools-check-dataset   Check local dataset information
#   tools-check-rs        Retrieve connected RealSense camera serial numbers

# Shell Tools:
#   map_gripper.sh        Map Gripper Serial Port

# Test Commands:
#   test-gripper-ctrl     Run gripper control command (operate the gripper)

# --------------------------------------------------
#  Tip: Use 'franka-help' anytime to see this summary.
# ==================================================
```

## 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
Please make sure only one camera is connected at a time.
```bash
tools-check-rs
```
Or you can check the serial number in the realsense-viewer.
```
realsense-viewer
```

## 3. Dataset Recording

### 3.1 Upload Data to Hugging Face (Optional)
1. Set the following in `record_config.yaml`:
```yaml
push_to_hub: True
```
2. Obtain your Hugging Face account token and login:
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} 
huggingface-cli whoami  # Verify successful login
```

### 3.2 Start Recording
1. Activate **FCI** on the Franka desk interface.  
2. Verify that the parameters in `record_config.yaml` are correct.  
3. To ensure standardized data collection, please read `9. Dataset Naming and Recording Details` before running the following command:
```bash
franka-record
```

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Record</b>
</p>

## 4. Dataset Replay
```bash
franka-replay # Make sure record_config.yaml is properly configured
```

## 5. Dataset Visualization
```bash
franka-visualize # Make sure record_config.yaml is properly configured
```
<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 2: Visualization</b>
</p>

## 6. Dataset Appending and Resume
If you have already recorded a dataset under the specified `repo_id`, set `resume: True` in `record_config.yaml` and enter the dataset name in `resume_dataset` to continue recording new data on the existing dataset. Then, run the following command:

```bash
franka-record
```

## 7. Merge Datasets
If you recorded data in multiple stages, ensure each dataset has a unique `repo_id`. Use the following command to merge them into a single dataset:
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```
- For more dataset processing commands, see [LeRobot](https://huggingface.co/docs/lerobot/using_dataset_tools).

## 8. Recording Control Keys

- **Right Arrow Key**  
   1. Press the right arrow key: end the current episode and save the data. The program will pause.  
   2. Press the right arrow key again to enter the **reset** phase: the robot returns to its initial position.  
   3. Press the right arrow key once more to finish the reset and start recording the next episode.

- **Left Arrow Key**  
   - Press to restart the current episode recording (overwrite the current data).

- **Esc Key**  
   - Press to exit the recording session and save all recorded episodes.

- **Ctrl+C or Exception**
   - Pressing or on exception: enters exception handling and asks whether to delete the incomplete dataset.
   - If you delete the dataset by accident, you can find it in the trash folder.
  
## 9. Dataset Naming and Recording Details
### 9.1 Dataset Naming
<p align="center">
  <img src="assets/dataset.png" alt="dataset" width="600">
  <br>
  <b>Figure 3: Dataset</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="dataset_info" width="600">
  <br>
  <b>Figure 4: Dataset Info</b>
</p>

1. The dataset is by default stored in the `~/.cache/huggingface/lerobot` directory and contains three types of items:

   - `dataset_info.txt`: Automatically records local dataset information, including the following fields: `record_id`, `name`, `task`, `date`, `version`, `user_info`, and `type`. The `user_info` field can be annotated via the `user_notes` entry in `record_config.yaml`.

   - `dataset_info_backup`: When `tools-check-dataset` is used to manually update `dataset_info.txt`, this folder stores backups of the previous records.

   - Dataset folders: Stores the actual dataset contents.

2. Dataset names follow the format `[description]_[date]_[version]`, where:
   - `description` comes from `record_config.yaml` as `repo_id=<user_name>/<description>`;
   - `date` is generated automatically;
   - `version` is incremented automatically if a dataset with the same `repo_id` already exists.
3. Description naming rule: `task.description -> Verb_SourceObject_prep_TargetObject`.  
   For example:  
   `Pick up the green cube and put it into the trash bin -> pick_greencube_into_trashbin`.

### 9.2 Recording Instructions
1. Complete step `2. Get and configure required parameters`.
2. Fill in the task instruction in `record_config.yaml` under `task.description` and set `repo_id` according to the dataset naming rules.
3. Check and adjust other parameters in `record_config.yaml` to ensure correctness.
4. Run `franka-record`. Then follow `8. Recording Control Keys` to complete the recording.
5. After recording, a `dataset_info.txt` file will be automatically created in the same directory to store local dataset information. If you have manually deleted any datasets, update the `dataset_info.txt` using:
```bash
tools-check-dataset
```

## 10. Training and Evaluation Using Collected Datasets
### 10.1 Training
Config the `train_config.yaml` file, including the following fields:
1. dataset_repo_id: The repository ID of the dataset to use for training.
2. policy.type: The policy type to use for training. You can choose from `diffusion` and `act`.
3. output_dir: The directory to save the training outputs.
4. job_name: The name of the training job.
5. batch_size: The batch size for training.
6. steps: The total number of training steps.

Then run the following command to start training:
```bash
franka-train
```

### 10.2 Evaluation
To evaluate the trained policy, config the `record_config.yaml` file, including the following fields:
1. run_mode: Set to `run_policy` to enable evaluation mode.
2. policy.type: The policy type to use for evaluation. You can choose from `diffusion` and `act`. Make sure it is the same as the policy type used for training.
3. policy.checkpoint_path: The path to the trained policy checkpoint. The path should include `config.json`.

Then run the following command to start evaluation:
```bash
franka-record
```
