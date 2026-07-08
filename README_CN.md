# Franka 遥操作数据采集（同构关节映射 + 末端位姿控制）

[English](README.MD) | 中文

## 简介

本项目是面向 Franka Research 3 机器人的遥操作数据采集工具，基于 [LeRobot](https://github.com/huggingface/lerobot.git) 构建。

它使用 [SpaceMouse](https://3dconnexion.com/) 进行 6 自由度末端控制，配合两台 Orbbec（或 RealSense）相机采集外部视角和腕部视角，以标准 LeRobot 格式录制数据集。机器人控制链路为：机器人旁的 NUC 上运行 [Polymetis](https://polymetis-docs.github.io/)，通过 zerorpc 桥接到本工作站。

<p align="center">
  <img src="assets/spacemouse.png" alt="SpaceMouse" width="600">
  <br>
</p>

SpaceMouse 是一个 6 轴输入设备，用于控制机器人末端位姿。左键打开夹爪，右键闭合夹爪。

## 系统概览

| 组件 | 值 |
|---|---|
| 工作站 conda 环境 | `franka`（Python 3.12，lerobot 0.6.1，本包） |
| NUC（机器人主机） | `192.168.50.10` — 运行 Polymetis + `server.py`（zerorpc，端口 4242），统一放在名为 `franka` 的 tmux 会话中 |
| 机器人 | Franka Research 3，控制接口 `172.16.0.2`（Franka Desk） |
| 外部相机 | Orbbec Gemini 336L |
| 腕部相机 | Orbbec Gemini 336L |
| 遥操作设备 | 3Dconnexion SpaceMouse Compact |
| 数据集存储 | `~/.cache/huggingface/lerobot/<user>/<name>_<日期>_<版本>` |

## 安装

### NUC 端：Polymetis

1. 安装 libfranka（根据机器人和系统选择版本；示例为 Ubuntu 22.04 上安装 libfranka 0.19.0）：

```bash
wget https://github.com/frankarobotics/libfranka/releases/download/0.19.0/libfranka_0.19.0_jammy_amd64.deb
sudo dpkg -i libfranka_0.19.0_jammy_amd64.deb
```

2. 克隆并编译 Polymetis：

```bash
git clone https://github.com/ryanzhangtianran/polymetis.git
cd polymetis
conda env create -f ./polymetis/environment.yml
conda activate polymetis
cd polymetis
pip install -e .

mkdir build && cd build
cmake -DBUILD_FRANKA=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=ON ..
make -j32
```

3. NUC 上需要运行四个服务（日常由工作站上的 `franka-start` 一键拉起，见下文）：

```bash
python launch_robot.py robot_client=franka_hardware   # 机械臂驱动
python launch_gripper.py gripper=franka_hand          # 夹爪驱动
python launch_server.py                               # Polymetis 服务
python server.py                                      # zerorpc 桥接（端口 4242），来自 interface/interface/server.py
```

> `interface/interface/server.py` 运行在 **NUC 上**。在本仓库修改它之后，需要手动同步到 NUC 并重启服务才会生效。

4. 为了让工作站命令（`franka-start`、`franka-connect` 等）正常工作，需在工作站的 `~/.ssh/config` 中添加名为 `NUC` 的 SSH 别名：

```
Host NUC
  HostName 192.168.50.10
  User <nuc用户名>
  IdentityFile ~/.ssh/<你的密钥>
```

### 工作站端

```bash
conda create -n franka python=3.12
conda activate franka
pip install "lerobot[orbbec] @ git+https://github.com/ryanzhangtianran/lerobot.git@feat/add-orbbec-support"

git clone https://github.com/ryanzhangtianran/Franka-Teleop.git
cd Franka-Teleop
pip install -e .                     # 会同时安装本地的 interface 和 teleoperation 两个子包
```

> 本环境必须保持 `numpy==2.2.6` 和 `av>=15`（av 15.x 提供 lerobot 0.6.1 数据集视频默认使用的 `libsvtav1` 编码器）。重装 `pyorbbecsdk2` 可能把 `av` 降级，装完后需要重新升级。严禁安装 `open3d`。

## 命令速查

随时运行 `franka-help` 可在终端查看命令摘要。

### 机器人 / NUC 服务命令

| 命令 | 用法 |
|---|---|
| `franka-start` | 启动 NUC 上的 Franka 驱动服务（通过 SSH 执行 `~/start_franka.sh`，在 `franka` tmux 会话中拉起 `launch_robot`、`launch_gripper`、`launch_server` 和 zerorpc `server.py`）。 |
| `franka-stop` | 停止 NUC 上的全部 Franka 驱动服务（执行 `~/stop_franka.sh`）。 |
| `franka-status` | 查看 NUC 上驱动进程的运行情况，以及 zerorpc 端口 4242 是否在线。 |
| `franka-attach` | 把当前终端接入 NUC 上的 `franka` tmux 会话查看服务日志。按 `Ctrl+b d` 退出。 |
| `franka-gripper-reinit` | 重启 NUC tmux 会话中的 `launch_gripper`。夹爪卡死不响应时使用；NUC 窗口打印 `Homing gripper` 后夹爪即恢复可用。 |
| `franka-connect` | 打开到 Franka Desk 的后台 SSH 隧道并打印地址（`https://localhost:8443/desk/`）。幂等——已有隧道时直接复用。 |
| `franka-disconnect` | 关闭 Franka Desk SSH 隧道。 |

环境变量覆盖：`FRANKA_NUC_SSH_HOST`（服务命令使用的 SSH 主机，默认 `NUC`）；`FRANKA_DESK_SSH_HOST`、`FRANKA_DESK_LOCAL_PORT`、`FRANKA_DESK_TARGET`（隧道设置，默认 `NUC`、`8443`、`172.16.0.2:443`）。

### 录制命令

| 命令 | 用法 |
|---|---|
| `franka-record` | 按 `scripts/config/record_config.yaml` 录制遥操作数据集。启动前先做预检（NUC zerorpc、SpaceMouse、2 台相机、X display、机器人状态/FCI），任一项失败则退出。 |
| `franka-record --check` | 只做预检并打印配置摘要，不开始录制。 |
| `franka-record --single`（或 `-1`） | 录制单条长 episode（用 → 键结束）。 |
| `franka-record --episodes N` | 本次录制 N 条，覆盖配置中的 `task.num_episodes`（不修改配置文件本身）。 |
| `franka-reset` | 让机器人回到 Home 位并打开夹爪。移动前需要确认（机械臂将自主运动约 5 秒——请先清空工作区）。 |

环境变量覆盖：`FRANKA_RECORD_CFG`（指定其他配置文件路径，默认 `scripts/config/record_config.yaml`）；`FRANKA_NUM_EPISODES`（等效于 `--episodes`）。

若未设置 `DISPLAY`，`franka-record` 会自动使用物理图形会话 `:2`，使 rerun 预览窗口和全局方向键监听在本机屏幕上生效。

### 数据集命令

| 命令 | 用法 |
|---|---|
| `franka-replay` | 在真机上回放一条已录 episode。先在 `record_config.yaml` 中设置 `replay.dataset_name` 和 `replay.episode_idx`（使用自动生成的完整名称，含日期/版本后缀）。 |
| `franka-visualize` | 用 rerun 可视化一条 episode。默认读取 `record_config.yaml` 的 `visualize:` 段；可用 `--repo-id <名称>` 和 `--episode-index N` 覆盖。其他选项：`--root <目录>`（本地数据集根目录）、`--save 1 --output-dir <目录>`（保存 `.rrd` 文件而不弹出查看器）、`--mode local\|distant` 配合 `--web-port`/`--ws-port`（远程查看）、`--tolerance-s`（时间戳容差）。 |
| `tools-check-dataset` | 手动删除数据集文件夹后清理 `dataset_info.txt`——移除失效条目，并在 `dataset_info_backup/` 下保存备份。 |
| `tools-check-rs` | 列出已连接的 RealSense 和 Orbbec 相机及其序列号（用于填写配置中的 `cameras.*_cam_id`）。 |
| `map_gripper.sh <名称>` | 创建 udev 规则，把外接夹爪的 USB 串口映射为固定名称 `/dev/<名称>`。只连接这一个 USB 串口设备，然后用 `sudo` 运行。 |
| `franka-help` | 打印命令速查。 |

## 录制流程

### 1. 启动机器人

1. 机器人上电、释放急停、解锁关节。打开 Franka Desk（`franka-connect` → `https://localhost:8443/desk/`）并激活 **FCI**。
2. 启动驱动服务：`franka-start`，再用 `franka-status` 确认（zerorpc 4242 应为 `UP`）。

### 2. 配置任务

编辑 `scripts/config/record_config.yaml`：

- `repo_id`：`<user>/<Verb_Object_prep_Target>`（命名规范见下文）
- `task.description`：任务的自然语言描述（训练时作为 language instruction）
- `task.num_episodes`：本次计划录制条数
- `cameras`：相机类型（`orbbec` / `realsense`）、序列号、分辨率
- `fps`：数据集与相机帧率

如需录制后推送到 Hugging Face Hub，设置 `storage.push_to_hub: True` 并先登录：

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN}
huggingface-cli whoami
```

### 3. 录制

```bash
franka-record --check   # 可选：先整链路自检一遍
franka-record
```

需在可交互终端中运行（SSH 或本机均可）。所有控制按键都从**物理机键盘**全局捕获，录制过程中无需回到启动终端。

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>图 1：录制</b>
</p>

### 遥操作按键

| 输入 | 效果 |
|---|---|
| 推/转 SpaceMouse | 控制末端 6 自由度。**注意存在死区——推杆需过半行程机械臂才响应**（轻推会被静默忽略，属设计行为）。 |
| SpaceMouse 左键 | 打开夹爪 |
| SpaceMouse 右键 | 闭合夹爪 |
| **→**（第 1 次） | 结束并保存当前 episode，程序暂停 |
| **→**（第 2 次） | 进入复位阶段：机械臂回 Home（此时可摆放物体） |
| **→**（第 3 次） | 结束复位，开始录制下一条 |
| **←** | 放弃并重录当前 episode |
| **Esc** | 结束会话，保存全部已录 episode |
| **Ctrl+C** / 异常 | 进入清理流程，询问是否删除不完整数据集（误删可在回收站找回） |

### 4. 录完之后

```bash
franka-replay        # 在机器人上回放（先配置 replay: 段）
franka-visualize     # 用 rerun 检查数据（先配置 visualize: 段）
tools-check-dataset  # 手动删过数据集后刷新 dataset_info.txt
```

<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>图 2：可视化</b>
</p>

## 续录与合并数据集

**续录**：要在已有数据集上追加 episode，在 `record_config.yaml` 中设置 `task.resume: True`，并把已有数据集的完整名称填入 `task.resume_dataset`，然后再次运行 `franka-record`。

**合并**：如果分多次、用不同 `repo_id` 录制，可用 LeRobot 的数据集工具合并：

```bash
lerobot-edit-dataset \
    --repo_id <合并后的repo_id> \
    --operation.type merge \
    --operation.repo_ids "['<repo_id_1>', '<repo_id_2>']"
```

更多数据集操作见 [LeRobot 数据集工具文档](https://huggingface.co/docs/lerobot/using_dataset_tools)。

## 数据集命名与存储

<p align="center">
  <img src="assets/dataset.png" alt="dataset" width="600">
  <br>
  <b>图 3：数据集</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="dataset_info" width="600">
  <br>
  <b>图 4：数据集信息</b>
</p>

1. 数据集存放在 `~/.cache/huggingface/lerobot/<user>/` 下，包含：
   - `dataset_info.txt`：自动记录本地数据集信息（`record_id`、`name`、`task`、`date`、`version`、`user_info`、`type`）。`user_info` 字段来自 `record_config.yaml` 中的 `user_notes`。
   - `dataset_info_backup/`：每次 `tools-check-dataset` 重写 `dataset_info.txt` 时生成的备份。
   - 数据集文件夹本体。
2. 数据集名称格式为 `<描述>_<日期>_<版本>`：
   - `描述` 来自配置中的 `repo_id: <user>/<描述>`；
   - `日期` 自动生成；
   - 相同 `repo_id` 已存在时 `版本` 自动递增。
3. 描述命名规则：`Verb_SourceObject_prep_TargetObject`。例如 "Pick up the green cube and put it into the trash bin" → `pick_greencube_into_trashbin`。

## 常见问题

| 现象 | 原因与处理 |
|---|---|
| 推 SpaceMouse 机械臂不动 | ① 推力不够——死区要求推杆过半行程。② 查看终端日志有无 `[ROBOT] zerorpc error`（执行失败会被降级为 warning，容易漏看）。③ 重跑 `franka-record --check`。 |
| 预检报 NUC 不可达 | NUC 服务没起全——运行 `franka-start`，再 `franka-status` 确认；也可用 `nc -z 192.168.50.10 4242` 验证。 |
| 预检报机器人状态不可读 | Franka Desk 中 FCI 未激活，或急停被按下。 |
| 夹爪不响应 | 运行 `franka-gripper-reinit`，等 NUC 夹爪窗口打印 `Homing gripper`。 |
| 相机枚举时报 `libusb ... timed out` | 该相机接在 USB 2 口，告警无害（15fps 实测不掉帧）；介意可换到 USB 3 口。 |
| 提示无法访问 X display | 物理图形会话（`:2`）不在了——需有人在物理机上保持登录，或改用 VNC（见下）。 |
| 装新包后各种报错 | 大概率 `numpy` 被降级。保持 `numpy==2.2.6`、`av>=15`；严禁安装 `open3d`。 |
| 远程操作 | 笔记本上：`ssh -L 5900:localhost:5900 franka@<工作站>`；工作站上：`x11vnc -display :2 -auth /run/user/1007/gdm/Xauthority -localhost -nopw`；然后 VNC 客户端连 `localhost:5900`。 |
