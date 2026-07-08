# Franka Teleoperation Data Collection with Isomorphic Joint Mapping and End-Effector Pose

English | [中文](README_CN.md)

## Introduction

This project is a teleoperation data collection tool for the Franka Research 3 robot, built on top of [LeRobot](https://github.com/huggingface/lerobot.git).

It records datasets in the standard LeRobot format using a [SpaceMouse](https://3dconnexion.com/) for 6-DoF end-effector control, with two Orbbec (or RealSense) cameras for exterior and wrist views. Robot control runs through [Polymetis](https://polymetis-docs.github.io/) on a NUC next to the robot, bridged to this workstation over zerorpc.

<p align="center">
  <img src="assets/spacemouse.png" alt="SpaceMouse" width="600">
  <br>
</p>

The SpaceMouse is a 6-axis input device that controls the end-effector pose of the robot. The left button opens the gripper, the right button closes it.

## System Overview

| Component | Value |
|---|---|
| Workstation conda env | `franka` (Python 3.12, lerobot 0.6.1, this package) |
| NUC (robot host) | `192.168.50.10` — runs Polymetis + `server.py` (zerorpc, port 4242), managed in a tmux session named `franka` |
| Robot | Franka Research 3, control interface at `172.16.0.2` (Franka Desk) |
| Exterior camera | Orbbec Gemini 336L |
| Wrist camera | Orbbec Gemini 336L |
| Teleop device | 3Dconnexion SpaceMouse Compact |
| Dataset storage | `~/.cache/huggingface/lerobot/<user>/<name>_<date>_<version>` |

## Installation

### NUC side: Polymetis

1. Install libfranka (pick the version matching your robot/OS; example: libfranka 0.19.0 on Ubuntu 22.04):

```bash
wget https://github.com/frankarobotics/libfranka/releases/download/0.19.0/libfranka_0.19.0_jammy_amd64.deb
sudo dpkg -i libfranka_0.19.0_jammy_amd64.deb
```

2. Clone and build Polymetis:

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

3. The NUC runs four services (normally started via `franka-start` from the workstation, see below):

```bash
python launch_robot.py robot_client=franka_hardware   # robot driver
python launch_gripper.py gripper=franka_hand          # gripper driver
python launch_server.py                               # Polymetis server
python server.py                                      # zerorpc bridge (port 4242), from interface/interface/server.py
```

> `interface/interface/server.py` runs **on the NUC**. After editing it in this repo, copy it to the NUC and restart the services for changes to take effect.

4. For the workstation commands (`franka-start`, `franka-connect`, ...) to work, add an SSH alias named `NUC` to `~/.ssh/config` on the workstation:

```
Host NUC
  HostName 192.168.50.10
  User <nuc-user>
  IdentityFile ~/.ssh/<your-key>
```

### Workstation side

```bash
conda create -n franka python=3.12
conda activate franka
pip install "lerobot[orbbec] @ git+https://github.com/ryanzhangtianran/lerobot.git@feat/add-orbbec-support"

git clone https://github.com/ryanzhangtianran/Franka-Teleop.git
cd Franka-Teleop
pip install -e .                     # also installs the local `interface` and `teleoperation` packages
```

> Keep `numpy==2.2.6` and `av>=15` in this environment (`av` 15.x provides the `libsvtav1` encoder that lerobot 0.6.1 uses for dataset videos). Reinstalling `pyorbbecsdk2` may downgrade `av` — upgrade it back afterwards. Do not install `open3d`.

## Command Reference

Run `franka-help` at any time to see this summary in the terminal.

### Robot / NUC service commands

| Command | Usage |
|---|---|
| `franka-start` | Start the Franka driver services on the NUC (runs `~/start_franka.sh` over SSH; brings up `launch_robot`, `launch_gripper`, `launch_server`, and the zerorpc `server.py` in the `franka` tmux session). |
| `franka-stop` | Stop all Franka driver services on the NUC (runs `~/stop_franka.sh`). |
| `franka-status` | Show the driver processes running on the NUC and whether the zerorpc port 4242 is up. |
| `franka-attach` | Attach your terminal to the `franka` tmux session on the NUC to watch service logs. Detach with `Ctrl+b d`. |
| `franka-gripper-reinit` | Restart `launch_gripper` in the NUC tmux session. Use this when the gripper stops responding; the gripper is ready again once the NUC window prints `Homing gripper`. |
| `franka-connect` | Open a background SSH tunnel to Franka Desk and print the URL (`https://localhost:8443/desk/`). Idempotent — reuses an existing tunnel. |
| `franka-disconnect` | Close the Franka Desk SSH tunnel. |

Environment variable overrides: `FRANKA_NUC_SSH_HOST` (SSH host for the service commands, default `NUC`); `FRANKA_DESK_SSH_HOST`, `FRANKA_DESK_LOCAL_PORT`, `FRANKA_DESK_TARGET` (tunnel settings, defaults `NUC`, `8443`, `172.16.0.2:443`).

### Recording commands

| Command | Usage |
|---|---|
| `franka-record` | Record a teleoperation dataset according to `scripts/config/record_config.yaml`. Runs preflight checks first (NUC zerorpc, SpaceMouse, 2 cameras, X display, robot state/FCI) and aborts if any fail. |
| `franka-record --check` | Run the preflight checks and print the config summary only; do not start recording. |
| `franka-record --single` (or `-1`) | Record a single long episode (end it with the → key). |
| `franka-record --episodes N` | Record `N` episodes this run, overriding `task.num_episodes` in the config (the file itself is not modified). |
| `franka-reset` | Move the robot back to the home position and open the gripper. Asks for confirmation before moving (the arm moves autonomously for ~5 s — clear the workspace first). |

Environment variable overrides: `FRANKA_RECORD_CFG` (path to an alternative config file, default `scripts/config/record_config.yaml`); `FRANKA_NUM_EPISODES` (same effect as `--episodes`).

If no `DISPLAY` is set, `franka-record` targets the physical graphical session `:2` so the rerun preview window and the global arrow-key listener work on the machine's own screen.

### Dataset commands

| Command | Usage |
|---|---|
| `franka-replay` | Replay one recorded episode on the real robot. Set `replay.dataset_name` and `replay.episode_idx` in `record_config.yaml` first (use the full auto-generated name, including the date/version suffix). |
| `franka-visualize` | Visualize a recorded episode with rerun. Defaults come from the `visualize:` section of `record_config.yaml`; override with `--repo-id <name>` and `--episode-index N`. Other options: `--root <dir>` (local dataset root), `--save 1 --output-dir <dir>` (write a `.rrd` file instead of spawning a viewer), `--mode local\|distant` with `--web-port`/`--ws-port` (remote viewing), `--tolerance-s` (timestamp tolerance). |
| `tools-check-dataset` | Clean up `dataset_info.txt` after you manually delete dataset folders — removes stale entries and saves a backup under `dataset_info_backup/`. |
| `tools-check-rs` | List connected RealSense and Orbbec cameras with their serial numbers (used for `cameras.*_cam_id` in the config). |
| `map_gripper.sh <name>` | Create a udev rule that maps an external gripper's USB serial port to a stable name like `/dev/<name>`. Connect only that one USB serial device, then run with `sudo`. |
| `franka-help` | Print the command reference. |

## Recording Workflow

### 1. Start the robot

1. Power on the robot, release the emergency stop, and unlock the joints. Open Franka Desk (`franka-connect` → `https://localhost:8443/desk/`) and activate **FCI**.
2. Start the driver services: `franka-start`, then confirm with `franka-status` (zerorpc 4242 should be `UP`).

### 2. Configure the task

Edit `scripts/config/record_config.yaml`:

- `repo_id`: `<user>/<Verb_Object_prep_Target>` (see naming rules below)
- `task.description`: natural-language task description (used as the language instruction during training)
- `task.num_episodes`: number of episodes to record
- `cameras`: camera type (`orbbec` / `realsense`), serial numbers, resolution
- `fps`: dataset and camera frame rate

To push the dataset to the Hugging Face Hub after recording, set `storage.push_to_hub: True` and log in first:

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN}
huggingface-cli whoami
```

### 3. Record

```bash
franka-record --check   # optional: verify the whole chain first
franka-record
```

Run it in an interactive terminal (SSH or local). All control keys are captured globally from the **physical keyboard**, so you do not need to return to the launching terminal while recording.

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Recording</b>
</p>

### Teleoperation controls

| Input | Effect |
|---|---|
| Push / twist the SpaceMouse | 6-DoF end-effector control. **Note: there is a dead zone — push past roughly half travel before the arm responds** (light pushes are silently ignored by design). |
| SpaceMouse left button | Open the gripper |
| SpaceMouse right button | Close the gripper |
| **→** (1st press) | End and save the current episode; the program pauses |
| **→** (2nd press) | Enter the reset phase: the arm returns to home (rearrange objects now) |
| **→** (3rd press) | Finish the reset and start recording the next episode |
| **←** | Discard and re-record the current episode |
| **Esc** | End the session and save all recorded episodes |
| **Ctrl+C** / exception | Enter the cleanup flow, which asks whether to delete the incomplete dataset (accidentally deleted datasets can be recovered from the trash) |

### 4. After recording

```bash
franka-replay        # replay an episode on the robot (configure the replay: section first)
franka-visualize     # inspect an episode in rerun (configure the visualize: section first)
tools-check-dataset  # refresh dataset_info.txt after manually deleting datasets
```

<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 2: Visualization</b>
</p>

## Resuming and Merging Datasets

**Resume**: to append episodes to an existing dataset, set `task.resume: True` in `record_config.yaml` and put the existing dataset's full name in `task.resume_dataset`, then run `franka-record` again.

**Merge**: if you recorded in multiple stages under different `repo_id`s, merge them with LeRobot's dataset tools:

```bash
lerobot-edit-dataset \
    --repo_id <merged_repo_id> \
    --operation.type merge \
    --operation.repo_ids "['<repo_id_1>', '<repo_id_2>']"
```

See the [LeRobot dataset tools docs](https://huggingface.co/docs/lerobot/using_dataset_tools) for more operations.

## Dataset Naming and Storage

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

1. Datasets are stored under `~/.cache/huggingface/lerobot/<user>/`, which contains:
   - `dataset_info.txt`: automatically records local dataset information (`record_id`, `name`, `task`, `date`, `version`, `user_info`, `type`). The `user_info` field is filled from `user_notes` in `record_config.yaml`.
   - `dataset_info_backup/`: backups created whenever `tools-check-dataset` rewrites `dataset_info.txt`.
   - The dataset folders themselves.
2. Dataset names follow `<description>_<date>_<version>`:
   - `description` comes from `repo_id: <user>/<description>` in the config;
   - `date` is generated automatically;
   - `version` auto-increments when a dataset with the same `repo_id` already exists.
3. Description naming rule: `Verb_SourceObject_prep_TargetObject`. Example: "Pick up the green cube and put it into the trash bin" → `pick_greencube_into_trashbin`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Arm does not move when pushing the SpaceMouse | ① Not pushing hard enough — the dead zone requires more than half travel. ② Check the terminal log for `[ROBOT] zerorpc error` (execution failures are downgraded to warnings and easy to miss). ③ Re-run `franka-record --check`. |
| Preflight reports NUC unreachable | The NUC services are not all running — run `franka-start`, then `franka-status`; verify with `nc -z 192.168.50.10 4242`. |
| Preflight reports robot state unreadable | FCI is not activated in Franka Desk, or the e-stop is engaged. |
| Gripper stops responding | Run `franka-gripper-reinit` and wait until the NUC gripper window prints `Homing gripper`. |
| `libusb ... timed out` during camera enumeration | That camera is on a USB 2 port; the warning is harmless (no frame drops at 15 fps). Move it to a USB 3 port if desired. |
| "Cannot access X display" | The physical graphical session (`:2`) is gone — someone must be logged in on the physical machine, or use VNC (below). |
| Random errors after installing a new package | `numpy` was probably downgraded. Keep `numpy==2.2.6` and `av>=15`; never install `open3d`. |
| Remote operation | On your laptop: `ssh -L 5900:localhost:5900 franka@<workstation>`; on the workstation: `x11vnc -display :2 -auth /run/user/1007/gdm/Xauthority -localhost -nopw`; then point a VNC client at `localhost:5900`. |
