#!/usr/bin/env bash
# 一键启动 Franka SpaceMouse 数据采集(含预检)
# 用法: ./scripts/start_record.sh              正式采集(按 record_cfg.yaml 的 num_episodes)
#       ./scripts/start_record.sh --check       只做预检不启动
#       ./scripts/start_record.sh --single      只录 1 条(长 episode,按 → 主动结束)
#       ./scripts/start_record.sh --episodes N  本次只录 N 条(覆盖配置,不改文件)
# (参数可组合,如 --single --check)
set -euo pipefail

NUC_IP="192.168.50.10"
NUC_PORT=4242
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ── 参数解析 ───────────────────────────────
CHECK_ONLY=0
NUM_OVERRIDE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --check)        CHECK_ONLY=1 ;;
        --single|-1)    NUM_OVERRIDE=1 ;;
        --episodes)     shift; NUM_OVERRIDE="${1:?--episodes 后面需要一个数字}" ;;
        --episodes=*)   NUM_OVERRIDE="${1#*=}" ;;
        *) echo "未知参数: $1"; echo "可用: --check | --single | --episodes N"; exit 1 ;;
    esac
    shift
done
if [ -n "$NUM_OVERRIDE" ]; then
    case "$NUM_OVERRIDE" in
        ''|*[!0-9]*) echo "--episodes 必须是正整数,收到: $NUM_OVERRIDE"; exit 1 ;;
    esac
    export FRANKA_NUM_EPISODES="$NUM_OVERRIDE"
fi

# 1. conda 环境
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate franka

# 2. GUI 会话(rerun 窗口 + 方向键监听落在 :2 物理屏幕)
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:2
    export XAUTHORITY=/run/user/1007/gdm/Xauthority
fi

# 3. 预检
fail=0
echo "── 预检 ──────────────────────────────"
if nc -z -w 3 "$NUC_IP" "$NUC_PORT" 2>/dev/null; then
    echo "  ✅ NUC ${NUC_IP}:${NUC_PORT} (Polymetis/zerorpc) 在线"
else
    echo "  ❌ NUC 服务不可达 — 请在 NUC 上启动 Polymetis 三件套 + server.py"; fail=1
fi
if lsusb | grep -qi "3dconnexion"; then
    echo "  ✅ SpaceMouse 已连接"
else
    echo "  ❌ 未检测到 SpaceMouse"; fail=1
fi
cam_n=$(lsusb | grep -ci "2bc5:0807" || true)
if [ "$cam_n" -ge 2 ]; then
    echo "  ✅ Orbbec 相机 ×${cam_n}"
else
    echo "  ❌ Orbbec 相机只检测到 ${cam_n} 个(需要 2 个)"; fail=1
fi
if xset q >/dev/null 2>&1; then
    echo "  ✅ X display ${DISPLAY} 可用"
else
    echo "  ❌ 无法访问 ${DISPLAY}(图形会话不在?)"; fail=1
fi
robot_state=$(timeout 10 python - <<'PY' 2>/dev/null | tail -1
from interface.client import FrankaInterfaceClient
import numpy as np
c = FrankaInterfaceClient(ip="192.168.50.10", port=4242)
jp = c.robot_get_joint_positions(); c.close()
print("OK" if np.isfinite(jp).all() and len(jp) == 7 else "BAD")
PY
) || robot_state="BAD"
if [ "$robot_state" = "OK" ]; then
    echo "  ✅ 机械臂状态可读(FCI 正常)"
else
    echo "  ❌ 读不到机械臂关节角 — 检查 Franka Desk 是否激活 FCI、急停是否释放"; fail=1
fi
echo "──────────────────────────────────────"
[ "$fail" -ne 0 ] && { echo "预检未通过,已退出。"; exit 1; }

cfg="${FRANKA_RECORD_CFG:-$PROJECT_ROOT/scripts/config/record_cfg.yaml}"
echo "当前任务配置:"
python - "$cfg" <<'PY'
import sys, os, yaml
c = yaml.safe_load(open(sys.argv[1]))["record"]
n = c['task']['num_episodes']
override = os.environ.get("FRANKA_NUM_EPISODES")
n_show = f"{override} (本次命令行覆盖, 配置文件是 {n})" if override else str(n)
print(f"  repo_id     : {c['repo_id']}")
print(f"  task        : {c['task']['description']}")
print(f"  episodes    : {n_show}  | fps: {c['fps']}  | debug: {c['debug']}")
print(f"  robot.ip    : {c['robot']['ip']}  | control: {c['teleop']['control_mode']}")
PY

if [ "$CHECK_ONLY" -eq 1 ]; then
    echo "(--check 模式,预检完成,不启动录制)"
    exit 0
fi

echo
echo "操作提示(全部在物理键盘): →=保存→复位→录下一条(连按推进)  ←=重录本条  Esc=结束保存全部"
cd "$PROJECT_ROOT"
exec franka-record
