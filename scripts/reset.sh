#!/usr/bin/env bash
# 复位机械臂到 Home 位(每次使用后收尾用)。
# 用法: ./scripts/reset.sh        复位前会让你确认(机械臂会自主运动)
#       ./scripts/reset.sh -y     跳过确认直接复位
set -euo pipefail

NUC_IP="192.168.50.10"
NUC_PORT=4242
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ASSUME_YES=0
[ "${1:-}" = "-y" ] || [ "${1:-}" = "--yes" ] && ASSUME_YES=1

# 1. conda 环境
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate franka

# 2. 预检: NUC zerorpc 在线(否则连不上,复位无意义)
if ! nc -z -w 3 "$NUC_IP" "$NUC_PORT" 2>/dev/null; then
    echo "❌ NUC ${NUC_IP}:${NUC_PORT} 不可达 — 请先确认 Polymetis 三件套在跑(franka-start)。"
    exit 1
fi
echo "✅ NUC ${NUC_IP}:${NUC_PORT} 在线"

# 3. 安全确认
echo
echo "⚠️  机械臂将【自主运动】回 Home 位(关节移动约 5 秒),夹爪会张开。"
echo "    请确认:工作区无人无物、急停在手边。"
if [ "$ASSUME_YES" -ne 1 ]; then
    read -r -p "确认复位? 输入 y 继续,其它键取消: " ans
    [ "$ans" = "y" ] || { echo "已取消。"; exit 0; }
fi

# 4. 复位
cd "$PROJECT_ROOT"
echo "正在复位..."
exec franka-reset
