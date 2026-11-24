#!/usr/bin/env bash
# 题三：Isaac Sim 玩具触碰。完整参数见 src/question3_project/run_q3.sh --help
#   bash run_q3.sh                 # 三例整轮
#   bash run_q3.sh --case rubik_cube
#   bash run_q3.sh --gui
#   bash run_q3.sh --check         # 只探测环境
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$project/src/question3_project/run_q3.sh" "$@"
