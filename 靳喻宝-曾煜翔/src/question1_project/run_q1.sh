#!/usr/bin/env bash
# 题一：关节分步驱动。默认无界面（评审复跑用这条）；需要录 RViz 演示时加 --gui。
#   bash src/question1_project/run_q1.sh            # 无界面，只跑驱动节点
#   bash src/question1_project/run_q1.sh --gui      # 同时启动 RViz（需要可用的图形桌面）
set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)
cd "$project"

mode=headless
while (($#)); do
    case $1 in
        --gui) mode=gui; shift ;;
        --headless) mode=headless; shift ;;
        -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

ros_setup=""
for cand in ${ROS_DISTRO:+"/opt/ros/$ROS_DISTRO/setup.bash"} /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
    [[ -f $cand ]] && { ros_setup=$cand; break; }
done
[[ -n $ros_setup ]] || { echo "未找到 ROS 2。Ubuntu 22.04 请先安装：sudo apt install ros-humble-desktop" >&2; exit 1; }
# ROS 的 setup.bash 会引用未定义变量，须临时关掉 nounset
set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

input=$project/inputs/q1_demo_input.jsonl
output=$project/answers/q1_demo_output.jsonl
export BAJIE_URDF_DIR="$project/src/bajie_robot"
log=${Q1_LOG:-$project/assets/q1_run.log}
mkdir -p "$(dirname "$log")"

echo "[q1] 编译 bajie_robot 与 bajie_tasks（ROS=${ros_setup}）"
# 显式指定系统解释器：若 PATH 里还有 conda/pyenv 的 python3，CMake 会选错版本导致缺 catkin_pkg
cmake_args=(-DBUILD_TESTING=OFF)
[[ -x /usr/bin/python3 ]] && cmake_args+=(-DPython3_EXECUTABLE=/usr/bin/python3)
colcon build --base-paths src/question1_project src/bajie_robot \
    --packages-select bajie_robot bajie_tasks \
    --cmake-args "${cmake_args[@]}"
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

if [[ $mode == gui && -z ${DISPLAY:-} ]]; then
    echo "[q1] DISPLAY 为空，无法启动 RViz；去掉 --gui 即走无界面模式" >&2
    exit 1
fi

echo "[q1] 运行模式：$mode；作答写入 $output"
if [[ $mode == gui ]]; then
    cmd=(ros2 launch bajie_tasks q1_rviz.launch.py "input_file:=$input" "output_file:=$output")
else
    cmd=(ros2 run bajie_tasks q1_drive --ros-args -p "input_file:=$input" -p "output_file:=$output")
fi

set -o pipefail
set +e
timeout "${Q1_TIMEOUT:-900}" "${cmd[@]}" 2>&1 | tee "$log"
rc=${PIPESTATUS[0]}
set -e
if ((rc != 0)); then
    echo "[q1] 运行失败，退出码 $rc（日志 $log）" >&2
    ((rc == 124)) && echo "[q1] 超过 ${Q1_TIMEOUT:-900}s 被终止" >&2
    exit "$rc"
fi

echo "[q1] 完成：$output 共 $(wc -l < "$output") 行"
