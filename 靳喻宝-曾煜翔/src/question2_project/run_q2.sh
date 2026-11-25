#!/usr/bin/env bash
# 题二：正向运动学与坐标变换。只需要 Python 3 + NumPy，不启动 ROS 节点。
#   bash src/question2_project/run_q2.sh            # 求解并写作答
#   bash src/question2_project/run_q2.sh --verify   # 额外用 ROS 2 TF2 独立复核
set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)
cd "$project"

verify=0
while (($#)); do
    case $1 in
        --verify) verify=1; shift ;;
        -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

urdf=${BAJIE_URDF:-$project/src/question2_project/generated/bajie_robot.urdf}
input=$project/inputs/q2_demo_input.jsonl
output=$project/answers/q2_demo_output.jsonl
log=${Q2_LOG:-$project/assets/q2_run.log}

[[ -f $urdf ]] || { echo "缺少 URDF：$urdf" >&2; exit 1; }
py=""
for cand in "$(command -v python3)" /usr/bin/python3; do
    [[ -n $cand && -x $cand ]] || continue
    "$cand" -c 'import numpy' 2>/dev/null && { py=$cand; break; }
done
[[ -n $py ]] || { echo "缺少 numpy：sudo apt install python3-numpy" >&2; exit 1; }

echo "[q2] 求解（urdf=$urdf python=$py）"
set -o pipefail
"$py" src/question2_project/solve_q2.py --urdf "$urdf" --input "$input" --output "$output" 2>&1 | tee "$log"
echo "[q2] 完成：$output 共 $(wc -l < "$output") 行"

if ((verify)); then
    ros_setup=""
    for cand in ${ROS_DISTRO:+"/opt/ros/$ROS_DISTRO/setup.bash"} /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
        [[ -f $cand ]] && { ros_setup=$cand; break; }
    done
    [[ -n $ros_setup ]] || { echo "[q2] 未找到 ROS 2，跳过 TF2 复核" >&2; exit 0; }
    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    set -u
    echo "[q2] TF2 独立复核"
    if [[ ${ROS_DISTRO:-} == humble ]]; then
        export ROS_LOCALHOST_ONLY=1
    else
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    fi
    "$py" src/question2_project/verify_tf.py --urdf "$urdf" --input "$input"
fi
