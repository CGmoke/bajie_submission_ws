#!/usr/bin/env bash
# 三题单元测试一次跑完。题一需要先编译（脚本会自动补编译）。
#   bash src/question1_project/run_tests.sh
set -uo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)
cd "$project"

py=""
for cand in "$(command -v python3)" /usr/bin/python3; do
    [[ -n $cand && -x $cand ]] || continue
    "$cand" -c 'import numpy' 2>/dev/null && { py=$cand; break; }
done
[[ -n $py ]] || { echo "缺少 numpy：sudo apt install python3-numpy" >&2; exit 1; }

ros_setup=""
for cand in ${ROS_DISTRO:+"/opt/ros/$ROS_DISTRO/setup.bash"} /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
    [[ -f $cand ]] && { ros_setup=$cand; break; }
done

if [[ -n $ros_setup ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    set -u
    if [[ ! -f install/setup.bash ]]; then
        echo "[test] 题一需要已安装的包，先编译"
        cmake_args=(-DBUILD_TESTING=OFF)
        [[ -x /usr/bin/python3 ]] && cmake_args+=(-DPython3_EXECUTABLE=/usr/bin/python3)
        colcon build --base-paths src/question1_project src/bajie_robot \
            --packages-select bajie_robot bajie_tasks --cmake-args "${cmake_args[@]}" >/dev/null
    fi
    set +u
    # shellcheck disable=SC1091
    source install/setup.bash
    set -u
else
    echo "[test] 未找到 ROS 2：题一、题二的 TF 复核相关测试会跳过" >&2
fi

declare -A result=()
run_suite() {
    local name=$1 dir=$2 out rc
    echo "---------- $name ----------"
    out=$("$py" -m unittest discover -s "$dir" -v 2>&1); rc=$?
    printf '%s\n' "$out" | tail -4
    if ((rc == 0)); then result[$name]="通过"; else result[$name]="失败"; fi
}

run_suite "题一 净空与停播" src/question1_project/test
run_suite "题二 运动学" src/question2_project/tests
run_suite "题三 触碰逻辑" src/question3_project/tests

echo
echo "== 汇总 =="
for k in "题一 净空与停播" "题二 运动学" "题三 触碰逻辑"; do
    printf '  %-20s %s\n' "$k" "${result[$k]:-未运行}"
done
