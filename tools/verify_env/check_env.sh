#!/usr/bin/env bash
# 作品自检：确认本目录在干净机器上能跑三题，缺什么直接告诉你装什么。
#   bash check_env.sh        # 全部可用退出码 0，存在缺失项退出码 1
set -uo pipefail

project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$project"

ok=0; bad=0
good() { printf '  \033[32m[可用]\033[0m %s\n' "$*"; ok=$((ok+1)); }
miss() { printf '  \033[31m[缺失]\033[0m %s\n' "$*"; bad=$((bad+1)); }
note() { printf '         %s\n' "$*"; }

echo "作品目录：$project"
echo "系统：$(. /etc/os-release && echo "$PRETTY_NAME")  ($(uname -m))"
echo

# ---------- 交付物完整性 ----------
echo "== 交付物完整性 =="
for f in generated/bajie_robot.urdf \
         src/bajie_robot/urdf/bajie_robot.urdf.xacro \
         src/bajie_sim/scene/abr_scene.usd \
         inputs/q1_demo_input.jsonl inputs/q2_demo_input.jsonl inputs/q3_demo_input.jsonl; do
    [[ -f $f ]] && good "$f" || miss "$f"
done
[[ -d src/bajie_sim/models ]] && good "src/bajie_sim/models（官方模型相对目录）" || miss "src/bajie_sim/models"
for d in src/question1_project src/question2_project src/question3_project src/bajie_robot src/bajie_sim; do
    [[ -d $d ]] && good "$d/" || miss "$d/"
done
echo

# ---------- 题一 / 题二：ROS 2 ----------
echo "== 题一、题二：ROS 2 与 Python =="
ros_setup=""
for cand in ${ROS_DISTRO:+"/opt/ros/$ROS_DISTRO/setup.bash"} /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
    [[ -f $cand ]] && { ros_setup=$cand; break; }
done
if [[ -n $ros_setup ]]; then
    good "ROS 2：$ros_setup"
    distro=$(basename "$(dirname "$ros_setup")")
    for pkg in rclpy sensor_msgs xacro robot_state_publisher launch_ros ament_cmake; do
        { [[ -d /opt/ros/$distro/share/$pkg ]] || compgen -G "/opt/ros/$distro/lib/python3*/site-packages/$pkg" >/dev/null; } \
            && good "  ros-$distro-$pkg" || miss "  ros-$distro-$pkg"
    done
    [[ -d /opt/ros/$distro/share/rviz2 ]] && good "  ros-$distro-rviz2（题目一图形演示）" \
        || note "  未装 rviz2：题一可用无界面方式跑通，但没有三维展示"
else
    miss "ROS 2（未发现 /opt/ros/*/setup.bash）"
    note "Ubuntu 22.04 安装：sudo apt install ros-humble-desktop"
fi
command -v colcon >/dev/null && good "colcon：$(command -v colcon)" \
    || { miss "colcon"; note "sudo apt install python3-colcon-common-extensions"; }
np=""
for cand in "$(command -v python3)" /usr/bin/python3; do
    [[ -n $cand && -x $cand ]] || continue
    if "$cand" -c 'import numpy' 2>/dev/null; then np=$cand; break; fi
done
if [[ -n $np ]]; then
    good "numpy：$np（$("$np" -c 'import numpy; print(numpy.__version__)')）"
else
    miss "python3 numpy"; note "sudo apt install python3-numpy"
fi
# scipy 只在题三用到；Isaac Sim 自带 Python 一般已含，系统 Python 需单独装
if [[ -n $np ]] && "$np" -c 'import scipy' 2>/dev/null; then
    good "scipy：$("$np" -c 'import scipy; print(scipy.__version__)')"
else
    note "系统 Python 缺 scipy：题三规划与 run_tests.sh 的题三用例需要它"
    note "  sudo apt install python3-scipy（Isaac Sim 自带 Python 通常已有）"
fi
echo

# ---------- 题三：Isaac Sim ----------
echo "== 题三：Isaac Sim 4.5 =="
looks_like() { [[ -d $1/exts ]] && { [[ -x $1/python.sh ]] || [[ -d $1/isaacsim ]]; }; }
isaac=""
for cand in ${ISAAC_PATH:-} ${ISAAC_SIM_PATH:-} /isaac-sim /opt/isaac-sim /opt/isaac_sim \
            "$HOME/isaac-sim" "$HOME/isaacsim" "$HOME/isaac_sim"; do
    [[ -n $cand ]] && looks_like "$cand" && { isaac=$cand; break; }
done
if [[ -z $isaac ]]; then
    for cand in "$HOME/.local/share/ov/pkg"/isaac_sim-* /opt/isaac_sim-*; do
        [[ -d $cand ]] && looks_like "$cand" && { isaac=$cand; break; }
    done
fi
if [[ -n $isaac ]]; then
    good "Isaac Sim：$isaac"
    [[ -x $isaac/python.sh ]] && good "  $isaac/python.sh" || miss "  $isaac/python.sh"
else
    note "未装原生 Isaac Sim（题一题二不受影响；题三可改用容器，见下）"
fi

# 题三的另一条路径：容器。两者有其一即可。
q3_image=${Q3_IMAGE:-bajie/sim:humble45}
if command -v docker >/dev/null 2>&1; then
    if docker image inspect "$q3_image" >/dev/null 2>&1; then
        good "容器镜像：$q3_image"
        [[ -n $isaac ]] || good "  → 题三可走容器：bash src/question3_project/run_docker.sh"
    else
        miss "容器镜像：$q3_image（未构建）"
        note "docker build -f src/question3_project/Dockerfile -t $q3_image src/question3_project"
    fi
    docker run --rm --gpus all --entrypoint "" "$q3_image" true >/dev/null 2>&1 \
        && { good "  GPU 在容器内可用（--gpus all）"; container_gpu_ok=1; } \
        || note "  容器内 GPU 不可用：确认 NVIDIA 容器运行时，且 docker run --gpus all 能出显卡"
else
    note "未安装 docker：题三只能走原生 Isaac Sim"
fi

# 两条路径都没有，才算题三真的缺依赖
if [[ -z $isaac ]] && ! { command -v docker >/dev/null 2>&1 && docker image inspect "$q3_image" >/dev/null 2>&1; }; then
    miss "题三运行环境：原生 Isaac Sim 与容器镜像都不可用"
fi
if command -v nvidia-smi >/dev/null; then
    if nvidia-smi -L >/dev/null 2>&1; then
        gpu_line=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -1)
        good "GPU：$gpu_line"
        free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1); free=${free%%.*}
        if [[ ${free:-0} =~ ^[0-9]+$ ]] && ((free >= 6000)); then
            good "  空闲显存 ${free} MiB（题三要求 ≥ 6000）"
        else
            miss "  空闲显存 ${free:-0} MiB（题三要求 ≥ 6000）"
        fi
    elif [[ ${container_gpu_ok:-0} == 1 ]]; then
        # 受限命名空间（例如在沙箱/容器里执行本脚本）常见：宿主 shell 看不到
        # /dev/nvidia*，但 docker --gpus all 仍能拿到显卡。以容器内结果为准。
        note "宿主 shell 读不到 nvidia-smi，但容器内 GPU 可用 → 题三走容器没问题"
    else
        miss "nvidia-smi 存在但读不到驱动（确认 NVIDIA 驱动已加载）"
    fi
else
    if [[ ${container_gpu_ok:-0} == 1 ]]; then
        note "宿主无 nvidia-smi，但容器内 GPU 可用 → 题三走容器没问题"
    else
        miss "nvidia-smi（题三需要 NVIDIA 显卡与驱动）"
    fi
fi
mem=$(awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo)
((mem >= 12800)) && good "可用内存 ${mem} MiB（题三要求 ≥ 12800）" \
    || miss "可用内存 ${mem} MiB（题三要求 ≥ 12800，关掉浏览器再试）"
echo

# ---------- 资源指纹 ----------
echo "== 官方资源指纹 =="
check_sha() {
    local want=$1 file=$2 got
    [[ -f $file ]] || { miss "$file 不存在"; return; }
    got=$(sha256sum "$file" | awk '{print $1}')
    [[ $got == "$want" ]] && good "$file 与文档记录一致" || { miss "$file 指纹不符"; note "实际 $got"; }
}
check_sha 6dc26b06aafb82aa7030eabf54e5c181b17b4173eb8fc5e64103fbcbdb0d8f49 src/bajie_robot/urdf/bajie_robot.urdf.xacro
check_sha 42d9882469651205dc590d51f5c02b15eb52d29b6424bd0983a11f2743628735 src/bajie_sim/scene/abr_scene.usd
echo

# ---------- 汇总 ----------
echo "== 汇总 =="
echo "  可用项 $ok 个，缺失项 $bad 个"
if ((bad == 0)); then
    echo "  三题环境齐备，可直接执行 bash run_all.sh"
    exit 0
fi
echo "  缺失项可能只影响其中一题；按上面提示补齐后再执行 bash run_all.sh"
exit 1
