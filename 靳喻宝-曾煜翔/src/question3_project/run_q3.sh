#!/usr/bin/env bash
# 题三启动器：在原生 Isaac Sim 4.5 上复现三例玩具触碰（Ubuntu 22.04 + ROS 2 Humble，无需容器）。
#
#   bash src/question3_project/run_q3.sh                 # 三例整轮，作答写入新的临时目录
#   bash src/question3_project/run_q3.sh --case rubik_cube   # 单例排查，不写作答
#   bash src/question3_project/run_q3.sh --gui           # 打开 Isaac Sim 窗口
#   bash src/question3_project/run_q3.sh --check         # 只做环境探测，不启动仿真
#
# 路径默认按作品目录推算，无需任何 export；可用 BAJIE_SIM_DIR / BAJIE_URDF 覆盖。
set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)

log() { printf '[q3] %s\n' "$*" >&2; }
die() { printf '[q3] 未启动：%s\n' "$*" >&2; exit 1; }

usage() {
    cat <<'EOF'
用法: run_q3.sh [--case rubik_cube|dinosaur|soft_toy] [--gui] [--check]

  --case NAME   只验证一例，不写正式作答
  --gui         打开 Isaac Sim 原生窗口（需要 DISPLAY）
  --check       只探测 Isaac Sim / ROS 2 / 资源与显存，不启动仿真

环境变量:
  ISAAC_PATH      Isaac Sim 4.5 安装根目录（含 python.sh 与 exts/），默认自动探测
  BAJIE_SIM_DIR   官方仿真资源根目录，默认 <作品目录>/src/bajie_sim
  BAJIE_URDF      展开后的官方 URDF，默认 <作品目录>/generated/bajie_robot.urdf
  Q3_RESULT_DIR   结果目录（必须已存在），默认新建临时目录
  Q3_STRATEGY     adaptive（默认）| baseline | adaptive-fallback
  Q3_SKIP_CHECKS  1 时跳过内存与显存预检（不推荐）
EOF
}

gui=0
case_name=""
check_only=0
while (($#)); do
    case $1 in
        --gui) gui=1; shift ;;
        --check) check_only=1; shift ;;
        --case)
            (($# >= 2)) || die '--case 缺少参数'
            case_name=$2
            [[ $case_name =~ ^(rubik_cube|dinosaur|soft_toy)$ ]] || die "--case 只支持 rubik_cube、dinosaur、soft_toy"
            shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "未知参数: $1" ;;
    esac
done

strategy=${Q3_STRATEGY:-adaptive-fallback}
[[ $strategy =~ ^(baseline|adaptive|adaptive-fallback)$ ]] || die "Q3_STRATEGY 无效: $strategy"

# --- 作品内资源：默认值即提交目录，不依赖任何外部工作区 ---
scene=${BAJIE_SIM_DIR:-$project/src/bajie_sim}
urdf=${BAJIE_URDF:-$project/generated/bajie_robot.urdf}
input=$project/inputs/q3_demo_input.jsonl
scene=$(realpath -e "$scene" 2>/dev/null) || die "仿真资源目录不存在：${BAJIE_SIM_DIR:-$project/src/bajie_sim}"
urdf=$(realpath -e "$urdf" 2>/dev/null) || die "URDF 不存在：${BAJIE_URDF:-$project/generated/bajie_robot.urdf}"
[[ -f $scene/scene/abr_scene.usd ]] || die "缺少官方场景 $scene/scene/abr_scene.usd"
[[ -d $scene/models ]] || die "缺少官方模型目录 $scene/models（只保留入口 USD 不可用）"
[[ -f $input ]] || die "缺少输入 $input"

if [[ -n ${Q3_RESULT_DIR:-} ]]; then
    [[ -d $Q3_RESULT_DIR ]] || die "Q3_RESULT_DIR 不存在：$Q3_RESULT_DIR"
    out=$(realpath "$Q3_RESULT_DIR")
else
    out=$(mktemp -d "${TMPDIR:-/tmp}/bajie-q3.XXXXXX")
fi
[[ -w $out ]] || die "结果目录不可写：$out"

# --- 原生 Isaac Sim 探测 ---
looks_like_isaac() { [[ -d $1/exts ]] && { [[ -x $1/python.sh ]] || [[ -d $1/isaacsim ]]; }; }

find_isaac() {
    local cand
    for cand in ${ISAAC_PATH:-} ${ISAAC_SIM_PATH:-} \
                /isaac-sim /opt/isaac-sim /opt/isaac_sim \
                "$HOME/isaac-sim" "$HOME/isaacsim" "$HOME/isaac_sim"; do
        [[ -n $cand ]] || continue
        looks_like_isaac "$cand" && { printf '%s\n' "$cand"; return 0; }
    done
    for cand in "$HOME/.local/share/ov/pkg"/isaac_sim-* /opt/isaac_sim-* /opt/IsaacSim*; do
        [[ -d $cand ]] || continue
        looks_like_isaac "$cand" && { printf '%s\n' "$cand"; return 0; }
    done
    return 1
}

isaac=$(find_isaac) || die "未找到 Isaac Sim 4.5。请安装官方 Isaac Sim 4.5，或 export ISAAC_PATH=<安装根目录>（该目录应含 python.sh 与 exts/）"
export ISAAC_PATH=$isaac

runner=$isaac/python.sh
if [[ ! -x $runner ]]; then
    python3 -c 'import isaacsim' 2>/dev/null || die "$isaac 下没有可执行的 python.sh。若使用 pip 方式安装，请改用官方 tarball/工作站版（含 python.sh 与 exts/）"
    runner=$(command -v python3)
    log "警告：使用 $runner 而非 python.sh，仅支持含 exts/ 的完整安装布局"
fi

# --- ROS 2 运行时（Isaac Sim 4.5 的 Python 是 3.10，配 ROS 2 Humble） ---
ros_setup=""
for cand in ${ROS_DISTRO:+"/opt/ros/$ROS_DISTRO/setup.bash"} /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
    [[ -f $cand ]] && { ros_setup=$cand; break; }
done
[[ -n $ros_setup ]] || die "未找到 ROS 2（/opt/ros/humble/setup.bash 或 /opt/ros/jazzy/setup.bash），题三需要 ROS 2 Humble"
[[ $ros_setup == */humble/* ]] || log "警告：$ros_setup 不是 Humble；Isaac Sim 4.5 的 Python 为 3.10，与 Jazzy 的 3.12 模块不兼容，官方环境请用 Humble"
# Isaac Sim 的 Python 靠 PYTHONPATH / LD_LIBRARY_PATH 找到 rclpy 与 ROS 库，
# 必须先 source 再启动；ROS 脚本引用未定义变量，故临时关闭 nounset。
set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

# --- 资源预检：与容器版同一条线，避免把机器拖死 ---
avail_mib() { awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 0; }
gpu_free_mib() { timeout 3s nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | awk 'NR==1 {print int($1)}' ; }
need_mem=12800; need_gpu=6000
[[ $gui == 1 ]] || true

if [[ $check_only == 1 || ${Q3_SKIP_CHECKS:-0} != 1 ]]; then
    mem=$(avail_mib); gpu=$(gpu_free_mib) || gpu=""
    [[ $gpu =~ ^[0-9]+$ ]] || gpu=0
    if [[ $check_only == 1 ]]; then
        log "可用内存 ${mem} MiB（要求 ≥ ${need_mem}）"
        log "空闲显存 ${gpu} MiB（要求 ≥ ${need_gpu}）"
    fi
    if [[ ${Q3_SKIP_CHECKS:-0} != 1 ]]; then
        ((mem >= need_mem)) || die "系统可用内存不足 ${need_mem} MiB，请先关闭占用内存的程序（确认无碍可设 Q3_SKIP_CHECKS=1 跳过）"
        ((gpu >= need_gpu)) || die "显存余量不足 ${need_gpu} MiB，或 nvidia-smi 不可用（确认无碍可设 Q3_SKIP_CHECKS=1 跳过）"
    fi
fi

# --- 组装命令 ---
args=(--scene "$scene/scene/abr_scene.usd" --urdf "$urdf" --input "$input" --strategy "$strategy")
if [[ -n $case_name ]]; then
    args+=(--case "$case_name")
else
    args+=(--output "$out/q3_demo_output.jsonl")
fi
if [[ $gui == 1 ]]; then
    [[ -n ${DISPLAY:-} ]] || die 'GUI 模式需要图形桌面，但 DISPLAY 为空'
    args+=(--gui)
fi

cmd=("$runner" -u "$here/run_touch.py" "${args[@]}")
# 结构化日志（events.jsonl / contact-<case>.json / run.json）与 Kit 原始输出分开落盘。
export Q3_LOG_DIR="${Q3_LOG_DIR:-$project/assets/q3_logs}"

if [[ $check_only == 1 ]]; then
    log "Isaac Sim : $isaac"
    log "ROS 2     : $ros_setup"
    log "场景      : $scene/scene/abr_scene.usd"
    log "URDF      : $urdf"
    log "输入      : $input"
    log "结果目录  : $out"
    log "将执行    : ${cmd[*]}"
    exit 0
fi

log "Isaac Sim=$isaac ROS=$ros_setup strategy=$strategy result_dir=$out"

# ---------------------------------------------------------------------------
# 后台看门狗：可用内存或显存触线、或超过时限，就终止整个进程组。
# 原生进程不受容器内存上限约束，这一层是唯一的兜底。
# ---------------------------------------------------------------------------
# 时限按时长分档：单例约 2~3 分钟，三例整轮约 8~9 分钟（每例先触碰再逆序归位）。
# 固定 360s 会把整轮从第二例起就掐断。
if [[ -n ${Q3_TIMEOUT:-} ]]; then
    limit=$Q3_TIMEOUT
elif [[ $gui == 1 ]]; then
    limit=1800
elif [[ -n $case_name ]]; then
    limit=600
else
    limit=1800
fi
watcher=""
pid=""
cleanup() {
    [[ -z $watcher ]] || kill "$watcher" 2>/dev/null || true
    if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        sleep 2
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

(
    start=$SECONDS; low=0
    while sleep 2; do
        kill -0 "$pid" 2>/dev/null || break
        a=$(avail_mib); g=$(gpu_free_mib) || g=0
        [[ $g =~ ^[0-9]+$ ]] || g=0
        if ((a < 4500 || g < 500)); then low=$((low+1)); else low=0; fi
        printf '[watchdog] host_available=%s MiB gpu_free=%s MiB\n' "$a" "$g" >&2
        if ((low >= 2 || SECONDS - start >= limit)); then
            printf '[watchdog] 内存、显存或运行时间达到停止线，正在终止仿真。\n' >&2
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
            break
        fi
    done
) & watcher=$!

# 独立进程组便于整组回收；Isaac Sim 会派生多个子进程。
setsid "${cmd[@]}" &
pid=$!
rc=0
wait "$pid" || rc=$?

kill "$watcher" 2>/dev/null || true
wait "$watcher" 2>/dev/null || true
pid=""   # 已退出，避免 cleanup 重复发信号

if [[ $rc -eq 0 && -n $case_name && -f $out/q3_demo_output.jsonl ]]; then
    log "单例验证通过：$out/q3_demo_output.jsonl"
elif [[ $rc -eq 0 && -f $out/q3_demo_output.jsonl ]]; then
    log "三例全部通过，作答已写入：$out/q3_demo_output.jsonl"
fi
exit "$rc"
