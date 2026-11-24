#!/usr/bin/env bash
# 题三 Docker 启动器：在本机容器里跑三例玩具触碰，不需要装原生 Isaac Sim。
#
#   bash src/question3_project/run_docker.sh --check           # 只探测镜像/GPU/内存，不启动
#   bash src/question3_project/run_docker.sh --case rubik_cube # 单例排查，不写作答
#   Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_docker.sh   # 整轮，写作答
#
# 与 run_q3.sh 的分工：run_q3.sh 走原生 Isaac Sim（官方评审环境），本脚本走容器；
# 两者执行同一份 run_touch.py，参数与策略完全一致。
#
# 关键设计（都是被本机踩过的坑）：
#   1. 显存预检先在宿主机跑 nvidia-smi；宿主 shell 读不到驱动时（受限命名空间里
#      没有 /dev/nvidia*），退化为一次性容器探测。宿主机读不到 ≠ 容器用不了。
#   2. 官方模型与场景只读挂载，只有结果目录可写。
#   3. Kit 着色器缓存持久化，否则每次冷启动都要重编 RTX 着色器（约 4 分钟）。
set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)

log() { printf '[q3-docker] %s\n' "$*" >&2; }
die() { printf '[q3-docker] 未启动：%s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,10p' "$0"; }

image=${Q3_IMAGE:-bajie/sim:humble45}
strategy=${Q3_STRATEGY:-adaptive-fallback}
check_only=0
case_name=""
while (($#)); do
    case $1 in
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
[[ $strategy =~ ^(baseline|adaptive|adaptive-fallback)$ ]] || die "Q3_STRATEGY 无效: $strategy"

command -v docker >/dev/null || die '缺 docker'

# ---- 作品内资源：默认按提交目录推算，不依赖外部工作区 ----
scene=${BAJIE_SIM_DIR:-$project/src/bajie_sim}
urdf=${BAJIE_URDF:-$project/generated/bajie_robot.urdf}
input=$project/inputs/q3_demo_input.jsonl
scene=$(realpath -e "$scene" 2>/dev/null) || die "仿真资源目录不存在：${BAJIE_SIM_DIR:-$project/src/bajie_sim}"
urdf=$(realpath -e "$urdf" 2>/dev/null) || die "URDF 不存在：${BAJIE_URDF:-$project/generated/bajie_robot.urdf}"
[[ -f $scene/scene/abr_scene.usd ]] || die "缺少官方场景 $scene/scene/abr_scene.usd"
[[ -d $scene/models ]] || die "缺少官方模型目录 $scene/models（只保留入口 USD 不可用）"
[[ -f $input ]] || die "缺少输入 $input"

# ---- 结果目录：整轮默认写临时目录，避免误覆盖正式作答 ----
if [[ -n ${Q3_RESULT_DIR:-} ]]; then
    [[ -d $Q3_RESULT_DIR ]] || die "Q3_RESULT_DIR 不存在：$Q3_RESULT_DIR"
    out=$(realpath "$Q3_RESULT_DIR")
elif [[ $strategy != baseline && -z $case_name ]]; then
    out=$(mktemp -d "${TMPDIR:-/tmp}/bajie-q3-touch.XXXXXX")
else
    out=$project/answers
fi
[[ -w $out ]] || die "结果目录不可写：$out"

# ---- 内存与显存预检 ----
avail_mib() { awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 0; }

# 宿主 nvidia-smi 能不能用，只决定「怎么探测显存」，不决定「能不能跑」
# 注意：nvidia-smi 失败时会把错误文本打到 stdout，必须只接受纯数字行，
# 否则 int("NVIDIA-SMI has failed...") 会变成 0，误判成「显存用满」。
host_gpu_free() {
    local v
    v=$(timeout 3s nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null \
        | awk 'NR==1 && $1 ~ /^[0-9]+$/ {print int($1)}')
    [[ -n $v ]] && printf '%s\n' "$v"
}
container_gpu_free() {
    local v
    v=$(timeout 60s docker run --rm --gpus all --entrypoint "" "$image" \
            nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null \
        | awk 'NR==1 && $1 ~ /^[0-9]+$/ {print int($1)}')
    [[ -n $v ]] && printf '%s\n' "$v"
}

docker image inspect "$image" >/dev/null 2>&1 \
    || die "镜像不存在：$image（用 docker/Dockerfile.sim 构建，或用 Q3_IMAGE 指定）"

gpu=$(host_gpu_free || true)
gpu_src="宿主机 nvidia-smi"
if [[ -z ${gpu:-} ]]; then
    gpu=$(container_gpu_free || true)
    gpu_src="容器内 nvidia-smi"
fi
mem=$(avail_mib)
need_mem=12800; need_gpu=6000

# 上一轮仿真的显存是异步释放的，单次采样容易读到刚被占用的瞬时值。
# 连续采样取最大，避免把「正在收尾」误判成「显存不够」。
if [[ -n ${gpu:-} ]] && ((gpu < need_gpu)) && [[ ${Q3_SKIP_CHECKS:-0} != 1 ]]; then
    for _ in 1 2 3 4; do
        sleep 4
        again=$(host_gpu_free || true)
        [[ -n ${again:-} ]] || again=$(container_gpu_free || true)
        [[ -n ${again:-} ]] || continue
        ((again > gpu)) && gpu=$again
        ((gpu >= need_gpu)) && break
    done
fi

if [[ $check_only == 1 ]]; then
    log "镜像      : $image"
    log "内存      : ${mem} MiB 可用（要求 ≥ ${need_mem}）"
    log "显存      : ${gpu:-未知} MiB 空闲（要求 ≥ ${need_gpu}，来源：${gpu_src}）"
    log "场景      : $scene/scene/abr_scene.usd"
    log "URDF      : $urdf"
    log "输入      : $input"
    log "结果目录  : $out"
    [[ -n ${gpu:-} ]] || log "警告：两种方式都读不到显存，实跑前请确认 GPU 可用"
    exit 0
fi

if [[ ${Q3_SKIP_CHECKS:-0} != 1 ]]; then
    ((mem >= need_mem)) || die "可用内存 ${mem} MiB < ${need_mem} MiB，先关掉占内存的程序（确认无碍可设 Q3_SKIP_CHECKS=1）"
    [[ -n ${gpu:-} ]] || die "读不到显存：宿主机与容器内的 nvidia-smi 都失败，检查 NVIDIA 容器运行时（确认无碍可设 Q3_SKIP_CHECKS=1）"
    ((gpu >= need_gpu)) || die "显存余量 ${gpu} MiB < ${need_gpu} MiB（确认无碍可设 Q3_SKIP_CHECKS=1）"
fi

# ---- 组装参数 ----
args=(--scene /scene/scene/abr_scene.usd --urdf /model.urdf --input /project/inputs/q3_demo_input.jsonl
      --strategy "$strategy")
if [[ -n $case_name ]]; then
    args+=(--case "$case_name")
else
    args+=(--output /result/q3_demo_output.jsonl)
fi

extra=(); net=none; mem_limit=${Q3_MEM:-12g}
cache=${Q3_CACHE_DIR:-$HOME/.cache/bajie-q3-kit}
mkdir -p "$cache/kit-cache" "$cache/kit-data"
extra+=(-v "$cache/kit-cache:/isaac-sim/kit/cache"
        -v "$cache/kit-data:/isaac-sim/kit/data")

# 结构化日志落 assets/q3_logs/<run_id>/（events.jsonl / contact-<case>.json /
# run.json），与容器写的作答分开放，避免 answers/ 里混进日志目录。
log_root=${Q3_LOG_DIR:-$project/assets/q3_logs}
mkdir -p "$log_root"
extra+=(-e Q3_LOG_DIR=/logs -v "$log_root:/logs")

if [[ ${Q3_GUI:-0} == 1 ]]; then
    [[ -n ${DISPLAY:-} ]] || die 'Q3_GUI=1 需要图形桌面，但 DISPLAY 为空'
    args+=(--gui); net=host
    extra+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    xauth=""
    for cand in "${XAUTHORITY:-}" "$HOME/.Xauthority" \
                "/run/user/$(id -u)/gdm/Xauthority" /run/user/"$(id -u)"/.mutter-Xwaylandauth.*; do
        [[ -n $cand && -f $cand ]] && { xauth=$cand; break; }
    done
    if [[ -n $xauth ]]; then
        extra+=(-v "$xauth:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
    else
        command -v xhost >/dev/null && xhost +si:localuser:"$(id -un)" >/dev/null 2>&1 || true
        log '未找到 Xauthority，已尝试 xhost 授权'
    fi
fi

if [[ -n ${Q3_DEBUG_PLANS:-} ]]; then
    extra+=(-e Q3_DEBUG_PLANS="$Q3_DEBUG_PLANS")
fi

name="bajie-q3-$$-$(date +%s)"
log "镜像=$image strategy=$strategy 结果目录=$out"
log "挂载：模型/场景只读，仅 $out 可写"

cid=$(docker create --name "$name" --gpus all --network "$net" \
    --memory "$mem_limit" --memory-swap "$mem_limit" \
    --cpus "${Q3_CPUS:-4}" --pids-limit 768 \
    --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=53 \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e OMP_NUM_THREADS="${Q3_THREADS:-2}" -e OPENBLAS_NUM_THREADS="${Q3_THREADS:-2}" \
    -e PYTHONDONTWRITEBYTECODE=1 \
    "${extra[@]}" \
    --mount "type=bind,src=$project,dst=/project,readonly" \
    --mount "type=bind,src=$scene,dst=/scene,readonly" \
    --mount "type=bind,src=$urdf,dst=/model.urdf,readonly" \
    --mount "type=bind,src=$out,dst=/result" \
    --entrypoint "" \
    "$image" bash -c 'source /opt/ros/humble/setup.bash && exec /isaac-sim/python.sh -u "$@"' _ \
    /project/src/question3_project/run_touch.py "${args[@]}")
[[ -n $cid ]] || die '容器创建失败'

# ---- 看门狗：内存/显存/时限触线即停，只停自己创建的容器 ----
# 时限按时长分档：单例约 2~3 分钟，三例整轮约 8~9 分钟（每例先触碰再逆序归位）。
# 固定 360s 会把整轮从第二例起就掐断，这是实测踩到的坑。
if [[ -n ${Q3_TIMEOUT:-} ]]; then
    limit=$Q3_TIMEOUT
elif [[ ${Q3_GUI:-0} == 1 ]]; then
    limit=1800
elif [[ -n $case_name ]]; then
    limit=600
else
    limit=1800
fi
watcher=""
cleanup() {
    [[ -z $watcher ]] || kill "$watcher" 2>/dev/null || true
    docker stop -t 2 "$cid" >/dev/null 2>&1 || true
    docker rm "$cid" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

(
    start=$SECONDS; low=0
    while sleep 5; do
        status=$(docker inspect -f '{{.State.Status}}' "$cid" 2>/dev/null) || break
        [[ $status == running || $status == created ]] || break
        a=$(avail_mib)
        g=$(docker exec "$cid" nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null \
            | awk 'NR==1 {print int($1)}') || g=""
        [[ -n ${g:-} ]] || g=$(host_gpu_free || true)
        [[ $g =~ ^[0-9]+$ ]] || g=99999   # 读不到就不参与判定，避免误杀
        if ((a < 4500 || g < 500)); then low=$((low+1)); else low=0; fi
        printf '[watchdog] available=%s MiB gpu_free=%s MiB\n' "$a" "$g" >&2
        if ((low >= 2 || SECONDS - start >= limit)); then
            printf '[watchdog] 内存、显存或运行时间达到停止线，正在终止仿真。\n' >&2
            docker stop -t 2 "$cid" >/dev/null 2>&1 || true
            break
        fi
    done
) & watcher=$!

rc=0
docker start -a "$cid" || rc=$?
docker inspect -f '[container] exit={{.State.ExitCode}} oom={{.State.OOMKilled}}' "$cid" 2>/dev/null || true
# 结果文件由容器内的 Isaac 进程写出（通常是 root），write_answers 已把它放开到 644，
# 所以调用者一定能读到。这里不再尝试 chown：不同机器的 uid 映射不同（有的 Docker
# 开了 userns 重映射），盲改属主反而可能把它改成宿主 root。
exit "$rc"
