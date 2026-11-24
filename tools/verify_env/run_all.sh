#!/usr/bin/env bash
# 一键复现三题：自检 → 题一 → 题二 → 题三。
#   bash run_all.sh             # 逐题执行，某题失败不影响后面的题
#   bash run_all.sh --q1-only   # 只跑题一
#   bash run_all.sh --skip-q3   # 跳过耗时最长的仿真（无 Isaac Sim 时自动跳过）
set -uo pipefail

project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$project"

do_q1=1; do_q2=1; do_q3=1
for arg in "$@"; do
    case $arg in
        --q1-only) do_q2=0; do_q3=0 ;;
        --skip-q3) do_q3=0 ;;
        -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done

echo "==================== 环境自检 ===================="
bash check_env.sh
echo

declare -A result=()
run() {
    local name=$1; shift
    echo "==================== $name ===================="
    if "$@"; then result[$name]="通过"; else result[$name]="失败(退出码 $?)"; fi
    echo
}

((do_q1)) && run "题一 关节分步驱动" bash run_q1.sh --headless
((do_q2)) && run "题二 正向运动学" bash run_q2.sh --verify
q3_key="题三 Isaac Sim 玩具触碰"
if ((do_q3)); then
    # 原生优先（官方评审环境），不可用时回退到容器；两者跑同一份 run_touch.py。
    if bash src/question3_project/run_q3.sh --check >/dev/null 2>&1; then
        run "$q3_key" env Q3_RESULT_DIR="$project/answers" bash run_q3.sh
    elif bash src/question3_project/run_docker.sh --check >/dev/null 2>&1; then
        q3_key="题三 Isaac Sim 玩具触碰（容器）"
        run "$q3_key" env Q3_RESULT_DIR="$project/answers" bash src/question3_project/run_docker.sh
    else
        echo "==================== 题三 Isaac Sim 玩具触碰 ===================="
        bash src/question3_project/run_q3.sh --check >/dev/null 2>&1 || true
        bash src/question3_project/run_docker.sh --check >/dev/null 2>&1 || true
        result["$q3_key"]="跳过（原生 Isaac Sim 与容器镜像都不可用）"
        echo
    fi
fi

echo "==================== 汇总 ===================="
for k in "题一 关节分步驱动" "题二 正向运动学" "$q3_key"; do
    [[ -n ${result[$k]:-} ]] && printf '  %-28s %s\n' "$k" "${result[$k]}"
done
echo
echo "作答目录：answers/"
echo "运行日志：assets/"
