#!/usr/bin/env python3
"""交付物自检：只读校验三题作答、视频与官方资源指纹。

与 `check_env.sh` 的分工：那个查"机器能不能跑"，这个查"交出去的东西对不对"。
不启动 ROS、不启动 Isaac、不修改任何文件；任何一项 FAIL 时退出码为 1。

检查项：
  1. 必需文件（README / 三题输入 / 三题作答 / 运行日志 / 视频）
  2. 作答结构：id 与输入一一对应、关键帧数量与顺序、十个关节齐全
  3. 数值合理：关节在 URDF 限位内、四元数已归一
  4. 视频体积不超过 30,000,000 字节
  5. 官方资源指纹（URDF xacro 与 scene USD 的 SHA-256）

用法：
    python3 src/question1_project/check_delivery.py
    python3 src/question1_project/check_delivery.py --root /path/to/bajie-demo
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

JOINTS = (
    "lift_joint", "pitch_joint", *(f"joint{i}" for i in range(1, 7)),
    "arm_finger_joint1", "arm_finger_joint2",
)
KEYFRAME_ORDER = ("lift", "head", "arm", "gripper_open", "gripper_close")
VIDEO_LIMIT = 30_000_000
QUAT_TOLERANCE = 1e-6

# 官方仓库入口文件的指纹，与 README「官方资源与指纹」一节一致。
OFFICIAL_SHA256 = {
    "src/bajie_robot/urdf/bajie_robot.urdf.xacro":
        "6dc26b06aafb82aa7030eabf54e5c181b17b4173eb8fc5e64103fbcbdb0d8f49",
    "src/bajie_sim/scene/abr_scene.usd":
        "42d9882469651205dc590d51f5c02b15eb52d29b6424bd0983a11f2743628735",
}

results: list[tuple[str, str, str]] = []


def record(level: str, item: str, note: str) -> None:
    results.append((level, item, note))


def ok(item: str, note: str = "") -> None:
    record("OK", item, note)


def warn(item: str, note: str) -> None:
    record("WARN", item, note)


def fail(item: str, note: str) -> None:
    record("FAIL", item, note)


def rows(path: Path) -> list[dict]:
    items = []
    for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path.name} 第 {number} 行不是合法 JSON：{exc}") from exc
        if not isinstance(value, dict):
            raise ValueError(f"{path.name} 第 {number} 行不是 JSON 对象")
        items.append(value)
    return items


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def check_joint_state(state, where: str, limits: dict[str, tuple[float, float]]) -> bool:
    if not isinstance(state, dict):
        fail(where, "joint_state 不是对象")
        return False
    missing = [name for name in JOINTS if name not in state]
    if missing:
        fail(where, f"缺关节：{', '.join(missing)}")
        return False
    extra = [name for name in state if name not in JOINTS]
    if extra:
        warn(where, f"多余关节：{', '.join(extra)}")
    bad = [name for name in JOINTS if not finite(state[name])]
    if bad:
        fail(where, f"关节不是有限数：{', '.join(bad)}")
        return False
    for name in JOINTS:
        if name not in limits:
            continue
        low, high = limits[name]
        if not low <= state[name] <= high:
            fail(where, f"{name}={state[name]:.6f} 超出 URDF 限位 [{low}, {high}]")
            return False
    return True


def check_pose(pose, where: str) -> bool:
    if not isinstance(pose, dict):
        fail(where, "位姿不是对象")
        return False
    xyz = pose.get("xyz")
    if not (isinstance(xyz, list) and len(xyz) == 3 and all(finite(v) for v in xyz)):
        fail(where, "xyz 必须是三个有限数")
        return False
    quat = pose.get("quat_xyzw")
    if quat is None:
        rpy = pose.get("rpy")
        if not (isinstance(rpy, list) and len(rpy) == 3 and all(finite(v) for v in rpy)):
            fail(where, "既没有合法 quat_xyzw 也没有合法 rpy")
            return False
        return True
    if not (isinstance(quat, list) and len(quat) == 4 and all(finite(v) for v in quat)):
        fail(where, "quat_xyzw 必须是四个有限数（顺序 XYZW）")
        return False
    norm = math.sqrt(sum(v * v for v in quat))
    if abs(norm - 1.0) > QUAT_TOLERANCE:
        fail(where, f"四元数模长 {norm:.9f}，未归一")
        return False
    return True


def urdf_limits(root: Path) -> dict[str, tuple[float, float]]:
    path = root / "src" / "question2_project" / "generated" / "bajie_robot.urdf"
    if not path.is_file():
        warn("URDF 限位", f"缺少 {path.relative_to(root)}，跳过限位检查")
        return {}
    limits = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in JOINTS and limit is not None:
            limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
    if set(limits) != set(JOINTS):
        warn("URDF 限位", f"只解析到 {len(limits)}/{len(JOINTS)} 个比赛关节")
    return limits


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check_required_files(root: Path) -> None:
    required = [
        "README.md",
        *(f"inputs/q{i}_demo_input.jsonl" for i in (1, 2, 3)),
        *(f"answers/q{i}_demo_output.jsonl" for i in (1, 2, 3)),
        *(f"assets/q{i}_run.log" for i in (1, 2, 3)),
    ]
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        fail("必需文件", f"缺 {len(missing)} 个：{', '.join(missing)}")
    else:
        ok("必需文件", f"{len(required)} 个齐全")


def check_videos(root: Path) -> None:
    videos = sorted((root / "video").glob("*.mp4")) if (root / "video").is_dir() else []
    if not videos:
        fail("演示视频", "video/ 下没有 mp4；题一与题三的演示视频是必交材料")
        return
    total = 0
    for path in videos:
        size = path.stat().st_size
        total += size
        if size > VIDEO_LIMIT:
            fail(f"视频 {path.name}", f"{size:,} 字节，超过上限 {VIDEO_LIMIT:,}")
        elif size < 10_000:
            warn(f"视频 {path.name}", f"仅 {size:,} 字节，疑似占位文件")
    if all(path.stat().st_size <= VIDEO_LIMIT for path in videos):
        ok("演示视频", f"{len(videos)} 个，合计 {total:,} 字节")


def check_official_assets(root: Path) -> None:
    for name, expected in OFFICIAL_SHA256.items():
        path = root / name
        if not path.is_file():
            warn("官方资源", f"缺少 {name}，无法核对指纹")
            continue
        actual = sha256(path)
        if actual == expected:
            ok(f"指纹 {Path(name).name}", actual[:16] + "…")
        else:
            fail(f"指纹 {Path(name).name}", f"实际 {actual[:16]}… 与记录不一致")


def check_q1(root: Path, limits: dict) -> None:
    inputs = rows(root / "inputs/q1_demo_input.jsonl")
    answers = rows(root / "answers/q1_demo_output.jsonl")
    if len(answers) != len(inputs):
        fail("题一", f"输入 {len(inputs)} 行、作答 {len(answers)} 行，数量不一致")
        return
    for case, answer in zip(inputs, answers):
        tag = f"题一 {answer.get('id', '?')}"
        if answer.get("id") != case.get("id"):
            fail(tag, f"id 与输入不一致：{answer.get('id')!r} vs {case.get('id')!r}")
            continue
        frames = answer.get("keyframes")
        if not isinstance(frames, list) or len(frames) != len(KEYFRAME_ORDER):
            fail(tag, f"keyframes 应为 {len(KEYFRAME_ORDER)} 个，实际 {type(frames).__name__}")
            continue
        names = [frame.get("name") for frame in frames]
        if tuple(names) != KEYFRAME_ORDER:
            fail(tag, f"关键帧顺序不对：{names}")
            continue
        good = all(check_joint_state(frame.get("joint_state"), f"{tag} · {frame.get('name')}", limits)
                   for frame in frames)
        good &= check_joint_state(answer.get("final_joint_state"), f"{tag} · final_joint_state", limits)
        if good:
            ok(tag, f"{len(frames)} 个关键帧，关节齐全且在限位内")


def check_q2(root: Path) -> None:
    inputs = rows(root / "inputs/q2_demo_input.jsonl")
    answers = rows(root / "answers/q2_demo_output.jsonl")
    if len(answers) != len(inputs):
        fail("题二", f"输入 {len(inputs)} 行、作答 {len(answers)} 行，数量不一致")
        return
    for case, answer in zip(inputs, answers):
        tag = f"题二 {answer.get('id', '?')}"
        if answer.get("id") != case.get("id"):
            fail(tag, f"id 与输入不一致：{answer.get('id')!r} vs {case.get('id')!r}")
            continue
        good = check_pose(answer.get("objects_link_in_base_footprint"), f"{tag} · objects_link_in_base_footprint")
        good &= check_pose(answer.get("scene_object_in_objects_link"), f"{tag} · scene_object_in_objects_link")
        if good:
            ok(tag, "两组位姿完整、四元数已归一")


def check_q3(root: Path, limits: dict) -> None:
    inputs = rows(root / "inputs/q3_demo_input.jsonl")
    answers = rows(root / "answers/q3_demo_output.jsonl")
    if len(answers) != len(inputs):
        fail("题三", f"输入 {len(inputs)} 行、作答 {len(answers)} 行，数量不一致")
        return
    for case, answer in zip(inputs, answers):
        tag = f"题三 {answer.get('id', '?')}"
        if answer.get("id") != case.get("id"):
            fail(tag, f"id 与输入不一致：{answer.get('id')!r} vs {case.get('id')!r}")
            continue
        good = check_joint_state(answer.get("final_joint_state"), f"{tag} · final_joint_state", limits)
        good &= check_pose(answer.get("objects_link_in_base_footprint"), f"{tag} · objects_link_in_base_footprint")
        good &= check_pose(answer.get("scene_object_in_objects_link"), f"{tag} · scene_object_in_objects_link")
        if good:
            ok(tag, "关节状态完整、两组位姿齐全")


def write_manifest(root: Path, target: Path) -> None:
    """把作答、日志、视频、题解源码与模型指纹写成一份哈希清单。

    这是本脚本唯一的写操作，且只在显式传 --manifest 时发生；默认运行是只读的。
    """
    patterns = [
        "answers/*.jsonl", "assets/*.log", "assets/q3_logs/**/*.json", "video/*",
        "inputs/*.jsonl",
        "src/question1_project/**/*.py", "src/question1_project/*.sh",
        "src/question2_project/**/*.py", "src/question2_project/*.sh",
        "src/question3_project/**/*.py", "src/question3_project/*.sh",
        "src/question3_project/*.kit", "src/question2_project/generated/bajie_robot.urdf",
    ]
    items = {}
    for pattern in patterns:
        for path in sorted(root.glob(pattern)):
            if not path.is_file() or "__pycache__" in path.parts:
                continue
            relative = path.relative_to(root).as_posix()
            items[relative] = {"bytes": path.stat().st_size, "sha256": sha256(path)}
    manifest = {
        "description": "八界杯初赛交付物哈希清单；answers 与 assets 逐项绑定，供评审回溯",
        "generated_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "official_assets": {
            name: {"expected_sha256": expected,
                   "actual_sha256": sha256(root / name) if (root / name).is_file() else None}
            for name, expected in OFFICIAL_SHA256.items()
        },
        "files": items,
    }
    target.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"清单已写入 {target}（{len(items)} 个文件）")


def check_q3_evidence(root: Path) -> None:
    """题三每一行答案都应能指回一次运行的一份接触快照。"""
    snapshots = sorted((root / "assets" / "q3_logs").glob("*/contact-*.json"))
    if not snapshots:
        warn("题三证据链", "assets/q3_logs 下没有 contact-*.json：作答暂时无法逐例指回接触快照，"
                          "下一次整轮运行会自动生成")
        return
    runs = sorted({path.parent.name for path in snapshots})
    ok("题三证据链", f"{len(snapshots)} 份接触快照，来自 {len(runs)} 次运行（最新 {runs[-1]}）")


def main() -> int:
    parser = argparse.ArgumentParser(description="八界杯初赛交付物自检")
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[2]),
                        help="作品根目录（默认为本脚本上两级，即 src/question1_project 的父目录的父目录）")
    parser.add_argument("--manifest", default=None,
                        help="生成哈希清单到指定路径（默认不写任何文件）")
    args = parser.parse_args()
    root = Path(args.root).resolve()

    check_required_files(root)
    limits = urdf_limits(root)
    for name, function in (("题一", lambda: check_q1(root, limits)),
                           ("题二", lambda: check_q2(root)),
                           ("题三", lambda: check_q3(root, limits))):
        try:
            function()
        except Exception as exc:  # 结构异常不应中断其余检查
            fail(name, f"检查中断：{exc}")
    check_videos(root)
    check_q3_evidence(root)
    check_official_assets(root)
    if args.manifest:
        write_manifest(root, Path(args.manifest).resolve())

    width = max(len(item) for _, item, _ in results)
    for level, item, note in results:
        print(f"[{level}] {item:<{width}}  {note}")
    failures = sum(1 for level, _, _ in results if level == "FAIL")
    warnings = sum(1 for level, _, _ in results if level == "WARN")
    print(f"\n合计 {len(results)} 项：OK {len(results) - failures - warnings} / "
          f"WARN {warnings} / FAIL {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
