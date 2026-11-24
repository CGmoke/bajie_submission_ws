#!/usr/bin/env python3
"""提交工作区核查：对照《八界杯创新挑战赛》初赛题面第三、二章逐项判定。

只读，不修改被核查目录。退出码 1 表示存在 FAIL。

用法：
    python3 check_submission.py --root /home/moke/bajie_submission_ws/bajie-demo
    python3 check_submission.py --root ... --source /home/moke/bajie_ws/bajie-demo
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from xml.etree import ElementTree as ET

JOINTS = (
    "lift_joint", "pitch_joint", *(f"joint{i}" for i in range(1, 7)),
    "arm_finger_joint1", "arm_finger_joint2",
)
KEYFRAMES = ("lift", "head", "arm", "gripper_open", "gripper_close")
VIDEO_LIMIT = 30_000_000
QUAT_TOL = 1e-6
BUILD_DIRS = ("build", "install", "log")
# 官方仓库自带、非本项目遗留：已与 ecovacs-bajie-urdf@907b061 的 meshes/ 逐项比对确认
UPSTREAM_BACKUPS = {
    "src/bajie_robot/meshes/arm_base_link.STL.bk",
    "src/bajie_robot/meshes/berxel_arm_base_link.STL.bk",
    "src/bajie_robot/meshes/berxel_upper_base_link.STL.bk",
    "src/bajie_robot/meshes/tp55_base_Link.STL.unuse",
}

rows: list[tuple[str, str, str, str]] = []


def rec(level: str, item: str, note: str, fix: str = "") -> None:
    rows.append((level, item, note, fix))


def ok(item: str, note: str = "") -> None:
    rec("OK", item, note)


def warn(item: str, note: str, fix: str = "") -> None:
    rec("WARN", item, note, fix)


def fail(item: str, note: str, fix: str = "") -> None:
    rec("FAIL", item, note, fix)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def read_jsonl(path: Path) -> list[dict]:
    out = []
    text = path.read_text(encoding="utf-8-sig")
    for i, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path.name} 第 {i} 行不是 JSON 对象")
        out.append(value)
    return out


# ---------- C1 复制完整性 ----------
# 只比对"不应变动的部分"：官方资源、三题源码与配置、题目输入、展开后的 URDF。
# README、docs/、answers/、assets/、video/ 属交付前按实际情况更新的材料，变动是预期的。
MUTABLE_DIRS = ("answers", "assets", "video", "docs")


def check_copy(root: Path, source: Path | None) -> None:
    if source is None or not source.is_dir():
        warn("复制完整性", "未提供源目录，跳过逐文件比对")
        return

    def listing(base: Path) -> dict[str, str]:
        got = {}
        for p in base.rglob("*"):
            rel = p.relative_to(base)
            if not p.is_file():
                continue
            if rel.parts[0] in BUILD_DIRS or rel.parts[0] in MUTABLE_DIRS:
                continue
            if "__pycache__" in rel.parts or p.suffix == ".pyc" or rel.parts[0] == ".git":
                continue
            if p.suffix.lower() in (".md", ".json", ".log"):
                continue
            got[rel.as_posix()] = sha256(p)
        return got

    src, dst = listing(source), listing(root)
    missing = sorted(set(src) - set(dst))
    extra = sorted(set(dst) - set(src))
    changed = sorted(k for k in set(src) & set(dst) if src[k] != dst[k])
    if missing or changed:
        fail("复制完整性", f"缺 {len(missing)} / 内容不同 {len(changed)}"
                          + (f" / 新增 {len(extra)}" if extra else ""),
             "重新执行 rsync -a 并排除 build/ install/ log/")
        for name in (missing + changed)[:5]:
            print(f"    · {name}")
    else:
        note = f"官方资源/源码/输入 {len(dst) - len(extra)} 个文件逐一致（sha256 全同）"
        if extra:
            note += f"；新增 {len(extra)} 项：{', '.join(extra[:4])}"
        ok("复制完整性", note)


# ---------- C2 目录结构 ----------
def check_structure(root: Path) -> None:
    required = {
        "README.md": "文件",
        "src/question1_project": "目录",
        "src/question2_project": "目录",
        "src/question3_project": "目录",
        "inputs/q1_demo_input.jsonl": "文件",
        "inputs/q2_demo_input.jsonl": "文件",
        "inputs/q3_demo_input.jsonl": "文件",
        "answers/q1_demo_output.jsonl": "文件",
        "answers/q2_demo_output.jsonl": "文件",
        "answers/q3_demo_output.jsonl": "文件",
        "docs": "目录",
        "assets": "目录",
        "video": "目录",
    }
    missing = [n for n, kind in required.items()
               if not (root / n).is_dir() if kind == "目录"] + \
              [n for n, kind in required.items()
               if not (root / n).is_file() if kind == "文件"]
    if missing:
        fail("§3.1 目录结构", f"缺 {len(missing)} 项：{', '.join(missing)}",
             "按题面 3.1 的树状图补齐缺失条目")
    else:
        ok("§3.1 目录结构", f"{len(required)} 个必备条目齐全")

    empty = [n for n, kind in required.items()
             if kind == "目录" and (root / n).is_dir() and not any((root / n).iterdir())]
    if empty:
        fail("目录非空", f"空目录：{', '.join(empty)}", "空目录视为未提供该项材料")


# ---------- C3 命名 ----------
def check_naming(root: Path) -> None:
    name = root.name
    if name.lower() in {"bajie-demo", "bajie_demo", "demo", "submit", "submission"}:
        fail("作品目录名", f"当前为 `{name}`，题面 3.1 明确此处须用报名姓名（bajie-demo 只是示例）",
             "把目录改名为报名时填写的姓名，例如 `张三/`；后续所有命令中的 `cd bajie-demo` 同步替换")
    else:
        ok("作品目录名", f"`{name}` 非题面占位名（是否与报名姓名一致需人工确认）")

    bad = [p.name for p in (root / "inputs").glob("*")
           if p.is_file() and not re.fullmatch(r"q[123]_demo_input\.jsonl", p.name)]
    bad += [p.name for p in (root / "answers").glob("*")
            if p.is_file() and not re.fullmatch(r"q[123]_demo_output\.jsonl", p.name)]
    if bad:
        fail("文件命名", f"inputs/answers 下有题面未定义的文件：{', '.join(sorted(bad))}",
             "题面只认 q{1,2,3}_demo_input.jsonl 与 q{1,2,3}_demo_output.jsonl")
    else:
        ok("文件命名", "inputs/answers 命名与题面一致")


# ---------- C4/C5 构建产物与杂物 ----------
def check_artifacts(root: Path) -> None:
    leaked = [d for d in BUILD_DIRS if (root / d).exists()]
    if leaked:
        fail("构建产物泄漏", f"存在 {', '.join(leaked)}/：colcon 输出不属于交付材料",
             "删除这三个目录（可由 run_*.sh 重新生成），并在 .gitignore 中保留忽略规则")
    else:
        ok("构建产物", "无 build/ install/ log/ 残留")

    junk = []
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        rel = p.relative_to(root).as_posix()
        if rel in UPSTREAM_BACKUPS:
            continue
        if p.suffix in {".bk", ".unuse", ".orig", ".bak", ".rej"}:
            junk.append(rel)
        elif "__pycache__" in p.parts or p.suffix == ".pyc":
            junk.append(rel)
    if junk:
        warn("冗余文件", f"{len(junk)} 个备份/缓存文件，如 {', '.join(junk[:4])}",
             "确认为官方原样（会破坏指纹一致性）则保留并在 README 说明，否则删除")
    else:
        ok("冗余文件", f"无本项目遗留的备份/缓存；官方自带的 {len(UPSTREAM_BACKUPS)} 个 .bk/.unuse 已确认属上游原样")


# ---------- C6 README 必备内容 ----------
def check_readme(root: Path) -> None:
    path = root / "README.md"
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8", errors="replace")
    checks = {
        "作品名称": bool(re.search(r"^# \S", text, re.M)),
        "参赛者/团队": ("参赛者" in text) and not re.search(r"参赛者[^\n]*待填写", text),
        "功能与创新点": ("功能亮点" in text or "创新" in text)
                       and not re.search(r"功能亮点[\s\S]{0,400}?待填写", text),
        "依赖版本": any(k in text for k in ("环境要求", "依赖版本", "版本清单")),
        "各题启动命令": all(f"run_q{i}.sh" in text for i in (1, 2, 3)),
        "执行目录": bool(re.search(r"^\s*cd\s+\S+|执行目录", text, re.M)),
        "输入输出路径": ("inputs/" in text) and ("answers/" in text),
        "复现步骤": "复刻" in text or "复现" in text,
    }
    bad = [k for k, v in checks.items() if not v]
    if bad:
        fail("§3.2 README 要素", f"缺/未填：{', '.join(bad)}",
             "题面要求 README 写明作品名称、参赛者、功能与创新点、依赖版本、启动命令、执行目录、"
             "输入输出路径、复现步骤；逐项补齐，占位文字（待填写/待补）不算完成")
    else:
        ok("§3.2 README 要素", f"{len(checks)} 项齐全")

    vids = sorted(p.name for p in (root / "video").glob("*.mp4"))
    if vids:
        unnamed = [v for v in vids if v not in text]
        if unnamed:
            warn("README 视频清单", f"README 未写明视频文件名：{', '.join(unnamed)}",
                 "在 README 标注每个 mp4 的文件名与对应题目")

    # 内部引用是否有效
    refs = set(re.findall(r"`((?:scripts|docs|src|assets|inputs|answers)/[A-Za-z0-9_./\-\u4e00-\u9fff]+)`", text))
    dead = sorted(r for r in refs if not (root / r.rstrip("/")).exists())
    if dead:
        warn("README 失效引用", f"{len(dead)} 处引用指向不存在的路径：{', '.join(dead)}",
             "改为目录内真实路径，或补交被引用的文件；评审按 README 复现时会直接踩空")


# ---------- C7/C8/C9 作答格式 ----------
def urdf_limits(root: Path) -> dict[str, tuple[float, float]]:
    path = root / "generated" / "bajie_robot.urdf"
    if not path.is_file():
        warn("URDF 限位", "缺少 generated/bajie_robot.urdf，跳过限位校验")
        return {}
    out = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") in JOINTS and limit is not None:
            out[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return out


def check_joint_state(state, where: str, limits: dict) -> bool:
    if not isinstance(state, dict):
        fail(where, "joint_state 不是对象")
        return False
    missing = [j for j in JOINTS if j not in state]
    if missing:
        fail(where, f"缺关节 {len(missing)} 个：{', '.join(missing)}")
        return False
    bad = [j for j in JOINTS if not finite(state[j])]
    if bad:
        fail(where, f"非有限数：{', '.join(bad)}")
        return False
    over = [(j, state[j], limits[j]) for j in JOINTS
            if j in limits and not limits[j][0] <= state[j] <= limits[j][1]]
    if over:
        fail(where, f"超限位：{over[0][0]}={over[0][1]:.6f} ∉ {over[0][2]}")
        return False
    extra = [k for k in state if k not in JOINTS]
    if extra:
        warn(where, f"多余字段：{', '.join(extra)}")
    return True


def check_pose(pose, where: str) -> bool:
    if not isinstance(pose, dict):
        fail(where, "位姿不是对象")
        return False
    xyz, rpy, quat = pose.get("xyz"), pose.get("rpy"), pose.get("quat_xyzw")
    good = True
    if not (isinstance(xyz, list) and len(xyz) == 3 and all(finite(v) for v in xyz)):
        fail(where, "xyz 须为 3 个有限数")
        good = False
    if not (isinstance(rpy, list) and len(rpy) == 3 and all(finite(v) for v in rpy)):
        fail(where, "rpy 须为 3 个有限数（弧度）")
        good = False
    if not (isinstance(quat, list) and len(quat) == 4 and all(finite(v) for v in quat)):
        fail(where, "quat_xyzw 须为 4 个有限数（顺序 xyzw）")
        return False
    norm = math.sqrt(sum(v * v for v in quat))
    if abs(norm - 1.0) > QUAT_TOL:
        fail(where, f"四元数模长 {norm:.9f} 未归一")
        good = False
    return good


def check_answers(root: Path) -> None:
    limits = urdf_limits(root)
    for q in (1, 2, 3):
        inp = root / f"inputs/q{q}_demo_input.jsonl"
        outp = root / f"answers/q{q}_demo_output.jsonl"
        if not (inp.is_file() and outp.is_file()):
            continue
        try:
            cases, ans = read_jsonl(inp), read_jsonl(outp)
        except Exception as exc:
            fail(f"题{q} JSONL", f"解析失败：{exc}", "每行一个完整对象，去掉多余换行与注释")
            continue
        if len(ans) != len(cases):
            fail(f"题{q} 行数", f"输入 {len(cases)} 行 / 作答 {len(ans)} 行",
                 "每个测例输出一行，id 与输入一一对应")
            continue
        if [a.get("id") for a in ans] != [c.get("id") for c in cases]:
            fail(f"题{q} id", f"作答 id {[a.get('id') for a in ans]} 与输入 "
                              f"{[c.get('id') for c in cases]} 不一致或顺序不同",
                 "沿用输入 id 且保持输入顺序")
            continue

        if q == 1:
            for case, a in zip(cases, ans):
                tag = f"题一 {a['id']}"
                if a.get("step") != case.get("step"):
                    fail(tag, f"step={a.get('step')!r} 与输入 {case.get('step')!r} 不一致")
                    continue
                frames = a.get("keyframes")
                if not isinstance(frames, list) or len(frames) != 5:
                    fail(tag, f"keyframes 须为 5 个，实际 {len(frames) if isinstance(frames, list) else type(frames).__name__}")
                    continue
                names = [f.get("name") for f in frames]
                if tuple(names) != KEYFRAMES:
                    fail(tag, f"关键帧顺序须为 {list(KEYFRAMES)}，实际 {names}")
                    continue
                good = all(check_joint_state(f.get("joint_state"), f"{tag}·{f.get('name')}", limits)
                           for f in frames)
                good &= check_joint_state(a.get("final_joint_state"), f"{tag}·final_joint_state", limits)
                if good:
                    ok(tag, "5 个关键帧顺序正确，10 关节齐全且在限位内")

        elif q == 2:
            for a in ans:
                tag = f"题二 {a['id']}"
                good = check_pose(a.get("objects_link_in_base_footprint"),
                                  f"{tag}·objects_link_in_base_footprint")
                good &= check_pose(a.get("scene_object_in_objects_link"),
                                   f"{tag}·scene_object_in_objects_link")
                if good:
                    ok(tag, "两组位姿齐全、四元数归一")

        else:
            for a in ans:
                tag = f"题三 {a['id']}"
                good = check_joint_state(a.get("final_joint_state"), f"{tag}·final_joint_state", limits)
                good &= check_pose(a.get("objects_link_in_base_footprint"),
                                   f"{tag}·objects_link_in_base_footprint")
                has_toy = "scene_object_in_objects_link" in a
                if not has_toy:
                    fail(tag, "缺 scene_object_in_objects_link（题面要求报告玩具在 objects_link 中的位姿）")
                    good = False
                elif check_pose(a.get("scene_object_in_objects_link"),
                                f"{tag}·scene_object_in_objects_link"):
                    pass
                else:
                    good = False
                if good:
                    ok(tag, "关节状态完整、两组位姿齐全（含玩具在 objects_link 的位姿）")


# ---------- C10 视频 ----------
def check_video(root: Path) -> None:
    folder = root / "video"
    mp4s = sorted(folder.glob("*.mp4")) if folder.is_dir() else []
    if not mp4s:
        fail("§3.2 演示视频", "video/ 下没有 mp4；题一 RViz 与题三 Isaac Sim 触碰为必交",
             "录 q1_rviz.mp4（run_q1.sh --gui）与 q3_touch.mp4（run_q3.sh --gui），"
             "各 ≤ 30,000,000 字节，MP4/H.264，README 写明文件名与对应题目")
        return
    total = 0
    for p in mp4s:
        size = p.stat().st_size
        total += size
        if size > VIDEO_LIMIT:
            fail(f"视频 {p.name}", f"{size:,} 字节 > 30,000,000",
                 "用 HandBrake 以平均码率重压到约 27 MB（720p / 25~30 fps，AAC 64–96 kbps）")
        else:
            ok(f"视频 {p.name}", f"{size:,} 字节（{(size / 1e6):.1f} MB）")
        if subprocess.run(["which", "ffprobe"], capture_output=True).returncode == 0:
            r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                                "-show_entries", "stream=codec_name,width,height",
                                "-of", "default=nw=1", str(p)], capture_output=True, text=True)
            info = dict(line.split("=", 1) for line in r.stdout.strip().splitlines() if "=" in line)
            if info.get("codec_name") not in {"h264", "hevc"}:
                warn(f"视频编码 {p.name}", f"codec={info.get('codec_name')}，题面建议 H.264/H.265")
    names = " ".join(p.name for p in mp4s)
    for need in ("q1", "q3"):
        if need not in names:
            fail(f"视频覆盖 {need}", f"未见对应题目的视频：{names}",
                 "题一与题三各需一段实录，文件名建议 q1_rviz.mp4 / q3_touch.mp4")


# ---------- C11 日志 ----------
def check_logs(root: Path) -> None:
    for q in (1, 2, 3):
        p = root / f"assets/q{q}_run.log"
        if not p.is_file():
            fail(f"题{q} 运行日志", f"缺 assets/q{q}_run.log", "复跑该题，脚本会把完整终端输出 tee 进 assets/")
        elif p.stat().st_size < 200:
            warn(f"题{q} 运行日志", f"仅 {p.stat().st_size} 字节，疑似不完整",
                 "以一次完整运行的终端输出覆盖")
        else:
            ok(f"题{q} 运行日志", f"{p.stat().st_size:,} 字节")


# ---------- C12 敏感信息与体积 ----------
def check_safety(root: Path) -> None:
    patterns = re.compile(r"(ghp_[A-Za-z0-9]{20,}|nvapi-[A-Za-z0-9_\-]{20,}|"
                          r"AKIA[0-9A-Z]{16}|-----BEGIN [A-Z ]*PRIVATE KEY-----)")
    hits = []
    for p in root.rglob("*"):
        if not p.is_file() or p.stat().st_size > 2_000_000:
            continue
        if p.suffix.lower() in {".stl", ".usd", ".png", ".jpg", ".mp4", ".urdf"}:
            continue
        try:
            if patterns.search(p.read_text(encoding="utf-8", errors="ignore")):
                hits.append(p.relative_to(root).as_posix())
        except OSError:
            continue
    if hits:
        fail("敏感信息", f"疑似凭证：{', '.join(hits[:5])}",
             "题面 3.2 要求移除密钥与访问令牌；撤销该凭证后从历史中清除再提交")
    else:
        ok("敏感信息", "未发现疑似密钥/令牌")

    big = sorted(((p.stat().st_size, p.relative_to(root).as_posix())
                  for p in root.rglob("*") if p.is_file()), reverse=True)[:3]
    total = sum(s for s, _ in ((p.stat().st_size, p) for p in root.rglob("*") if p.is_file()))
    note = "；".join(f"{n} {s / 1e6:.0f} MB" for s, n in big)
    if total > 1_500_000_000:
        warn("提交体积", f"合计 {total / 1e9:.2f} GB（最大：{note}）",
             "题面允许大体积依赖改附 README 链接；AtomGit 单仓有限制，需评估是否外链官方模型")
    else:
        ok("提交体积", f"合计 {total / 1e9:.2f} GB（最大：{note}）")


# ---------- C13 文档与事实一致性 ----------
def check_doc_consistency(root: Path) -> None:
    readme = root / "README.md"
    snapshots = sorted((root / "assets" / "q3_logs").glob("*/contact-*.json"))
    runs = sorted({p.parent.name for p in snapshots})
    if readme.is_file():
        text = readme.read_text(encoding="utf-8", errors="replace")
        m = re.search(r"`assets/q3_logs/`[^\n]*?(\d+)\s*份接触快照[^\n]*?(\d+)\s*次运行", text)
        if m and (int(m.group(1)) != len(snapshots) or int(m.group(2)) != len(runs)):
            warn("README 状态描述", f"README 称 {m.group(1)} 份快照 / {m.group(2)} 次运行，"
                                    f"实际 {len(snapshots)} 份 / {len(runs)} 次",
                 "以最近一次整轮运行为准改写这段话，避免声明与证据不符")
    incomplete = [d.name for d in (root / "assets" / "q3_logs").iterdir()
                  if d.is_dir() and not (d / "run.json").is_file()] if (root / "assets" / "q3_logs").is_dir() else []
    if incomplete:
        warn("题三运行目录", f"{len(incomplete)} 个目录缺 run.json（中断运行）：{', '.join(incomplete[:3])}",
             "删除中断目录，或只保留与作答对应的那一次运行，减少评审歧义")


def _rot(rpy: list[float]) -> list[list[float]]:
    """RPY（绕固定轴 x→y→z）转旋转矩阵：R = Rz(y)Ry(p)Rx(r)。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _in_base(hand: list[float], base_xyz: list[float], base_rpy: list[float]) -> list[float]:
    """世界坐标点 → 底盘坐标系（先减平移，再乘旋转的转置）。"""
    d = [hand[i] - base_xyz[i] for i in range(3)]
    r = _rot(base_rpy)
    return [sum(r[j][i] * d[j] for j in range(3)) for i in range(3)]


def check_q3_evidence(root: Path) -> None:
    """作答的关节值是否能在某次运行的接触快照里找到；末端位姿是否一致。"""
    ans_path = root / "answers/q3_demo_output.jsonl"
    logs = root / "assets/q3_logs"
    if not ans_path.is_file() or not logs.is_dir():
        return
    answers = {o["id"]: o for o in read_jsonl(ans_path)}
    inp_path = root / "inputs/q3_demo_input.jsonl"
    base_rpy: dict[str, list[float]] = {}
    if inp_path.is_file():
        for case in read_jsonl(inp_path):
            base_rpy[case["id"]] = (case.get("base_footprint_in_world") or {}).get("rpy") or [0.0, 0.0, 0.0]
    per_case: dict[str, tuple[float, str]] = {}
    best_pose: dict[str, float] = {}
    for cid, a in answers.items():
        best = (9e9, "")
        for run in sorted(p for p in logs.iterdir() if p.is_dir()):
            snap = run / f"contact-{cid}.json"
            if not snap.is_file():
                continue
            c = json.loads(snap.read_text(encoding="utf-8"))
            joints = c.get("joints", {})
            delta = max((abs(joints[k] - a["final_joint_state"][k])
                         for k in a["final_joint_state"] if k in joints), default=9e9)
            if delta < best[0] - 1e-12 or (abs(delta - best[0]) <= 1e-12 and run.name > best[1]):
                best = (delta, run.name)  # 同分取最新一次运行，避免指向旧记录
            hand, base = c.get("hand", {}), c.get("base", {})
            ref = a.get("objects_link_in_base_footprint", {}).get("xyz")
            rpy = c.get("base", {}).get("rpy") or base_rpy.get(cid) or [0.0, 0.0, 0.0]
            if hand.get("xyz") and base.get("xyz") and ref and len(rpy) == 3:
                got = _in_base(hand["xyz"], base["xyz"], rpy)
                d = max(abs(got[i] - ref[i]) for i in range(3))
                best_pose[cid] = min(best_pose.get(cid, 9e9), d)
        per_case[cid] = best

    if best_pose:
        worst_pose = max(best_pose.values())
        if worst_pose < 2e-3:
            ok("题三证据链·位姿", f"三例末端位姿均能在运行记录里对上（最大偏差 {worst_pose * 1000:.2f} mm）")
        else:
            miss = [k for k, v in best_pose.items() if v >= 2e-3]
            warn("题三证据链·位姿", f"{', '.join(miss)} 的末端位姿与任何运行都差 {worst_pose * 1000:.1f} mm",
                 "确认作答与 assets/q3_logs 出自同一套运行，否则重跑整轮覆盖作答")

    exact = {cid: run for cid, (d, run) in per_case.items() if d < 1e-6}
    worst = max((d for d, _ in per_case.values()), default=0.0)
    if len(exact) == len(answers):
        with_summary = [r for r in set(exact.values()) if (logs / r / "run.json").is_file()]
        if with_summary:
            ok("题三证据链·关节", f"三例关节值均与 {with_summary[-1]} 逐值一致，该次有 run.json")
        else:
            warn("题三证据链·关节", "三例关节值可逐值追到运行，但对应运行目录没有 run.json，三例 outcome 无处可查",
                 "以一次完整整轮运行重新生成作答，保留其 run.json 与 contact-*.json")
    elif exact:
        detail = "、".join(f"{cid.split('_')[-1]}→{run}" for cid, run in exact.items())
        warn("题三证据链·关节", f"只有 {len(exact)}/{len(answers)} 例能逐值追到运行（{detail}）；"
                              f"其余例与最接近的一次相差 ≤ {worst:.1e} rad",
             "作答应整轮一次生成；分批产生或事后手改会让「答案行 ↔ 接触快照 ↔ run.json」对不上，"
             "评审容易判为不可复现")
    else:
        warn("题三证据链·关节", f"作答关节值与所有运行都不完全相同（最接近差 {worst:.1e} rad）",
             "重跑题三整轮并覆盖 answers/q3_demo_output.jsonl，使作答与日志出自同一 run_id")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--source", default=None)
    a = ap.parse_args()
    root = Path(a.root).resolve()
    source = Path(a.source).resolve() if a.source else None

    check_copy(root, source)
    check_structure(root)
    check_naming(root)
    check_artifacts(root)
    check_readme(root)
    check_answers(root)
    check_video(root)
    check_logs(root)
    check_safety(root)
    check_doc_consistency(root)
    check_q3_evidence(root)

    width = max(len(i) for _, i, _, _ in rows)
    for level, item, note, fix in rows:
        print(f"[{level}] {item:<{width}}  {note}")
        if fix and level != "OK":
            print(f"{'':<8}{'↳ 修正':<{width}}  {fix}")
    fails = sum(1 for r in rows if r[0] == "FAIL")
    warns = sum(1 for r in rows if r[0] == "WARN")
    print(f"\n合计 {len(rows)} 项：OK {len(rows) - fails - warns} / WARN {warns} / FAIL {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
