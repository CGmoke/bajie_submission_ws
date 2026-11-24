#!/usr/bin/env python3
"""结构化运行日志：一行一个 JSON 事件、每例一份接触快照、整轮一份汇总。

为什么不是纯 print：`answers/q3_demo_output.jsonl` 里的一行必须能指回某一次
运行的某一帧。只用 stdout 时，这条链要靠人眼在几百行 Kit 输出里找；这里把
三件事拆开落盘：

  events.jsonl          本次运行的全部自有事件（小、干净、可 grep/jq）
  contact-<case>.json   接触瞬间的关节、base/hand/toy 位姿、接触对与间隙
  run.json              整轮汇总：三例 outcome、策略、耗时、是否写盘

Isaac Kit 自身的原始输出仍然原样留在启动器 tee 的日志里，不做删减——那是
证据；这里只是另建一条机器可读、体积可控的通道。

只依赖标准库，便于在没有 Isaac 的环境里跑单元测试。
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import secrets
import time


def _plain(value):
    """把 numpy 数组/标量、Path、元组等转成 JSON 可序列化的形式。"""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    for attribute in ('tolist', 'item'):
        method = getattr(value, attribute, None)
        if callable(method):
            return _plain(method())
    return str(value)


def pose(matrix) -> dict:
    """4x4 齐次变换 -> {xyz, quat_xyzw}；输入可为 ndarray 或嵌套列表。"""
    rows = matrix.tolist() if hasattr(matrix, 'tolist') else [list(r) for r in matrix]
    xyz = [float(rows[i][3]) for i in range(3)]
    r = [[float(rows[i][j]) for j in range(3)] for i in range(3)]
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (r[2][1] - r[1][2]) / s
        y, z = (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w, x = (r[2][1] - r[1][2]) / s, 0.25 * s
        y, z = (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w, x = (r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s
        y, z = 0.25 * s, (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w, x = (r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s
        y, z = (r[1][2] + r[2][1]) / s, 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return {"xyz": xyz, "quat_xyzw": [x / norm, y / norm, z / norm, w / norm]}


class Journal:
    """一次运行的日志出口。directory 为空时退化为只打印，不落盘。"""

    LEVELS = {'debug': 10, 'info': 20, 'warn': 30, 'error': 40}

    def __init__(self, directory=None, run_id=None, level='info', stream=None):
        self.run_id = run_id or (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                                 + '-' + secrets.token_hex(3))
        self.level = level if level in self.LEVELS else 'info'
        self.started_utc = datetime.now(timezone.utc)
        self.started_mono = time.monotonic()
        self.cases: dict[str, dict] = {}
        self._stream = stream
        # 每次运行独立子目录，避免上一轮 events.jsonl 被覆盖后无法对照。
        self.directory = Path(directory) / self.run_id if directory else None
        self.events_path = None
        if self.directory:
            self.directory.mkdir(parents=True, exist_ok=True)
            self.events_path = self.directory / 'events.jsonl'
            self.events_path.write_text('', encoding='utf-8')
        self.event('info', 'run_start', directory=str(self.directory) if self.directory else None)

    # ---------- 输出 ----------
    def _emit(self, line: str) -> None:
        if self._stream is None:
            print(line, flush=True)
        else:
            self._stream.write(line + '\n')
            self._stream.flush()

    def event(self, level: str, message: str, case: str | None = None, **fields) -> dict:
        """记一条事件：stdout 走人类可读格式，同时追加一行 JSON。"""
        if self.LEVELS.get(level, 20) < self.LEVELS[self.level]:
            return {}
        now = datetime.now(timezone.utc)
        record = {
            'ts': now.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z',
            'mono_s': round(time.monotonic() - self.started_mono, 3),
            'run': self.run_id,
            'level': level,
            'msg': message,
        }
        if case:
            record['case'] = case
        payload = {k: _plain(v) for k, v in fields.items() if v is not None}
        record.update(payload)
        if self.events_path:
            with self.events_path.open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        prefix = f'[q3 {record["mono_s"]:7.3f}s]'
        if case:
            prefix += f' {case}:'
        parts = [f'{k}={_short(v)}' for k, v in payload.items() if k != 'detail']
        if payload.get('detail'):
            parts.append(f'detail={payload["detail"]}')
        self._emit(' '.join([prefix, message, *parts]))
        return record

    # ---------- 每例 ----------
    def case_start(self, case: str, strategy: str, toy: str | None = None) -> None:
        self.cases.setdefault(case, {'id': case})['strategy'] = strategy
        self.event('info', 'case_start', case=case, strategy=strategy, toy=toy)

    def case_outcome(self, case: str, outcome: str, phase: str, **fields) -> None:
        record = self.cases.setdefault(case, {'id': case})
        record.update({'outcome': outcome, 'phase': phase})
        record.update({k: _plain(v) for k, v in fields.items() if v is not None})
        self.event('info' if outcome == 'contact' else 'warn', 'case_outcome',
                   case=case, outcome=outcome, phase=phase, **fields)

    def contact(self, case: str, phase: str, joints: dict, base, hand, toy,
                contact: dict | None = None, **fields) -> Path | None:
        """写 contact-<case>.json。返回路径；未配置目录时返回 None。"""
        snapshot = {
            'run': self.run_id,
            'case': case,
            'phase': phase,
            'strategy': self.cases.get(case, {}).get('strategy'),
            'joints': {k: float(v) for k, v in joints.items()},
            'base': pose(base),
            'hand': pose(hand),
            'toy': pose(toy),
            'contact': _plain(contact),
            'mono_s': round(time.monotonic() - self.started_mono, 3),
            'ts': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z',
        }
        snapshot.update({k: _plain(v) for k, v in fields.items() if v is not None})
        if not self.directory:
            return None
        path = self.directory / f'contact-{case}.json'
        path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                        encoding='utf-8')
        self.cases.setdefault(case, {'id': case})['contact_file'] = path.name
        self.event('info', 'contact_snapshot', case=case, phase=phase, path=path.name)
        return path

    # ---------- 收尾 ----------
    def finish(self, rows=None, misses=None, exit_state: str = 'ok', **fields) -> Path | None:
        finished = datetime.now(timezone.utc)
        summary = {
            'run': self.run_id,
            'started_utc': self.started_utc.strftime('%Y-%m-%dT%H:%M:%SZ'),
            'finished_utc': finished.strftime('%Y-%m-%dT%H:%M:%SZ'),
            'duration_s': round((finished - self.started_utc).total_seconds(), 3),
            'exit': exit_state,
            'cases': [self.cases[key] for key in sorted(self.cases)],
            'written_rows': len(rows or []),
            'misses': _plain([f'{case.name}: {exc}' for (case, _), exc in (misses or [])]),
        }
        summary.update({k: _plain(v) for k, v in fields.items() if v is not None})
        self.event('info' if exit_state == 'ok' else 'error', 'run_finish',
                   exit=exit_state, **{k: summary[k] for k in ('duration_s', 'written_rows') if k in summary})
        if not self.directory:
            return None
        path = self.directory / 'run.json'
        path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                        encoding='utf-8')
        return path


def _short(value) -> str:
    if isinstance(value, float):
        return f'{value:.4f}'.rstrip('0').rstrip('.')
    if isinstance(value, (dict, list)) and len(json.dumps(value)) > 60:
        text = json.dumps(value, ensure_ascii=False)
        return text[:57] + '...'
    return json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else str(value)


def journal_from_env(directory=None, level=None) -> Journal:
    """按 Q3_LOG_DIR / Q3_LOG_LEVEL 构造；两者都没有时只打印不落盘。"""
    return Journal(directory or os.environ.get('Q3_LOG_DIR'),
                   level=level or os.environ.get('Q3_LOG_LEVEL', 'info'))
