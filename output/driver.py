#!/usr/bin/env python3
"""按 request 逐个重建 HTML、转 DOCX、跑质量门禁。"""
import json
import subprocess
import sys
from pathlib import Path

WS = Path("/home/moke/bajie_submission_ws/output")
PLUGIN = Path("/home/moke/.workbuddy/plugins/cache/workbuddy-builtin/tencent-docx/5.6.2-wb.40046327.gc7b525f3.hbf00e3f3d64d")
PY = str(Path.home() / ".venv-html-to-docx/bin/python")

for rid in sorted(p.name for p in WS.iterdir() if (p / "pipeline-state.yaml").exists()):
    meta = json.loads((WS / rid / "working/meta.json").read_text(encoding="utf-8"))
    html = WS / rid / "stage2" / meta["html_name"]
    subprocess.run([sys.executable, str(WS / "build_html.py"),
                    str(WS / rid / "stage1/final_draft.md"), str(html),
                    json.dumps(meta, ensure_ascii=False)], check=True)
    out = subprocess.run([PY, "-m", "html_to_docx", "convert", str(html), "-o", meta["docx"]],
                         cwd=str(PLUGIN / "skills/html-to-docx/scripts"),
                         capture_output=True, text=True)
    ok = '"success": true' in out.stdout
    print(f"[{rid[:8]}] convert={'ok' if ok else 'FAIL'}  {out.stdout.strip()[:160] or out.stderr[:160]}")
    rev = subprocess.run([sys.executable, str(PLUGIN / "skills/html-review/scripts/review_html.py"),
                          "--html", str(html), "--genre", "business-report"],
                         capture_output=True, text=True)
    try:
        d = json.loads(rev.stdout)
        print(f"          review passed={d['passed']} score={d['score']} feedback={len(d.get('actionable_feedback') or [])}")
        for f in (d.get("actionable_feedback") or [])[:5]:
            print("            -", f)
    except Exception:
        print("          review 解析失败:", rev.stdout[:120], rev.stderr[:120])
