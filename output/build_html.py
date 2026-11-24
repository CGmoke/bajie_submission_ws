#!/usr/bin/env python3
"""doc-typeset 等价实现：按 business-report 模板 + design tokens 将 Markdown 排成 HTML。

用法: build_html.py <md> <out.html> <json_meta>
json_meta: {"category","title","subtitle","meta_line","abstract"}
"""
import html
import json
import re
import sys
from pathlib import Path

PLUGIN = Path("/home/moke/.workbuddy/plugins/cache/workbuddy-builtin/tencent-docx/5.6.2-wb.40046327.gc7b525f3.hbf00e3f3d64d")
TOKENS = json.loads((PLUGIN / "skills/design-token/tokens/compiled/business-report.json").read_text(encoding="utf-8"))


def inline(text: str) -> str:
    text = html.escape(text, quote=False)
    text = re.sub(r"`([^`]+)`", r"<code>\1</code>", text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)
    return text


def is_table_row(line: str) -> bool:
    return line.strip().startswith("|") and line.strip().endswith("|")


def split_row(line: str):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def md_to_html(md: str):
    lines = md.split("\n")
    out, toc = [], []
    i = 0
    while i < len(lines):
        line = lines[i]
        s = line.strip()
        if not s:
            i += 1
            continue
        if s.startswith("# "):
            i += 1  # 文档标题由封面承担
            continue
        if s.startswith("### "):
            out.append(f"<h3>{inline(s[4:])}</h3>")
            i += 1
            continue
        if s.startswith("## "):
            title = s[3:]
            anchor = f"sec{len(toc) + 1}"
            toc.append((anchor, title))
            out.append(f'<h2 id="{anchor}">{inline(title)}</h2>')
            i += 1
            continue
        if is_table_row(s):
            block = []
            while i < len(lines) and is_table_row(lines[i].strip()):
                block.append(lines[i].strip())
                i += 1
            rows = [split_row(r) for r in block]
            header, body = rows[0], rows[2:] if len(rows) > 2 else []
            parts = ['<table class="data-table">', "<thead><tr>"]
            parts += [f"<th>{inline(c)}</th>" for c in header]
            parts.append("</tr></thead><tbody>")
            for r in body:
                parts.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>")
            parts.append("</tbody></table>")
            out.append("".join(parts))
            continue
        if s.startswith("- "):
            items = []
            while i < len(lines) and lines[i].strip().startswith("- "):
                items.append(lines[i].strip()[2:])
                i += 1
            out.append("<ul>" + "".join(f"<li>{inline(x)}</li>" for x in items) + "</ul>")
            continue
        if re.match(r"^\d+\.\s", s):
            items = []
            while i < len(lines) and re.match(r"^\d+\.\s", lines[i].strip()):
                items.append(re.sub(r"^\d+\.\s", "", lines[i].strip()))
                i += 1
            out.append("<ol>" + "".join(f"<li>{inline(x)}</li>" for x in items) + "</ol>")
            continue
        # 段落：连续非空、非特殊行
        buf = []
        while i < len(lines):
            t = lines[i].strip()
            if (not t) or t.startswith("#") or is_table_row(t) or t.startswith("- ") or re.match(r"^\d+\.\s", t):
                break
            buf.append(t)
            i += 1
        out.append(f"<p>{inline(' '.join(buf))}</p>")
    return "\n".join(out), toc


def build(md_path: Path, out_path: Path, meta: dict) -> None:
    md = md_path.read_text(encoding="utf-8")
    main_html, toc = md_to_html(md)
    root_vars = "\n".join(f"    {k}: {v};" for k, v in TOKENS["css_variables"].items())
    toc_html = "".join(f'<li><a href="#{a}">{html.escape(t)}</a></li>' for a, t in toc)
    summary_table = meta.get("summary_table", [])
    kv = ""
    if summary_table:
        kv = ('<table class="data-table kv"><thead><tr><th>项目</th><th>内容</th></tr></thead><tbody>') + "".join(
            f"<tr><td>{inline(k)}</td><td>{inline(v)}</td></tr>" for k, v in summary_table
        ) + "</tbody></table>"

    doc = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="docx-page-size" content="A4">
  <title>{html.escape(meta['title'])}</title>
  <style>
  @page {{
    @bottom-center {{ content: counter(page); }}
  }}
  @page cover {{
    @bottom-center {{ content: none; }}
  }}
  section[role="cover"] {{ page: cover; }}

  :root {{
{root_vars}
    --fs-h1: var(--typography-fontSize-h1, 20pt);
    --fs-h2: var(--typography-fontSize-h2, 16pt);
    --fs-h3: var(--typography-fontSize-h3, 13pt);
    --fs-body: var(--typography-fontSize-body, 11pt);
    --fs-small: var(--typography-fontSize-small, 9pt);
    --ff-heading: var(--typography-fontFamily-heading, "Microsoft YaHei");
    --ff-body: var(--typography-fontFamily-body, "Microsoft YaHei");
    --lh-body: var(--typography-lineHeight-body, 1.7);
    --color-primary: var(--color-primary, #1a4f8b);
    --color-text: var(--color-text, #1a1a1a);
    --color-muted: var(--color-muted, #5b6470);
    --color-border: var(--color-border, #c9d2dd);
    --color-highlight: var(--color-highlight, #f3f6fa);
    --spacing-paragraph: 0.6em;
    --spacing-section: 1.6em;
    --spacing-block: 0.9em;
    --margin-page: 2.4cm;
    --page-content-width: 15.6cm;
    --ff-code: Consolas, "Courier New", monospace;
    --fs-code: 0.94em;
    --sp-list-margin: 0 0 var(--spacing-block) 1.4em;
    --sp-list-padding: 1.1em;
    --sp-li-bottom: 0.35em;
    --sp-code-padding: 0 2px;
    --sp-eyebrow-bottom: 0.8em;
    --sp-title-bottom: 0.4em;
    --sp-subtitle-bottom: 1.2em;
    --sp-toc-li-bottom: 0.25em;
    --sp-summary-padding: 0.9em;
    --sp-cell-padding: 0.4em 0.55em;
  }}

  * {{ box-sizing: border-box; }}
  body {{
    font-family: var(--ff-body); font-size: var(--fs-body); line-height: var(--lh-body);
    color: var(--color-text); margin: 0 auto; max-width: var(--page-content-width);
  }}
  h2 {{ font-family: var(--ff-heading); font-size: var(--fs-h2); font-weight: 700;
        color: var(--color-primary); margin: var(--spacing-section) 0 var(--spacing-paragraph) 0; }}
  h3 {{ font-family: var(--ff-heading); font-size: var(--fs-h3); font-weight: 700;
        margin: var(--spacing-block) 0 var(--spacing-paragraph) 0; }}
  p {{ margin: 0 0 var(--spacing-paragraph) 0; text-align: justify; }}
  ul, ol {{ margin: var(--sp-list-margin); padding-left: var(--sp-list-padding); }}
  li {{ margin-bottom: var(--sp-li-bottom); }}
  code {{ font-family: var(--ff-code); font-size: var(--fs-code);
          background: var(--color-highlight); padding: var(--sp-code-padding); }}

  .report-cover {{ padding: var(--margin-page) 0 var(--spacing-section) 0; border-bottom: 2px solid var(--color-primary); }}
  .cover-category {{ font-family: var(--ff-heading); font-size: var(--fs-small); font-weight: 700;
                     color: var(--color-primary); letter-spacing: 0.08em; margin-bottom: var(--sp-eyebrow-bottom); }}
  .cover-title {{ font-family: var(--ff-heading); font-size: var(--fs-h1); font-weight: 700;
                  color: var(--color-primary); margin: 0 0 var(--sp-title-bottom) 0; line-height: 1.35; }}
  .cover-subtitle {{ font-size: var(--fs-h3); color: var(--color-muted); margin-bottom: var(--sp-subtitle-bottom); }}
  .cover-meta {{ font-size: var(--fs-small); color: var(--color-muted); }}

  .doc-toc {{ padding: var(--spacing-block) 0; border-bottom: 1px solid var(--color-border); margin-bottom: var(--spacing-section); }}
  .toc-title {{ font-family: var(--ff-heading); font-weight: 700; margin-bottom: var(--sp-title-bottom); }}
  .toc-list {{ list-style: none; margin: 0; padding-left: 0; }}
  .toc-list li {{ margin-bottom: var(--sp-toc-li-bottom); }}
  .toc-list a {{ color: var(--color-primary); text-decoration: none; }}

  .executive-summary {{ background: var(--color-highlight); padding: var(--spacing-block) var(--sp-summary-padding);
                        border-left: 4px solid var(--color-primary); margin-bottom: var(--spacing-section); }}
  .executive-summary h2 {{ margin-top: 0; }}

  table.data-table {{ width: 100%; border-collapse: collapse; margin: 0 0 var(--spacing-block) 0; font-size: var(--fs-small); }}
  table.data-table th, table.data-table td {{ border: 1px solid var(--color-border); padding: var(--sp-cell-padding); text-align: left; vertical-align: top; }}
  table.data-table th {{ background: var(--color-highlight); font-family: var(--ff-heading); font-weight: 700; }}
  table.kv td:first-child {{ width: 30%; color: var(--color-muted); }}
  .appendix {{ border-top: 1px solid var(--color-border); margin-top: var(--spacing-section); padding-top: var(--spacing-block); }}
  .appendix p {{ font-size: var(--fs-small); color: var(--color-muted); }}
  </style>
</head>
<body>
  <section role="cover" class="report-cover">
    <p class="cover-category">{html.escape(meta['category'])}</p>
    <h1 class="cover-title">{html.escape(meta['title'])}</h1>
    <p class="cover-subtitle">{html.escape(meta['subtitle'])}</p>
    <div class="cover-meta"><p>{html.escape(meta['meta_line'])}</p></div>
  </section>

  <section role="body" data-page-restart="1">
    <nav class="doc-toc">
      <p class="toc-title">目录</p>
      <ol class="toc-list">{toc_html}</ol>
    </nav>

    <div class="executive-summary">
      <h2>执行摘要</h2>
      <p>{inline(meta['abstract'])}</p>
      {kv}
    </div>

    <main>
{main_html}
    </main>

    <div class="appendix">
      <p>本文档为「八界杯创新挑战赛」初赛作品说明之一，与另外两份题目说明配套使用；文中数值、哈希与日志均可在作品目录内复核。</p>
    </div>
  </section>
</body>
</html>
"""
    out_path.write_text(doc, encoding="utf-8")


if __name__ == "__main__":
    build(Path(sys.argv[1]), Path(sys.argv[2]), json.loads(sys.argv[3]))
    print("wrote", sys.argv[2])
