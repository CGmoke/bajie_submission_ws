# 运行日志

本目录收集三题的完整终端 `.log`；不要将开发环境缓存、无关日志或包含私密信息的内容提交上去。

| 文件 | 内容 | 状态 |
| --- | --- | --- |
| [`q1_run.log`](q1_run.log) | 题一无界面整轮运行（5 个关键帧 + 五段净空数值），本次提交的作答由它写出 | 已生成，`exit=0` |
| [`q2_run.log`](q2_run.log) | 题二读入正式测例 `demo_zero`、求解并写出作答 | 已生成 |
| [`q2_humble_verify.log`](q2_humble_verify.log) | 官方模型展开、题二自动测试、求解及 Humble TF2 复核的完整日志 | 已生成 |
| [`q3_docker_run.log`](q3_docker_run.log) | 题三整轮运行的终端全量输出，与本次作答同源（run `20261124T043221Z-c67f31`） | 已生成，`exit=ok`、`written_rows=3` |
| [`q3_logs/`](q3_logs/) | 题三每次运行的结构化产物：`run.json`、`contact-<case>.json`、`events.jsonl` | 4 次运行，作答对应 `20261124T043221Z-c67f31` |
| [`q3_run.log`](q3_run.log) | 2026-11-21 的题三整轮日志（旧版动作策略），非当前作答来源 | 留档备查 |
| [`q3_bridge_baseline.txt`](q3_bridge_baseline.txt) | 原开发工作区的桥接诊断摘要 | 仅证明探针收到状态，不是触碰日志 |

题一的净空有两种口径——日志里的「整段保守下界」与该帧的「静态几何下界」，区别与数值见作品 README 第 4.1 节的净空表；不要拿整段保守值当作静态几何值比较。
