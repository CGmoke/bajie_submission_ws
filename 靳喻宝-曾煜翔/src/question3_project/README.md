# 题三 · Isaac Sim 玩具触碰

在官方 Isaac Sim 场景中一次性连续执行三例假夹取：在上方转腕张开，保持张开接近目标，等手腕实际到位并停稳，再低速合指（恐龙与软玩具改为腕部垂直下压、不合指）。只有同一夹指连续 6 个仿真步既被 PhysX 报告为有效目标接触，又与玩具的**可见表面**相接，才把末个验证步的实测姿态记下来。不夹起玩具；记录后竖直抬起、确认脱离该玩具，就从当前实测位姿直接规划下一件玩具，全程不回到初始位姿。官方场景和模型不改动。

`answers/q3_demo_output.jsonl` 是**连续接触判定加入之前的实测产物**：在容器 `bajie/sim:humble45` 中整轮跑出，三例均取得 PhysX 目标接触且与可见表面相接，完整终端日志见 [`assets/q3_docker_run.log`](../../assets/q3_docker_run.log)。

[`assets/q3_run.log`](../../assets/q3_run.log) 是旧版（仅查碰撞外壳）的历史记录，保留备查：那一版魔方、恐龙和软玩具的可见表面曾分别有约 0.48、2.10 和 0.33 mm 的间隙，不能作为接触证明。

## 运行日志

每次运行写到 `assets/q3_logs/<run_id>/`，与 Kit 原始输出分开：

| 文件                    | 作用                                                                                                                                                       |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `events.jsonl`        | 一行一个 JSON：UTC 时间戳、单调时钟、run_id、case、level、msg，外加 outcome / 步数 / 位移 / 间隙等字段                                                     |
| `contact-<case>.json` | 接触瞬间的十关节实测值、base/hand/toy 三方位姿、接触刚体对与间隙、连续有效帧数 `stable_contact_frames`、微调次数 `contact_corrections`、生效策略与相位 |
| `run.json`            | 三例 outcome、策略、耗时、写入行数、退出状态                                                                                                               |

`contact-*.json` 与 `answers/q3_demo_output.jsonl` 的对应关系是「同一 `id` + 同一 `run`」：答案由接触瞬间的实测值写出，快照保留当时的证据。`--log-dir` 换目录，`--log-level` 调级别，未配置目录时只打印不落盘（与旧版行为一致）。

## 目录结构

| 路径                                          | 作用                                                                                               |
| --------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| [`run_touch.py`](run_touch.py)               | 主程序：启动 Kit、打开场景、复位与约束、规划、下发指令、订阅接触与关节反馈、写作答                 |
| [`touch.py`](touch.py)                       | 纯逻辑：读取三例输入、`Planner` 增量 IK、smoothstep 采样、接触判定、作答组装、原子写文件         |
| [`surface.py`](surface.py)                   | 使用官方可见三角面独立排除夹指与玩具间的正间隙                                                     |
| [`journal.py`](journal.py)                   | 结构化日志：`events.jsonl` 事件流、`contact-<case>.json` 接触快照、`run.json` 整轮汇总       |
| [`bridge.py`](bridge.py)                     | 在内存会话层创建 ROS 2 Action Graph：时钟、关节状态发布、关节指令订阅与 `ArticulationController` |
| [`run_docker.sh`](run_docker.sh)             | Docker 启动器：资源/显存预检、只读挂载、资源限制、watchdog、GUI 与缓存挂载                         |
| [`run_q3.sh`](run_q3.sh)                     | 原生启动器：探测本机 Isaac Sim 4.5，同样的预检与 watchdog                                          |
| [`physics.kit`](physics.kit)                 | 无界面体验配置：只用 USD/时间线/PhysX/ROS 2 Bridge，关闭渲染与窗口                                 |
| [`bridge_probe.py`](bridge_probe.py)         | 旧桥接诊断：只验证 `/clock` 与 `/joint_states`，**不用于正式复现**                       |
| [`tests/test_touch.py`](tests/test_touch.py) | 输入契约、坐标系换算、smoothstep、接触配对                                                         |

## 环境与依赖

- 官方仿真资源 [https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim)。作品目录已完整包含 `src/bajie_sim`（含 `scene/`、`machine/`、`models/`），无需再下载；必须保留完整的 `models/` 相对目录，只保留入口文件不可用。
- 按[题二说明](../question2_project/README.md)准备的官方 URDF，作品目录已展开为 `src/question2_project/generated/bajie_robot.urdf`，供本题规划求解使用。
- Docker、可用的 NVIDIA 容器运行时与 `nvidia-smi`；镜像基于官方 `nvcr.io/nvidia/isaac-sim:4.5.0` 并安装 ROS 2 Humble（Python 3.10，使用 Isaac Sim 自带的 NumPy 与 SciPy）。镜像用**本目录的 [`Dockerfile`](Dockerfile)** 构建，不需要仓库外的任何文件：

  ```bash
  docker build -f src/question3_project/Dockerfile -t bajie/sim:humble45 src/question3_project
  ```

  本机镜像标签为 `bajie/sim:humble45`，可用 `Q3_IMAGE` 指定同环境的其他标签。

## 运行

两条等价入口，执行同一份 `run_touch.py`，参数与策略完全一致：

| 运行环境                        | 入口              | 前置条件                                                         |
| ------------------------------- | ----------------- | ---------------------------------------------------------------- |
| 本机 Docker（无需装 Isaac Sim） | `run_docker.sh` | Docker + NVIDIA 容器运行时 + 镜像 `bajie/sim:humble45`         |
| 官方评审原生环境                | `run_q3.sh`     | 本机 Isaac Sim 4.5，`ISAAC_PATH` 指向含 `python.sh` 的根目录 |

### 方式一：Docker（本机可直接跑）

在作品根目录 `靳喻宝-曾煜翔/`执行：

```bash
# 1) 先探测：镜像、可用内存、空闲显存、资源路径
bash src/question3_project/run_docker.sh --check

# 2) 整轮执行，作答写入 answers/
Q3_RESULT_DIR="$PWD/answers" \
  bash src/question3_project/run_docker.sh 2>&1 | tee /tmp/q3_docker_run.log
```

三个测例约 3～6 分钟；首次启动要编译 RTX 着色器（约 4 分钟），之后走 `Q3_CACHE_DIR` 缓存会快很多。

宿主机读不到 `nvidia-smi`（例如受限命名空间里没有 `/dev/nvidia*`）时，启动器会自动改用一次性容器探测显存——**宿主机读不到不等于容器用不了**，这两种情况分开判断。

### 方式二：原生 Isaac Sim

```bash
export ISAAC_PATH=<Isaac Sim 4.5 安装根目录>
out=$(mktemp -d /tmp/bajie-q3-verified.XXXXXX)
Q3_RESULT_DIR="$out" \
  bash src/question3_project/run_q3.sh --gui 2>&1 | tee "$out/run.log"
```

`--gui` 打开 Isaac Sim 原生窗口，需要可用的 `DISPLAY`；没有图形桌面时去掉 `--gui`，接触判定与作答完全一致。

作品目录内资源为 `$PWD/src/bajie_sim` 与 `$PWD/src/question2_project/generated/bajie_robot.urdf`；输出目录独立于正式提交目录，也可不设 `Q3_RESULT_DIR`，启动器会创建并打印临时输出目录。

### 启动器参数（环境变量）

| 变量               | 默认                             | 说明                                                                                                                                                                         |
| ------------------ | -------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `BAJIE_SIM_DIR`  | 必填                             | 含 `scene/abr_scene.usd` 与 `models/` 的官方资源根目录                                                                                                                   |
| `BAJIE_URDF`     | 必填                             | 展开后的官方 URDF 绝对路径                                                                                                                                                   |
| `Q3_RESULT_DIR`  | 新建临时目录（默认策略整轮执行） | 显式指定时目录必须已存在；整轮假夹取不会覆盖 `answers/`                                                                                                                    |
| `Q3_STRATEGY`    | `adaptive-fallback`            | 先跑自适应轻合；某例未取得可见表面接触时抬升脱离、回退同一题的竖直下探再试。日志逐例标注实际生效的 `strategy=`。`adaptive` 只跑自适应，`baseline` 只跑下探（历史对照） |
| `Q3_IMAGE`       | `bajie/sim:humble45`           | 容器镜像标签（仅 Docker 模式）                                                                                                                                               |
| `Q3_GUI`         | `0`                            | `1` 时打开 Isaac Sim 原生窗口；需要 `DISPLAY`，改用 host 网络与 12 GiB 内存                                                                                              |
| `Q3_MEM`         | `12g`                          | 容器内存上限与 swap 上限（仅 Docker 模式）                                                                                                                                   |
| `Q3_CACHE_DIR`   | `~/.cache/bajie-q3-kit`        | Kit 着色器/数据缓存，需持久化，否则每次重编 RTX 着色器                                                                                                                       |
| `Q3_SKIP_CHECKS` | `0`                            | `1` 时跳过内存与显存预检（不推荐，仅在已确认资源足够时使用）                                                                                                               |

单独排查一例（**不会**写正式作答）：

```bash
bash src/question3_project/run_docker.sh --case rubik_cube
# 可选：dinosaur、soft_toy；原生环境换成 run_q3.sh --gui --case rubik_cube
```

### 资源保护与自动终止

- 启动前要求宿主可用内存 ≥ **12,800 MiB**、空闲显存 ≥ **6,000 MiB**，否则直接不启动。
- 无界面容器限制 **8 GiB 内存、禁止交换、4 CPU、768 PID**，网络隔离；GUI 模式为 **12 GiB、host 网络**。
- watchdog 每 2 秒采样：可用内存 < **4,500 MiB** 或空闲显存 < **500 MiB**，连续两次触线即停止容器；运行满 **360 秒**（GUI 为 900 秒）同样终止。请勿同时启动 RViz 或其他仿真实例。
- 脚本只停止自己创建的容器（固定名 `bajie-q3-safe`），并在退出时清理。

正式实测最低可用内存 **10,037 MiB**、最低空闲显存 **6,324 MiB**（采样值）。

## 运行逻辑

1. `run_touch.py` 只打开一次官方场景，将所有 `OmniGraph` 置为 inactive，只保留本题自己创建的桥接图；把底盘按输入位姿摆到世界中并用内存中的 `FixedJoint` 锚定。三例在同一仿真会话里依次执行，不再逐例重开场景。
2. 使用 CPU PhysX、60 Hz 固定步长，按玩具轮廓规划张开的接近姿态。手腕实测关节位置和速度连续四步达标后，才允许合指。合指期间手腕目标不变。
3. `bridge.py` 通过 ROS 2 发布关节反馈和指令；指 2 是场景中的随动关节，由指 1 联动。
4. 两种策略都要求同一根夹指连续 6 个仿真步同时满足：对目标有带冲量的非正分离接触，且**夹指与玩具的可见网格相接**。首次满足时先保持该指令；连续 6 步无有效接触才沿原方向微调，最多 4 次。合指每次 0.003 rad，不超出已规划闭合范围；下探每次 0.25 mm，不改开合角。只碰到碰撞外壳不计成功；有非目标碰撞、自适应策略在到位前碰到玩具时仍中止。合指类动作的玩具原点位移保护为 2 mm；`*_press` 垂直下压是**有意轻碰**，允许玩具沿支撑方向动一下，位移上限放宽到 6 mm。只保存最后一个验证步的 10 个实测关节及相对位姿，不把玩具提起。
5. 保存接触快照后**不做归位**：保持当前场景，沿当前腕姿竖直抬起最多 8 cm，直到连续 4 帧不再接触该玩具；抬升中发现非目标碰撞、脱离后重新接触或到顶仍未脱离则中止。确认脱离后，以同一实测十关节位姿为起点规划下一件玩具，先竖直抬到悬停高度再横向对齐，因此三例动作是连续的。最后一例后不再抬升，仿真停在接触位姿。
6. 三例都在同一会话中完成触碰并逐例脱离后，才按输入顺序**原子替换** `q3_demo_output.jsonl`。任何一例未确认接触时，先抬升脱离再继续下一例；脱离失败则整轮中止，不写部分作答。`--case` 只跑该例、结束停在接触位姿，只打印单例结果，不写正式作答。

### 作答字段

- `final_joint_state`：10 个关节的实际位置。
- `objects_link_in_base_footprint`：夹爪相对底盘，`xyz` 米、`rpy` 弧度、`quat_xyzw` 顺序 `[x, y, z, w]`。
- `scene_object_in_objects_link`：玩具相对夹爪。

输入见 [`q3_demo_input.jsonl`](../../inputs/q3_demo_input.jsonl)，每例给出 `base_footprint_in_world`、`scene_object_in_world`、初始 `joint_state` 与玩具名（`rubik_cube` / `dinosaur` / `soft_toy`）。

### 按玩具选择合拢姿态

- **魔方**：合拢方向对准一条侧面法线，让夹指从侧面贴近。
- **恐龙**：优先用 `dinosaur_press`：腕部垂直下压、**不合指**，接触由下降产生。恐龙侧脸近乎垂直，贴着侧面合指会沿闭合圆弧横向扫掠，把玩具推走（实测 2.14 mm）。垂直下压的位移沿支撑方向，只推 0.82 mm。
- **软玩具**：与恐龙同样优先用 `soft_toy_press`，腕部垂直下压、**不合指**，接触由下降产生，玩具会被轻微碰到动一下；下压不可达时再回退 `soft_upper` / `surface_close`（斜贴外露边缘，让一侧指垫先碰到）。

用 `outcome=contact phase=touch visible_surface=touch` 和 `stable_contact_frames=6` 核对触碰，用 `clearance=released` 核对下一例前的脱离。整轮的验收是三组结果按输入顺序出现，且独立目录中写出三行作答。题面没有公开可据此计算最高分的细则；优先确认每例的真实触碰，不用回退结果凑齐新动作成功率。

### 当前仿真验证

连续动作改造后的整轮验证（2026-11-23，Docker GUI，[`assets/q3_logs/20261123T122937Z-53f587/run.json`](../../assets/q3_logs/20261123T122937Z-53f587/run.json)）：三例均 `outcome=contact` 且 `visible_surface=touch`，实际策略均为 `adaptive`，未触发回退；非目标接触为 0，前两例 `clearance=released`，三例接触时都弹出特写。软玩具改用 `soft_toy_press` 垂直下压，玩具被碰动 4.347 mm。整轮 144.4 s，写出 3 行作答，`exit=ok`。同版本无窗口复测（[`assets/q3_logs/20261123T061146Z-c58150/run.json`](../../assets/q3_logs/20261123T061146Z-c58150/run.json)）同为 3/3，59.8 s。

| 测例                   | 接触指       |   分离距离 | 玩具最大位移 | 脱离       |
| ---------------------- | ------------ | ---------: | -----------: | ---------- |
| 魔方 `touch_toy_1`   | 右指         | −0.339 mm |     0.188 mm | released   |
| 恐龙 `touch_toy_2`   | 左指         | −3.198 mm |     0.910 mm | released   |
| 软玩具 `touch_toy_3` | 左指（下压） | −5.997 mm |     4.347 mm | 末例不抬升 |

改造前的逐例归位验证（2026-11-23，Docker + 原生 GUI，单帧接触判定，[`assets/q3_logs/20261123T030715Z-aa8104/run.json`](../../assets/q3_logs/20261123T030715Z-aa8104/run.json)）保留备查：三例均 `outcome=contact` 且 `visible_surface=touch`，实际策略均为 `adaptive`，单次整轮 238.8 s。

## GUI 仿真（Docker + 原生窗口）

默认以无界面方式运行；需要观察三维画面时打开 Isaac Sim 原生窗口。容器环境用下面的 `Q3_GUI=1` 命令；原生 Isaac Sim 环境用 `bash src/question3_project/run_q3.sh --gui`：

```bash
out=$(mktemp -d /tmp/bajie-q3-gui.XXXXXX)
Q3_GUI=1 Q3_STRATEGY=adaptive-fallback Q3_RESULT_DIR="$out" \
  bash src/question3_project/run_docker.sh 2>&1 | tee "$out/run.log"
```

启动器会挂载 X11 与 Kit 缓存（`Q3_CACHE_DIR`），窗口分辨率 960×540，相机先对准机器人。获取作答快照后暂停物理，展示接触位置约 2.5 秒，然后切回全景并继续仿真，展示抬起脱离与直接驶向下一件玩具的动作。GUI 模式内存上限 12 GiB、运行上限 1800 秒，需要可用的 `DISPLAY`；启动前必须满足可用内存 12,800 MiB。首次启动需编译 RTX 着色器，耗时较长。

## 测试

不启动仿真，只需 Python 3.10+、NumPy 与 SciPy：

```bash
python3 -m unittest discover -s src/question3_project/tests -v
```

覆盖三例输入与作答坐标系、轨迹端点与速度限制、按形状转腕、用官方 URDF 求解三例动作、夹爪几何筛选、可见表面正间隙拒绝、先停稳再合指、同一夹指连续接触、接触丢失后的有界微调、失败回退、脱离抬升与链式续接。自适应规划测试使用合成碰撞点；测试需要 `src/question2_project/generated/bajie_robot.urdf`，独立提交目录可用 `BAJIE_URDF` 指定。真实接触和连续动作仍须用 Isaac Sim 验证。

## 常见问题

| 现象                                                        | 原因与处理                                                                                    |
| ----------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `Set BAJIE_SIM_DIR ...` / `Missing output directory...` | 环境变量未设、`Q3_RESULT_DIR` 不存在，或场景缺少 `models/`                                |
| `未启动：系统可用内存不足 12800 MiB`                      | 先关闭浏览器/IDE/其他仿真；这是启动前的硬门槛                                                 |
| `未启动：显存余量不足 6000 MiB`                           | 释放显存，确认 `nvidia-smi` 可用且 NVIDIA 容器运行时已配置                                  |
| `Missing official scene prim` / `Missing toy colliders` | 官方资源不完整或目录层级不对，重新下载完整 `models/`                                        |
| `ROS handshake failed`                                    | Bridge 未启用或 ROS 域冲突；确认镜像内含 `isaacsim.ros2.bridge`，并避免与宿主其他演示同域   |
| `Official FK does not match sim`                          | URDF 与场景模型不是同一版本，或底盘/玩具摆位错误                                              |
| `no verified fingertip contact`                           | 规划路径未真正触碰；先确认玩具位置与 `models/` 完整，再单独用 `--case` 排查               |
| `visible fingertip contact did not stabilize`             | 曾触碰但没有达到连续 6 步；检查可见表面、位移与接触日志，不将单帧或仅碰撞外壳的结果写入作答。 |
| `clearance` 失败 / 整轮中止                               | 抬升时碰到非目标、脱离后重新接触或到顶仍未脱离；程序不会跳到下一例                            |
| 容器被 watchdog 终止                                        | 内存/显存触线或超时；关闭其他程序，或临时降低并发需求后重试                                   |
| `Q3_GUI=1` 但无窗口                                       | `DISPLAY` 为空、Xauthority 不可用，或渲染缓存目录不可写                                     |

## 注意事项

- 本题依赖物理接触，**不能用题一的几何预检结果代替**；作答中的姿态是仿真实际读数。
- 不要改写官方 USD；脚本只使用会话层，若需要保存调试图请另存副本。
- GUI 模式会显著增加内存与显存占用，正式跑三例建议保持无界面模式。
- 必交演示视频预计命名 `q3_touch.mp4`，当前尚未生成；视频与作答是两项独立材料。

### 恐龙测例：为什么改成「垂直下压」

恐龙的侧脸近乎垂直。贴着侧面合指时，指尖沿闭合圆弧扫掠，结果只有两种：要么擦过表面（停在可见表面外 0.003 mm，判成正间隙假阴性），要么把玩具横向推走（实测 2.14 mm，触发 2 mm 保护）。把预压从 2 mm 减到 1 mm 也没有改善（位移仍有 2.005 mm），说明**姿态才是关键，预压量不是**。

于是加了一路 `dinosaur_press` 候选：腕部朝下、指尖正对目标点，**全程不合指**（开合角保持不变），接触完全由竖直下降产生，最后一段下降放在 `touch` 相位以便允许接触。实测位移 0.823 mm，远低于 2 mm 保护线。同一环境三种姿态对比：

| 方案                                    | 接触         | 玩具位移                         | 说明                             |
| --------------------------------------- | ------------ | -------------------------------- | -------------------------------- |
| `surface_close`（斜贴侧脸 + 合指）    | 无           | 0.75 mm                          | 闭合弧与侧脸相切，只擦过表面     |
| `dinosaur_single`（斜贴 + 2 mm 预压） | 有，但先拖动 | 2.14 mm（预压 1 mm 时 2.005 mm） | 斜向下滑拖走玩具，触发保护而中止 |
| `dinosaur_press`（垂直下压、不合指）  | 有           | 0.823 mm                         | 位移沿支撑方向，不产生横向拖拽   |

三例现在都能在**纯 `adaptive`** 下取得可见表面接触（见 `assets/q3_docker_run.log`，三行均为 `strategy=adaptive outcome=contact visible_surface=touch`）。`adaptive-fallback` 仍保留为默认策略，仅作兜底，在本环境未触发。

一个实现要点：`press` 的接近点必须高于玩具最高点（实测取「最高点上方 30 mm」）。否则接触会发生在 `approach` 相位，被 `check_touch` 判为「过早接触」而整题作废——这是这套执行器「到点前不许接触」的既有约束，不是 bug。
