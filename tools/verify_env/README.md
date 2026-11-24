# 八界机器人：关节展示、运动学与仿真触碰

参赛者 / 团队：靳喻宝、曾煜翔

本目录是初赛作品的**完整提交单元**：三题源码、官方 URDF、官方仿真资源、题三的容器 Dockerfile、输入、作答和运行脚本都在这里，不依赖任何外部工作区。

## 从零复刻（拿到仓库后照做）

### 第 0 步：确认前提

| 项目           | 要求                                                                                                                            | 说明                                                                 |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| 系统           | Ubuntu 22.04（24.04 也能跑题一、题二）                                                                                          | 题三必须 Humble：Isaac Sim 4.5 的 Python 是 3.10                     |
| 磁盘           | 克隆约 1.2 GB；跑题三另留 ≥ 20 GB                                                                                              | 官方模型 1.1 GB；题三镜像解压后约 22.6 GB                            |
| 题一 / 题二    | ROS 2 Humble 或 Jazzy + `colcon` + NumPy                                                                                      | 题二求解只需 Python + NumPy，加 `--verify` 才用 ROS                |
| 题三（二选一） | ① Docker + NVIDIA 容器运行时 + 镜像 `bajie/sim:humble45`② 原生 Isaac Sim 4.5（含 `python.sh` 与 `exts/`）+ ROS 2 Humble | 镜像基于 `nvcr.io/nvidia/isaac-sim:4.5.0`，首次构建需 NGC 账号登录 |
| 题三硬件       | 可用内存 ≥ 12,800 MiB、空闲显存 ≥ 6,000 MiB；驱动 535~580                                                                     | 启动器会预检，不满足直接拒绝启动；595.x 会在 RTX 初始化时段错误      |

### 第 1 步：拉取

```bash
# 仓库地址以飞书作品提交表里登记的 AtomGit 主页为准
git clone <作品仓库地址>
cd <仓库名>/靳喻宝-曾煜翔
```

如果你拿到的是 `靳喻宝-曾煜翔/` 压缩包，解压后直接 `cd 靳喻宝-曾煜翔` 即可——所有脚本都按**脚本自身所在目录**推算路径，不写死绝对路径，放在任何位置都能跑。

克隆后工作树约 1.2 GB（官方模型占 1.1 GB）；`generated/bajie_robot.urdf` 已随仓库提交，题二、题三不需要再手动展开。

### 第 2 步：环境自检

```bash
bash check_env.sh
```

逐项打印 `[可用]` / `[缺失]`，缺失项后面直接给出安装命令。重点看三类：ROS 2 是否 Humble、`numpy`/`scipy` 是否就位、题三的 Isaac Sim 或镜像是否可用。自检末尾还有官方资源的 SHA-256 指纹核对。

退出码约定：全部可用为 `0`，存在缺失项为 `1`，可直接接 CI；`run_all.sh` 仍会继续跑后面的题，缺件不影响其他题执行。

### 第 3 步：跑三题

```bash
bash run_all.sh          # 依次跑三题；某题失败不影响后面的题
```

或者逐题来：

```bash
bash run_q1.sh                           # 题一：关节分步驱动（无界面）
bash run_q2.sh --verify                  # 题二：正向运动学 + TF2 独立复核
bash run_tests.sh                        # 三题单元测试
```

题三的环境二选一：

```bash
# A. 容器（推荐，不用装 Isaac Sim）：镜像只需构建一次
#    首次拉取基础镜像需 NGC 账号：docker login nvcr.io
#    用户名填 $oauthtoken，密码填 NGC API key
docker build -f src/question3_project/Dockerfile -t bajie/sim:humble45 src/question3_project
bash src/question3_project/run_docker.sh --check            # 镜像/内存/显存/资源路径
Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_docker.sh

# B. 原生 Isaac Sim 4.5（Ubuntu 22.04 + ROS 2 Humble）
export ISAAC_PATH=$HOME/.local/share/ov/pkg/isaac_sim-4.5.0   # 该目录须含 python.sh 与 exts/
export ACCEPT_EULA=Y PRIVACY_CONSENT=Y                        # 首次无头启动建议设置
bash src/question3_project/run_q3.sh --check                  # 探测 Isaac Sim / ROS 2 / 资源
bash src/question3_project/run_q3.sh --case rubik_cube        # 先跑单例，约 2~3 分钟
Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_q3.sh   # 整轮三例
```

原生路径要求系统已安装 **ROS 2 Humble**（启动脚本会 source `/opt/ros/humble/setup.bash`），只装 Isaac Sim 而不装 ROS 2 会直接报错；Isaac Sim 必须是完整安装（含 `exts/`、`extscache/` 等目录），pip 方式安装不满足。

`run_all.sh` 会自动判断：有原生 Isaac Sim 就走原生，否则回退容器；两者都没有才跳过题三。

### 第 4 步：核对是否复刻成功

| 题目     | 成功判据                                                                                                                                  | 实测耗时                                   |
| -------- | ----------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| 题一     | `answers/q1_demo_output.jsonl` 1 行、5 个关键帧（`lift→head→arm→gripper_open→gripper_close`），末尾 `[q1] 完成`                 | 约 30 s                                    |
| 题二     | `answers/q2_demo_output.jsonl` 1 行；加 `--verify` 时三个测例误差均 < 1e-15（阈值 1e-8 m / 1e-7 rad）                                 | < 5 s                                      |
| 三题单测 | `run_tests.sh` 汇总三行全为「通过」，共 58 项（5 + 8 + 45）                                                                             | 约 5 s                                     |
| 题三     | 日志三例均 `outcome=contact` 且 `visible_surface=touch`，末尾 `[container] exit=0 oom=false`，`answers/q3_demo_output.jsonl` 3 行 | 约 6 min（冷启动首次编译着色器再加 4 min） |

题一与题二的作答可**逐字节复现**（各题章节给出了 SHA-256 与实测数值）。题三依赖物理接触，判定以日志里的 `outcome=contact` / `visible_surface=touch` 为准，数值不承诺逐字节一致——这是物理仿真的性质，不是实现缺陷。

本节四步已在从 `HEAD` 导出的**干净副本**中复测（2026-11-23，无 `build/`、`install/`，等同刚 clone 的状态）：`check_env.sh` 31 项全可用、退出码 `0`；题一从零编译 1.77 s 并写出 5 个关键帧，作答 SHA-256 与仓库样本一致；题二 3 个测例复核误差均 < 1e-15；58 项单测全绿；题三容器单例 `rubik_cube` 取得 `outcome=contact`、`visible_surface=touch`（`exit=0`，着色器缓存已热时 22 s）。

题三整轮三例与原生 Isaac Sim 路径未在这次复测中实跑，判据仍以上表为准。

### 常见卡点

| 现象                              | 处理                                                                                                                    |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `未找到 ROS 2`                  | `sudo apt install ros-humble-desktop python3-colcon-common-extensions`                                                |
| 原生题三 `未找到 Isaac Sim 4.5` | `export ISAAC_PATH=<安装根目录>`，该目录须含 `python.sh` 与 `exts/`；pip 安装不满足                               |
| 原生题三 `未找到 ROS 2`         | 题三必须装 Humble；Isaac Sim 4.5 的 Python 3.10 不兼容 Jazzy 的 3.12                                                    |
| 原生题三启动即退出                | 先 `export ACCEPT_EULA=Y PRIVACY_CONSENT=Y` 再运行                                                                    |
| 构建镜像报 `unauthorized`       | `docker login nvcr.io`（用户名 `$oauthtoken`，密码 NGC API key）                                                    |
| 题三 `可用内存 ... < 12800`     | 关掉占内存的程序重试；确认无碍可设 `Q3_SKIP_CHECKS=1`                                                                 |
| 题三 `未启动：显存余量不足`     | 关掉占显存的程序；容器模式会自动改用容器内 `nvidia-smi` 探测                                                          |
| 题三 `镜像不存在`               | 按第 3 步 A 构建镜像，或用 `Q3_IMAGE=<标签>` 指定已有镜像                                                             |
| 题三跑完但没有作答                | 三例都通过才写文件；看 `assets/q3_docker_run.log` 里哪一例 `outcome=miss`，再按题三说明的「按玩具选择合拢姿态」排查 |

各题更细的故障速查见 `src/question1_project/README.md`、`src/question2_project/README.md`、`src/question3_project/README.md`。

## 交付前自检

```bash
python3 check_delivery.py                                   # 只读：结构、限位、四元数、视频体积、资源指纹
python3 check_delivery.py --manifest assets/manifest.json   # 另生成哈希清单（唯一写操作）
```

`check_env.sh` 查「机器能不能跑」，`check_delivery.py` 查「交出去的东西对不对」：作答 `id` 与输入是否一一对应、五个关键帧的顺序、十个关节是否齐全且在 URDF 限位内、四元数是否归一、视频是否超过 30,000,000 字节、官方资源指纹是否与记录一致。有 FAIL 时退出码为 1，可直接接 CI。

`assets/manifest.json` 记录交付文件的字节数与 SHA-256，把 `answers/` 与 `assets/`（含题三每次运行的接触快照）逐项绑定，评审可以据此确认答案对应的就是这份日志。

当前状态：**10 项 OK、0 项 WARN、0 项 FAIL**——`video/` 下题一与题三的演示视频已补齐（见「交付状态」表）；题三作答与证据出自同一次运行 `20261124T043221Z-c67f31`，逐例对应关系见 `assets/q3_evidence_index.json`，运行产物见 `assets/q3_logs/<run_id>/`（内含 `run.json` 与 `contact-*.json`）。

## 30 秒上手

已经配好环境的机器上，最短路径就是三行：

```bash
cd 靳喻宝-曾煜翔
bash check_env.sh      # 自检：缺什么会直接告诉你装什么
bash run_all.sh        # 依次跑题一、题二、题三
```

只想单跑某一题：

```bash
bash run_q1.sh              # 题一（无界面；加 --gui 用 RViz 看三维）
bash run_q2.sh --verify     # 题二，附带 TF2 独立复核
bash run_q3.sh              # 题三·原生（需要本机 Isaac Sim 4.5）
bash src/question3_project/run_docker.sh   # 题三·容器（只需 Docker + GPU，不用装 Isaac Sim）
bash run_tests.sh           # 三题单元测试，共 58 项
```

所有脚本都用**自身所在目录**推算路径，任意克隆位置都能跑，不需要 `export` 任何变量。

## 环境要求

| 用途           | 需要                                                                        | 说明                                                                                                                                                               |
| -------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 题一           | Ubuntu 22.04/24.04 + ROS 2 Humble 或 Jazzy + `colcon`                     | `sudo apt install ros-humble-desktop python3-colcon-common-extensions`；Jazzy 同样可用                                                                           |
| 题二           | Python 3.10+ 与 NumPy                                                       | `sudo apt install python3-numpy`；求解本身不需要 ROS                                                                                                             |
| 题二复核、题三 | 题二复核可用 Humble/Jazzy；题三须 Humble                                    | 题三必须 Humble（Isaac Sim 4.5 的 Python 是 3.10）                                                                                                                 |
| 题三           | Python `scipy`                                                            | `sudo apt install python3-scipy`。Isaac Sim 自带 Python 已含 scipy 1.10.1，系统 Python 需单独装，否则 `run_tests.sh` 的题三用例报 `ModuleNotFoundError`      |
| 题三           | **二选一**：原生 Isaac Sim 4.5，或 Docker 镜像 `bajie/sim:humble45` | 镜像由本目录的 `src/question3_project/Dockerfile` 构建（见「从零复刻」第 3 步），无需外部文件；Docker 需 NVIDIA 容器运行时（`docker run --gpus all` 能出显卡） |
| 题三资源门槛   | 可用内存 ≥ 12,800 MiB，空闲显存 ≥ 6,000 MiB                               | 启动器会预检，不满足直接不启动（容器模式会自动改用容器内 `nvidia-smi` 探测）                                                                                     |

- 题三有**两条等价入口**，跑同一份 `run_touch.py`：容器用 `src/question3_project/run_docker.sh`（本机可直接跑，不用装 Isaac Sim），原生用 `src/question3_project/run_q3.sh`。官方评审环境按原生路径复现即可。
- 题一、题二在 Jazzy（Ubuntu 24.04）上也验证可用；题三必须 Humble。

## 目录结构

```text
靳喻宝-曾煜翔/
├── check_env.sh                 # 环境自检（机器能不能跑）
├── check_delivery.py            # 交付物自检（交出去的东西对不对）
├── run_all.sh                   # 一键复现三题
├── run_q1.sh  run_q2.sh  run_q3.sh
├── run_tests.sh                 # 三题单元测试
├── src/
│   ├── bajie_robot/             # 官方 URDF 包（urdf/ meshes/ calib/ launch/ rviz/）
│   ├── bajie_sim/               # 官方仿真资源（scene/ machine/ models/ scripts/ doc/），约 1.1 GB
│   ├── question1_project/       # 题一：bajie_tasks（q1_drive / q1_safety）+ launch
│   ├── question2_project/       # 题二：fk.py / solve_q2.py / verify_tf.py
│   └── question3_project/       # 题三：run_q3.sh / run_docker.sh / Dockerfile / run_touch.py / touch.py
├── generated/bajie_robot.urdf   # xacro 展开后的官方 URDF（题二、题三使用）
├── inputs/                      # 三题正式输入（q1/q2/q3_demo_input.jsonl）
├── answers/                     # 三题作答
├── assets/                      # 运行日志 + q3_logs/<run_id>/ 结构化日志 + manifest.json
├── video/                       # 演示视频
└── docs/提交自查.md             # 提交前核对清单
```

`src/bajie_sim/models/` 必须与 `scene/` 一起提交：USD 内部用相对路径引用 `../models/...`，只给入口 USD 打不开场景。

> 本目录的 `src/bajie_robot`、`src/bajie_sim` 与官方仓库逐字节一致（上游提交 `907b061` / `2fcd186`，完整版本见开发仓库的 `official.lock`）。其中 `bajie_sim` 的 `package.xml` 与 `CMakeLists.txt` 是为让 colcon 识别资源目录而加的 shim，官方资源文件本身一个字节都没改。

## 三题

**执行目录**：以下三题命令一律在作品目录 `靳喻宝-曾煜翔/`（本 README 所在目录）内执行；输入固定读 `inputs/q*_demo_input.jsonl`，作答固定写 `answers/q*_demo_output.jsonl`。

| 题目                 | 源码                                                | 输入                                                 | 作答                                                    |
| -------------------- | --------------------------------------------------- | ---------------------------------------------------- | ------------------------------------------------------- |
| 题一 · 关节分步驱动 | [`src/question1_project/`](src/question1_project/) | [`q1_demo_input.jsonl`](inputs/q1_demo_input.jsonl) | [`q1_demo_output.jsonl`](answers/q1_demo_output.jsonl) |
| 题二 · 正向运动学   | [`src/question2_project/`](src/question2_project/) | [`q2_demo_input.jsonl`](inputs/q2_demo_input.jsonl) | [`q2_demo_output.jsonl`](answers/q2_demo_output.jsonl) |
| 题三 · 仿真触碰     | [`src/question3_project/`](src/question3_project/) | [`q3_demo_input.jsonl`](inputs/q3_demo_input.jsonl) | [`q3_demo_output.jsonl`](answers/q3_demo_output.jsonl) |

### 题一 · 关节分步驱动

按升降 → 头部 → 机械臂 → 夹爪张开 → 夹爪闭合的顺序驱动 10 个关节，每阶段给出关键帧与最终状态。

```bash
bash run_q1.sh        # 无界面，只跑驱动节点（评审复跑用这条）
bash run_q1.sh --gui  # 同时启动 RViz 三维展示，录演示视频时用
```

脚本自动 `colcon build`（`bajie_robot` + `bajie_tasks`）并 source `install/setup.bash`。RViz 需要可用的图形桌面；没有图形栈时用默认的无界面模式，作答一模一样。

**轨迹净空（记录口径）：**程序用官方 URDF 的可视与碰撞几何构造包围盒，正向计算各连杆位姿，逐段量出「离 `base_footprint` 下 z=0 地面」和「离非相连机身」的下界，写入日志与作答日志。**这两个值只记录、不否决播放**——原因是官方 seed=7 示例位姿本身就低于地面 3 mm、机身包围盒重叠 3 mm（实测），若据此阻断，作答就无法与官方公布的关键帧一致。这是几何包围盒估计，不是物理接触仿真；包围盒重叠不等于网格相交。

净空可复现的关键帧：作答与官方 README §2.2 公布的 `q1_all_seed7` 示例逐值一致（十关节逐值比对，误差 0.0e+00）。抽样口径为 `random.Random(seed)` 依次对九个区间均匀抽样、保留 6 位小数，区间表见 `src/question1_project/bajie_tasks/q1_drive.py` 的 `SEED_RANGES`。

实测（无界面，1 例 5 关键帧）：地面下界 0.514 / 0.759 / −0.090 / −0.070 / −0.070 m，机身下界 0.014 / 0.044 / 0.001 / 0.006 / 0.006 m；第三段起低于地面，对应官方示例的姿态。作答可逐字节复现（SHA-256 `6e7db6ba…dc2081`）。

### 题二 · 正向运动学与坐标变换

从官方 URDF 提取 `base_footprint → objects_link` 关节链，按 `origin/axis/limit` 算刚体变换，输出夹爪在底盘系、物体在夹爪系的位姿。只需要 Python + NumPy。

```bash
bash run_q2.sh            # 求解并写作答
bash run_q2.sh --verify   # 另用 robot_state_publisher + TF2 独立复核
```

复核用独立进程（`frame_prefix=q2_verify/`）生成参考值再比对，阈值 `1e-8 m` / `1e-7 rad`，三组用例实测误差均在 `1e-15` 量级。作答可逐字节复现（SHA-256 `98fd3d80…74cc37`）。原理与"只升降 0.1 m"的手算练习见[题二说明](src/question2_project/README.md)。

### 题三 · Isaac Sim 玩具触碰

在官方场景中逐例执行假夹取：上方转腕张开、保持张开接近、手腕停稳后低速合指（恐龙与软玩具改为腕部垂直下压、不合指，让玩具轻微动一下）。同一夹指需连续 6 个仿真步同时满足 PhysX 有效目标接触与玩具**可见表面**相接才记录；不夹起玩具，记录后竖直抬起脱离该玩具，随即从当前实测位姿直接规划并触碰下一件玩具，中途不回到初始位姿，三例都通过才写作答。

```bash
bash run_q3.sh                                    # 原生 Isaac Sim：三例整轮
bash src/question3_project/run_docker.sh          # Docker：三例整轮，无窗口
bash run_q3.sh --check                            # 只探测 Isaac Sim / ROS / 资源 / 显存
bash src/question3_project/run_docker.sh --check  # 只探测镜像 / GPU / 资源
bash run_q3.sh --case rubik_cube                  # 单例排查，不写正式作答
bash run_q3.sh --gui                              # 原生 Isaac Sim 窗口

# Docker + 原生 GUI 窗口（需要可用的 DISPLAY）
out=$(mktemp -d /tmp/bajie-q3-gui.XXXXXX)
Q3_GUI=1 Q3_STRATEGY=adaptive-fallback Q3_RESULT_DIR="$out" \
  bash src/question3_project/run_docker.sh 2>&1 | tee "$out/run.log"
```

启动器做三件事：自动定位 Isaac Sim（`ISAAC_PATH` 或常见安装路径，容器模式改为解析镜像与容器内 GPU）、source ROS 2 Humble、预检内存与显存；运行中定期采样，触线即终止（单例 600 s、整轮 1800 s、GUI 1800 s，可用 `Q3_TIMEOUT` 覆盖）。默认策略 `adaptive-fallback`：自适应轻合优先，某例未取得可见表面接触时抬升脱离该玩具、回退竖直下探再试，日志逐例标注实际生效的 `strategy=`。细节与故障速查见[题三说明](src/question3_project/README.md)。

**连续动作实测结果（2026-11-23，Docker GUI）。** 镜像 `bajie/sim:humble45`，三例在同一仿真会话里背靠背执行，每例接触后竖直抬起脱离再直接驶向下一件玩具；第三件蜘蛛侠与恐龙同样采用腕部垂直下压、不合指，玩具被碰动 4.347 mm，三例接触时都弹出特写。整轮 144.4 s；三例实际生效策略均为 `adaptive`，没有触发回退：

| 测例                      | 接触指       | 接触判据                  |   分离距离 | 玩具最大位移 | 脱离       |
| ------------------------- | ------------ | ------------------------- | ---------: | -----------: | ---------- |
| `touch_toy_1`（魔方）   | 右指         | `visible_surface=touch` | −0.339 mm |     0.188 mm | released   |
| `touch_toy_2`（恐龙）   | 左指         | `visible_surface=touch` | −3.198 mm |     0.910 mm | released   |
| `touch_toy_3`（蜘蛛侠） | 左指（下压） | `visible_surface=touch` | −5.997 mm |     4.347 mm | 末例不抬升 |

分离距离为负表示 PhysX 已报告非正间隙的真实接触。本次提交的作答由 `20261124T043221Z-c67f31` 这一次整轮运行生成：`exit=ok`、`written_rows=3`、`misses=[]`、非目标接触步数均为 0，该 run 目录内的 `run.json` 与 3 份 `contact-*.json` 即为作答的同源证据，逐例对应关系见 [`assets/q3_evidence_index.json`](assets/q3_evidence_index.json)。同版本 GUI 连续动作复测 3/3 见 [`assets/q3_logs/20261123T122937Z-53f587/`](assets/q3_logs/20261123T122937Z-53f587/)，无窗口复测见 [`assets/q3_logs/20261123T061146Z-c58150/`](assets/q3_logs/20261123T061146Z-c58150/)，改造前的逐例归位验证（Docker + 原生 GUI，238.8 s）保留在 [`assets/q3_logs/20261123T030715Z-aa8104/`](assets/q3_logs/20261123T030715Z-aa8104/) 备查。

**运行日志（题三）。** 每次运行在 `assets/q3_logs/<run_id>/` 下留三类产物，与 Kit 原始输出分开：

| 文件                    | 内容                                                                                                                  |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `events.jsonl`        | 本次运行的事件流，一行一个 JSON：UTC 时间戳、单调时钟、run_id、case、outcome、步数、玩具位移、最近正间隙，通常几十 KB |
| `contact-<case>.json` | 接触瞬间的十关节实测值、base / hand / toy 三方位姿、接触刚体对与间隙、生效策略与相位                                  |
| `run.json`            | 整轮汇总：三例 outcome、策略、耗时、写入行数、退出状态                                                                |

Kit 自身的原始输出仍完整保留在启动器 tee 的日志里（含背景材质缺失等提示），不做任何删减——那是证据。`events.jsonl` 只是另建一条可 `grep` / `jq` 的通道，让「答案的哪一行对应哪一帧」不必在几百行 Kit 输出里人眼找。`--log-dir` 可指定目录，`--log-level` 控制级别（默认 `info`）。

**作答与证据的对应关系**记在 [`assets/q3_evidence_index.json`](assets/q3_evidence_index.json)：逐例列出「关节值逐值一致的快照」「关节值最接近的快照」与「末端位姿最接近的快照」，并给出实测偏差。当前三例作答都能在 `20261124T043221Z-c67f31` 的同一次运行里找到**关节值逐值一致**的接触快照（位姿偏差 < 1e-6 m），即作答与 `run.json`、`contact-*.json` 同源，评审可逐行回溯。`q3_logs/` 只保留本次作答对应的运行与三次有文档引用的复测；其余探索记录已移出提交目录。若某次复测的关节值与作答不同，属同一末端姿态下的多组 IK 解，不影响触碰结论。

## 交付状态

| 项目                   | 状态                                                                                                                                                                                                                                                                                                            |
| ---------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 题一源码 / 作答 / 日志 | 完成，作答与官方 `q1_all_seed7` 示例逐值一致，可复现                                                                                                                                                                                                                                                          |
| 题二源码 / 作答 / 日志 | 完成，8 项测试 + TF2 三组复核通过                                                                                                                                                                                                                                                                               |
| 题三源码 / 作答 / 日志 | 完成；作答由容器 `bajie/sim:humble45` 整轮运行 `20261124T043221Z-c67f31` 生成（77.8 s，三例 `outcome=contact`、`visible_surface=touch`），该 run 目录内 `run.json` 与 3 份 `contact-*.json` 齐全，与作答逐值一致；终端全量日志见 `assets/q3_docker_run.log`，对应关系见 `assets/q3_evidence_index.json`；另有 GUI 连续动作复测（`20261123T122937Z-53f587/`）备查 |
| 三题单元测试           | 58 项全通过（5 + 8 + 45），本机实测                                                                                                                                                                                                                                                                             |
| 题一 RViz 视频         | 完成 `video/q1_rviz.mp4`：RViz 内完整展示五个阶段，32.6 s，976×720 / 25 fps，H.264 + AAC，888,772 字节（0.89 MB） |
| 题三 Isaac Sim 视频    | 完成 `video/q3_touch.mp4`：Isaac Sim 内魔方、恐龙、软玩具三例触碰实录，3 min 35 s，832×544 / 25 fps，H.264 + AAC，24,364,079 字节（24.4 MB） |
| 报名信息 / 功能亮点    | 完成：参赛者为靳喻宝、曾煜翔，功能亮点见文末「功能亮点」 |

`assets/` 收完整运行日志，`video/` 两个必交视频单个不超过 30,000,000 字节。提交前请按[自查清单](docs/提交自查.md)逐项确认。

## 已知限制

- 题三有**两条等价入口**：容器（`run_docker.sh`，本机即可复跑，本目录的作答就出自这条路径）与原生 Isaac Sim（`run_q3.sh`，官方评审环境）。两者执行同一份 `run_touch.py`，参数与策略一致。
- 恐龙一例原本会在自适应路径上临界失败（指尖贴着侧脸擦过，停在可见表面外 0.003 mm）。现已加入 `dinosaur_press` 候选：腕部垂直下压、**全程不合指**，接触由下降产生，位移 0.823 mm（合指类动作的保护阈值 2 mm；下压轻碰放宽到 6 mm）。软玩具也改用同一手法。三例都能在纯 `adaptive` 下取得可见表面接触；`adaptive-fallback` 保留为默认策略仅作兜底，本环境未触发，日志逐例标注实际生效的 `strategy=`。
- 题一净空检查由「阻断播放」改为「记录数值」。官方 seed=7 示例位姿实测地面 −0.003 m、机身包围盒重叠 −0.003 m，按原 5 cm / 2 cm 阈值会拒绝它，那样作答就无法与官方公布的关键帧一致。包围盒是保守估计，重叠不等于网格相交，因此这两个数只写进日志，不再否决播放。

## 官方资源与指纹

- URDF：[https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf)，入口 `urdf/bajie_robot.urdf.xacro`，SHA-256 `6dc26b06…0d8f49`
- 仿真资源：[https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim](https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim)，入口 `scene/abr_scene.usd`，SHA-256 `42d98824…628735`

两处资源均随作品放在本目录，`bash check_env.sh` 与 `python3 check_delivery.py` 都会校验这两个指纹，确认评审拿到的模型与文档记录一致。官方模型未作任何几何或 `origin` 改动。

`src/bajie_robot/meshes/` 下的 `arm_base_link.STL.bk`、`berxel_arm_base_link.STL.bk`、`berxel_upper_base_link.STL.bk`、`tp55_base_Link.STL.unuse` 是**官方仓库自带的备份/弃用文件**（见上游 `ecovacs-bajie-urdf@907b061` 的 `meshes/` 目录），为保持逐字节一致而原样保留，不是本项目遗留的临时文件。

## 功能亮点

**题一 · 关节分步驱动**
- 抽样口径完全固定：`random.Random(seed)` 对九个关节区间依次均匀抽样、保留 6 位小数，作答与官方 `q1_all_seed7` 示例**逐值一致**（误差 0.0e+00），换机器重跑仍逐字节相同。
- 用官方 URDF 的可视与碰撞几何构造包围盒，正向算出各连杆位姿，逐段量出「离地」与「离非相连机身」的净空下界。实测官方示例本身就低于地面 3 mm，因此净空**只记录、不否决**播放——既不篡改官方位姿，又留下可复核的量化证据。

**题二 · 正向运动学与坐标变换**
- 不依赖 TF 求解：从 URDF 直接提取 `base_footprint → objects_link` 关节链，按每个关节的 `origin` / `axis` / `limit` 自行实现刚体变换，只用 Python + NumPy。
- 再用 `robot_state_publisher + TF2` 做**独立第二实现**交叉验证，三个测例的位置与旋转误差均 < 1e-15（阈值 1e-8 m / 1e-7 rad）——两种实现互相印证，结果不依赖单一代码路径。

**题三 · Isaac Sim 玩具触碰**
- 双判据确认触碰：既要 `visible_surface=touch`（接触发生在可见表面，不停在包围盒外），又要接触后玩具位移在阈值内（合指类 ≤ 2 mm、下压轻碰 ≤ 6 mm），避免「擦过」或「推倒」被判成触碰。
- 按玩具形态选姿态而不是一套参数打天下：魔方贴合侧推（`cube_side`，位移 0.124 mm）、恐龙腕部垂直下压且全程不合指（`dinosaur_press`，0.823 mm）、软玩具同类手法（`soft_toy_surface_close`，0.098 mm）。恐龙早期用斜贴侧脸会因指尖闭合弧与侧脸近乎相切而差 0.00015 mm 擦过，改成垂直下压后一次通过——说明这类侧脸近乎垂直的目标，**姿态比预压量更关键**。
- 每次触碰都落成可追溯证据：`run.json` 记三例 outcome / 策略 / 耗时，`contact-<case>.json` 记接触瞬间的十关节实测值、base / hand / toy 三方位姿与间隙，评审可据此把作答的每一行对回具体帧。

**工程化**
- 一条命令复现：`check_env.sh`（机器能不能跑）、`run_all.sh`（三题依次跑）、`run_tests.sh`（58 项单测）、`check_delivery.py`（交出去的东西对不对），四者都以退出码表达结论，可直接接 CI。
- 题三提供容器与原生 Isaac Sim 两条等价入口，执行同一份 `run_touch.py`，评审按官方环境走原生路径即可。
