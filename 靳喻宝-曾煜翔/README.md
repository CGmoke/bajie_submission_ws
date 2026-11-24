# 八界杯创新挑战赛初赛作品：八界机器人关节展示、运动学与仿真触碰

本目录是初赛作品的完整提交单元，按官方示例的章节顺序组织：作品信息、目录结构、运行环境与依赖、三道题的实现与运行、作答格式与单位、复现步骤与材料、提交前自查、原创内容与引用来源。三题源码、官方模型与仿真资源、输入、作答、运行日志和演示视频都在这里，不依赖本目录之外的任何文件。

## 一、作品信息

- **作品目录**：`靳喻宝-曾煜翔/`，与本作品报名时填写的姓名一致。
- **参赛者**：靳喻宝、曾煜翔。

**成员分工**：

| 成员   | 承担部分                                                                                                                                                                            |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 靳喻宝 | 算法与运动学：题二 `base_footprint → objects_link` 关节链提取、刚体变换与 TF2 交叉验证；题三增量 IK 规划、按玩具形态选择的接触姿态与触碰判定；题一关节抽样规则与包围盒净空模型。 |
| 曾煜翔 | 仿真与工程化：题一 ROS 2 驱动节点与 RViz 展示；题三 Isaac Sim 4.5 场景装配、ROS 2 Bridge 与启动器（原生 / 容器两条入口）；三题日志、演示视频与提交材料的整理。                      |

两人共同负责三题的联调、实机与仿真验证、结果复核与文档校对。

- **作品名称**：八界机器人关节展示、运动学与仿真触碰。
- **作品简介**：基于官方八界模型完成初赛三题。题目一在 RViz 中按升降 → 头部 → 机械臂 → 夹爪张开 → 夹爪闭合的顺序分步驱动 10 个关节，每个阶段结束记录关键帧并保持 `pause_sec`；题目二不依赖 TF，直接由 URDF 关节链自研正向运动学，给出夹爪在底盘系与物体在夹爪系的位姿，并用独立的 `robot_state_publisher + TF2` 进程交叉复核；题目三在 Isaac Sim 4.5 中经 ROS 2 Bridge 驱动机械臂，对魔方、恐龙、软玩具三件玩具逐一完成真实 PhysX 接触，并把接触瞬间的十关节实测值与三方位姿落成可回溯的证据文件。三题作答均为 JSONL，`id` 与输入一一对应。
- **创新点**：① 题目二用两套彼此独立的实现（自研 URDF 正解、ROS 2 TF2）互相印证，三个测例误差均在 1e-15 量级；② 题目三按玩具形态选姿态而不是一套参数打天下（魔方贴合侧推、恐龙与软玩具腕部垂直下压且不合指），并用「可见表面接触 + 玩具位移上限」双判据排除擦过与推倒；③ 每次触碰都落成 `run.json` + `contact-<case>.json` + `events.jsonl`，作答的每一行都能回查到具体帧；④ 一条 `src/question1_project/run_all.sh` 复现三题，四个自检脚本均以退出码表达结论，可直接接入 CI。
- **作品版本**：v1.0.0
- **作品仓库**：AtomGit 独立作品仓库，地址在建仓后回填为 `https://atomgit.com/<用户名或组织名>/<仓库名>`。
- **实现状态**：

| 题目                     | 已完成                                                                         | 待完成 | 实际验证情况                                                                                                                                                                |
| ------------------------ | ------------------------------------------------------------------------------ | ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 题目一 · 关节分步驱动   | 五阶段驱动、每阶段保持 `pause_sec`、10 关节关键帧、包围盒净空记录、RViz 展示 | 无     | 无界面实跑 1 例 5 关键帧，退出码 0；作答与官方 `q1_all_seed7` 示例十关节逐值一致，SHA-256 `6e7db6ba…dc2081`；RViz 实录见 `video/q1_rviz.mp4`                         |
| 题目二 · 正向运动学     | URDF 关节链正解、两组位姿输出、TF2 独立复核、单元测试                          | 无     | 正式测例 1 例求解通过；`--verify` 三组复核误差均 < 1e-15（阈值 1e-8 m / 1e-7 rad）；跨自研与 TF2 的位姿链复算误差 0.0 m                                                   |
| 题目三 · Isaac Sim 触碰 | 场景加载、ROS 2 Bridge、三例连续触碰、接触证据留档、作答原子写入               | 无     | 整轮三例 `outcome=contact`、`visible_surface=touch`，`exit=ok`、`written_rows=3`（run `20261124T102429Z-1b639e`，GUI 整轮 151.0 s）；作答与接触快照关节值逐值一致 |

**运行限制（如实说明）**：

- 题目三依赖物理接触，跨机器、跨驱动版本不承诺作答逐字节一致，验收以日志中的 `outcome=contact` / `visible_surface=touch` 为准；同机同版本实测可逐字节复现。
- 题目三要求 `ROS 2 Humble`（Isaac Sim 4.5 自带 Python 3.10），题目一、题目二在 Humble 与 Jazzy 上均验证可用。
- 题目三启动前会预检可用内存 ≥ 12,800 MiB、空闲显存 ≥ 6,000 MiB；实测 11.8 GB 内存亦可跑完三例，此时可设 `Q3_SKIP_CHECKS=1` 跳过预检。NVIDIA 驱动需落在 535~580，595.x 会在 RTX 初始化时段错误。
- 题目一的净空是包围盒几何估计，不是物理接触仿真；官方示例位姿本身即不满足 5 cm / 2 cm 阈值，净空就只记录、不否决播放（数值见 4.1）。
- 题目一、题目三的复跑命令统一使用 GUI 原生窗口（`run_q1.sh --gui`、`run_q3.sh --gui`），需要可用的图形桌面；无图形栈时去掉 `--gui` 即走无界面模式，作答内容一致，仅看不到三维画面。

## 二、目录结构

```text
靳喻宝-曾煜翔/
├── README.md                   # 本文件
├── src/
│   ├── bajie_robot/            # 官方 URDF 包：urdf/ meshes/ calib/ launch/ rviz/
│   ├── bajie_sim/              # 官方仿真资源：scene/ machine/ models/ scripts/ doc/（约 1.1 GB）
│   ├── question1_project/      # 题目一源码：bajie_tasks（q1_drive / q1_safety）、launch、test
│   │   ├── run_q1.sh           # 题目一入口（--gui 启动 RViz 原生窗口）
│   │   ├── run_all.sh          # 一键依次跑三题（自检 → 题一 → 题二 → 题三）
│   │   ├── run_tests.sh        # 三题单元测试（58 项）
│   │   ├── check_env.sh        # 环境自检：ROS / NumPy / SciPy / Isaac / 官方资源指纹
│   │   └── check_delivery.py   # 交付自检：结构、限位、四元数、视频体积、资源指纹
│   ├── question2_project/      # 题目二源码：run_q2.sh、fk.py / solve_q2.py / verify_tf.py / test
│   │   └── generated/
│   │       └── bajie_robot.urdf    # 官方 xacro 展开的 URDF，题目二、题目三共用
│   └── question3_project/      # 题目三源码：run_q3.sh / run_docker.sh / Dockerfile / run_touch.py / touch.py / bridge.py
├── inputs/
│   ├── q1_demo_input.jsonl     # 题目一公开输入
│   ├── q2_demo_input.jsonl     # 题目二公开输入
│   └── q3_demo_input.jsonl     # 题目三公开输入（三个玩具测例）
├── answers/
│   ├── q1_demo_output.jsonl    # 题目一实际作答结果
│   ├── q2_demo_output.jsonl    # 题目二实际作答结果
│   └── q3_demo_output.jsonl    # 题目三实际作答结果
├── docs/
│   ├── 提交自查.md             # 提交前核对清单
│   ├── 题目一_关节分步驱动与RViz展示_解题思路与创新点.md
│   ├── 题目二_正向运动学与坐标变换_解题思路与创新点.md
│   └── 题目三_IsaacSim玩具触碰_解题思路与创新点.md   # 三题各自的解法说明与设计取舍（Markdown）
├── assets/                     # 各题完整终端日志、题三结构化运行产物、哈希清单
│   ├── q1_run.log  q2_run.log  q2_humble_verify.log
│   ├── q3_docker_run.log  q3_run.log
│   ├── q3_logs/<run_id>/       # run.json、contact-<case>.json、events.jsonl
│   ├── q3_evidence_index.json  # 作答行与接触快照的对应索引
│   ├── q3_bridge_baseline.txt  # 桥接诊断摘要
│   └── manifest.json           # 交付文件字节数与 SHA-256
├── video/
│   ├── q1_rviz.mp4             # 题目一 RViz 演示
│   └── q3_touch.mp4            # 题目三 Isaac Sim 触碰演示
└── .gitignore                  # 排除 build/ install/ log/ __pycache__ 等运行产物
```

说明：

- 官方模型的入口 `urdf/bajie_robot.urdf.xacro` 位于 `src/bajie_robot/urdf/`，`scene/abr_scene.usd` 位于 `src/bajie_sim/scene/`，两个包内的官方资源与官方仓库逐字节一致（`bajie_sim` 另补了两个 shim 以便 colcon 识别资源目录，见第八节），连杆几何与关节 `origin` 未作任何改动。
- `src/bajie_sim/models/` 必须与 `scene/` 一起提交：USD 内部用相对路径引用 `../models/...`，只给入口 USD 打不开场景。
- `src/question2_project/generated/bajie_robot.urdf` 由官方 xacro 展开得到并随目录提交，题目二、题目三不需要再手动展开；其网格路径为相对路径，整个目录放到任意位置都能解析。它放在题二源码目录下，是因为 URDF 解析与正解由题二实现、题三通过 `touch.py` 复用题二的 `fk`；未放进官方包 `src/bajie_robot/`，是为了让官方包保持与上游逐字节一致。
- 根目录只保留官方示例的 `README.md` 与 `src/ inputs/ answers/ docs/ assets/ video/` 六个目录，`src/` 下也只放官方资源包与三个题目目录。所有入口脚本都放在题目目录内：题一、题二、题三各自的启动脚本在各自目录（`src/question1_project/run_q1.sh`、`src/question2_project/run_q2.sh`、`src/question3_project/run_q3.sh`）；跨三题的自检与一键运行脚本统一放在题一目录（`src/question1_project/` 下的 `run_all.sh`、`run_tests.sh`、`check_env.sh`、`check_delivery.py`），因为题一目录本就是 colcon 构建的起点、也已包含全仓单测。它们都是运行所必需的：三题命令由脚本统一封装，避免评审拼接长参数。
- 相对官方示例的目录结构，本作品多出的只有 `src/question2_project/generated/`（展开后的 URDF）与 `src/` 下的入口/自检脚本；根目录层级与官方示例一致。
- 运行时 `src/question1_project/run_q1.sh` 会在作品目录生成 colcon 的 `build/`、`install/`、`log/`，已在 `.gitignore` 中排除，交付目录内不保留。

## 三、运行环境与依赖

### 3.1 统一环境

- 操作系统：Ubuntu 22.04（题目一、题目二在 Ubuntu 24.04 亦可）。
- ROS 2：**Humble**。题目一、题目二两份测试在 Humble 与 Jazzy 均通过，题目三必须 Humble（Isaac Sim 4.5 的 Python 为 3.10）。
- 官方模型：基于官方八界模型作答，入口为 `urdf/bajie_robot.urdf.xacro`（本目录 `src/bajie_robot/urdf/`），保留官方连杆几何和关节 `origin`，SHA-256 `6dc26b06aafb82aa…0d8f49`。
- 题目一：使用 RViz 展示模型与关节运动（`--gui`）。
- 题目三：使用 **Isaac Sim 4.5** 与 **ROS 2 Bridge**，场景入口为 `scene/abr_scene.usd`（本目录 `src/bajie_sim/scene/`），SHA-256 `42d9882469…628735`。
- 官方资源来源：URDF 包 `https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf`，仿真资源包 `https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim`。本作品所用版本对应上游提交 `907b061`（URDF）与 `2fcd186`（仿真资源），见第八节。

### 3.2 本作品依赖

| 用途       | 依赖                                                                         | 版本                                                          | 安装方式                                                                                                                               |
| ---------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| 通用       | Ubuntu                                                                       | 22.04 LTS                                                     | 官方镜像                                                                                                                               |
| 通用       | ROS 2                                                                        | Humble（`ros-humble-desktop`）                              | `sudo apt install ros-humble-desktop`                                                                                                |
| 通用       | Python                                                                       | 3.10（Humble 自带 `rclpy` 为 cp310）                        | 随 ROS 2                                                                                                                               |
| 题目一     | `colcon`、`ament_cmake`、`xacro`、`robot_state_publisher`、`rviz2` | 随 Humble                                                     | `sudo apt install python3-colcon-common-extensions ros-humble-rviz2`                                                                 |
| 题目二     | NumPy                                                                        | 实测 1.21.5（Ubuntu 22.04 系统包）与 1.26.0（Isaac Sim 内建） | `sudo apt install python3-numpy`                                                                                                     |
| 题目二复核 | `tf2_ros`、`robot_state_publisher`                                       | 随 Humble                                                     | 同上                                                                                                                                   |
| 题目三     | SciPy                                                                        | 实测 1.10.1（Isaac Sim 内建）；系统 Python 需 1.8+            | `sudo apt install python3-scipy`                                                                                                     |
| 题目三     | Isaac Sim                                                                    | 4.5（含 `python.sh`、`exts/`、`extscache/`）            | 官方安装包，或按本目录 `src/question3_project/Dockerfile` 构建镜像 `bajie/sim:humble45`（基于 `nvcr.io/nvidia/isaac-sim:4.5.0`） |
| 题目三     | NVIDIA 驱动                                                                  | 535~580                                                       | 系统驱动；595.x 会在 RTX 初始化时段错误                                                                                                |
| 题目三     | `rmw_fastrtps_cpp`                                                         | 随 Humble                                                     | `sudo apt install ros-humble-rmw-fastrtps-cpp`                                                                                       |

外部材料与获取条件：

- 官方 URDF 与仿真资源随本目录一起提交，无需另行下载；如需重新获取，按 3.1 的两个 AtomGit 地址拉取，并核对上表的 SHA-256。
- 第三方算法库只用 NumPy 与 SciPy（分别按 BSD-3-Clause 使用），未引入其他运动规划或动力学库；题目三的接触判定与规划均为本作品自行实现，详见第八节。
- 题目三若走容器路径，首次构建镜像需 NGC 账号：`docker login nvcr.io`（用户名 `$oauthtoken`，密码为 NGC API Key）。镜像只用于复现，不作为提交内容。

各题的安装命令、启动命令、执行目录、输入输出路径与预期完成标志见第四节每题的「运行说明」；也可先跑一次自检：

```bash
cd 靳喻宝-曾煜翔
bash src/question1_project/check_env.sh          # 逐项打印 [可用] / [缺失]，缺失项后直接给出安装命令
```

`src/question1_project/check_env.sh` 全部可用时退出码 0，存在缺失项时退出码 1，并核对官方资源指纹。

## 四、三道题的实现与运行

以下命令一律在**作品目录内**执行；输入固定读 `inputs/q*_demo_input.jsonl`，作答固定写 `answers/q*_demo_output.jsonl`。

| 题目                     | 源码                       | 输入                           | 作答                             |
| ------------------------ | -------------------------- | ------------------------------ | -------------------------------- |
| 题目一 · 关节分步驱动   | `src/question1_project/` | `inputs/q1_demo_input.jsonl` | `answers/q1_demo_output.jsonl` |
| 题目二 · 正向运动学     | `src/question2_project/` | `inputs/q2_demo_input.jsonl` | `answers/q2_demo_output.jsonl` |
| 题目三 · Isaac Sim 触碰 | `src/question3_project/` | `inputs/q3_demo_input.jsonl` | `answers/q3_demo_output.jsonl` |

### 4.1 题目一 关节分步驱动与 RViz 展示

**源码目录**：`src/question1_project/`。ROS 2 包 `bajie_tasks` 内含驱动节点 `q1_drive`（发布 `/joint_states`）、净空模型 `q1_safety`、launch 文件 `q1_rviz.launch.py` 与单元测试 `test/`；`bajie_robot` 为官方模型包，由脚本一并编译。跨三题的 `run_all.sh`、`run_tests.sh`、`check_env.sh`、`check_delivery.py` 也放在本目录（原因见第二节说明）。

**输入文件**：`inputs/q1_demo_input.jsonl`，公开测例编号 `q1_all_seed7`：

```json
{"id": "q1_all_seed7", "step": "all", "seed": 7, "move_sec": 3.0, "pause_sec": 5.0}
```

`step` 为 `all`，`seed`、`move_sec`、`pause_sec` 按题面解析；`move_sec` 默认 3.0 s，`pause_sec` 默认 5.0 s。

**实现要求**（题面要求与对应实现）：

1. 在 RViz 中依次展示升降、头部俯仰、机械臂运动、夹爪张开和夹爪闭合。`q1_drive` 按 `lift → head → arm → gripper_open → gripper_close` 五段依次插值（smoothstep），一段只动本段关节，其余不动。加 `--gui` 时 launch 会同时拉起 `robot_state_publisher` 与 RViz，`/joint_states` 只由 `q1_drive` 一个节点发布。
2. 升降、头部和机械臂三段做完后，各等一次 `pause_sec`（默认 5 秒）。节点在这三段的收尾处调用 `hold(pause_sec)`，原地保持并继续发状态。夹爪张开与闭合之间不插停顿。
3. 每个阶段完成时记录全部 10 个关节的状态。段末把 `lift_joint`、`pitch_joint`、`joint1`–`joint6`、`arm_finger_joint1`、`arm_finger_joint2` 的当前值原样写进该段关键帧。

目标位姿由 `seed` 决定：`random.Random(seed)` 对九个预设区间依次均匀抽样，统一保留 6 位小数。同一个 `seed` 换台机器也是同一组目标值。作答与官方 README §2.2 公布的 `q1_all_seed7` 示例十关节逐值一致（误差 0.0e+00）。

**轨迹净空（记录口径）**：程序拿官方 URDF 的可视与碰撞几何拼包围盒，正向算各连杆位姿，逐段量两个下界：离 `base_footprint` 下方 z=0 地面多远，离非相连机身多远。**只记录、不否决播放**。官方示例位姿本身就压到地面以下，机身包围盒也贴住；真要据此阻断，作答就跟官方公布的关键帧对不上了。这是几何包围盒估计，不是物理接触仿真，包围盒重叠不等于网格相交。

日志里打印的「净空下界」是**整段插值运动的保守下界**：先取该段两端之间逐帧几何下界的最小值，再减去这一段里任一点可能移动的距离（旋转关节按半径 × 转角的估计）。臂段转角大，扣掉的余量也大，日志值就比该帧的几何值更负。两者不能混着比：

| 阶段          | 日志「净空下界」（整段保守，地面 / 机身） | 该帧静态几何（地面 / 机身，最低连杆）     |
| ------------- | ----------------------------------------- | ----------------------------------------- |
| lift          | 0.514 / −0.200                           | 0.759 / 0.044（`arm1_link`）            |
| head          | 0.759 / 0.044                             | 0.759 / 0.044（`arm1_link`）            |
| arm           | −0.807 / −0.407                         | −0.056 / 0.006（`gripper_right_link`） |
| gripper_open  | −0.070 / 0.006                           | −0.054 / 0.006（`gripper_right_link`） |
| gripper_close | −0.070 / 0.006                           | −0.056 / 0.006（`gripper_right_link`） |

单位米。「整段保守」列取自 `assets/q1_run.log`。lift 段的最小值落在起点（零位 0.514 m），比该帧静态值低。arm 段那个 −0.807 m，几何只占 −0.056，其余全是运动余量；夹爪实际探到地面以下 5.6 cm，跟 RViz 里看到的一致。5 cm 与 2 cm 两档都不满足，这就是不阻断的理由。

**输出文件**：`answers/q1_demo_output.jsonl`，1 行，字段为 `id`、`step`、`keyframes`、`final_joint_state`。`keyframes` 按 `lift`、`head`、`arm`、`gripper_open`、`gripper_close` 顺序记录，每帧含 `name` 与 10 关节的 `joint_state`；`final_joint_state` 与末帧逐值一致。

**运行说明**：

```bash
# 执行目录：作品目录（本 README 所在目录）
bash src/question1_project/run_q1.sh --gui    # 启动 RViz 原生窗口，边跑边看五阶段动作（评审复跑用这条）
```

`run_q1.sh` 自动 `source` ROS 2、`colcon build` 两个包并 `source install/setup.bash`，随后启动节点。`--gui` 会同时拉起 `robot_state_publisher` 与 RViz，需要可用的图形桌面（`DISPLAY` 非空）；没有图形栈时去掉 `--gui` 即走无界面模式，作答内容一致，只是看不到三维画面。输入由 `input_file` 参数读取 `inputs/q1_demo_input.jsonl`；作答写入 `answers/q1_demo_output.jsonl`；终端输出同时落到 `assets/q1_run.log`（可用 `Q1_LOG` 指定别的路径）。**预期完成标志**：日志末尾出现 `[q1] 完成：… 共 1 行`，作答文件 1 行、5 个关键帧。实测耗时约 30 s（五次移动各 3 s + 三次 5 s 保持）。手动命令等价形式见 `src/question1_project/README.md`。

**验收与演示**：RViz 中可见完整动作顺序（升降 → 头部 → 机械臂 → 夹爪张开 → 夹爪闭合）；关键帧顺序与关节字段齐全，结束态与末帧一致。完整终端日志见 `assets/q1_run.log`，RViz 演示视频见 `video/q1_rviz.mp4`。

### 4.2 题目二 正向运动学与坐标变换

**源码目录**：`src/question2_project/`：`fk.py`（URDF 解析与正向运动学）、`solve_q2.py`（命令行入口，读写 JSONL）、`verify_tf.py`（TF2 独立复核）、`tests/`（单元测试）、`src/question2_project/generated/bajie_robot.urdf`（展开后的官方模型，题二、题三共用）。

**输入文件**：`inputs/q2_demo_input.jsonl`，公开测例编号 `demo_zero`，含完整 `joint_state` 与物体相对底盘的位姿 `scene_object_in_base_footprint`：

```json
{"id":"demo_zero","joint_state":{"lift_joint":0.0,"pitch_joint":0.0,"joint1":0.0,"joint2":0.0,"joint3":0.0,"joint4":0.0,"joint5":0.0,"joint6":0.0,"arm_finger_joint1":0.0,"arm_finger_joint2":0.0},
 "scene_object_in_base_footprint":{"xyz":[0.5,-0.5,0.3],"rpy":[0.0,0.0,0.0]}}
```

**实现要求**（题面要求与对应实现）：

1. 根据关节状态和官方模型计算 `objects_link` 相对 `base_footprint` 的位姿。**运动链读取方式**：解析 `src/question2_project/generated/bajie_robot.urdf`（官方 xacro 展开，`origin` 未改），从 `objects_link` 沿父关节往上一路回溯到 `base_footprint`，拿到一条有序关节链，再倒序相乘。每段先代入模型给出的 `origin`，再在关节局部系中沿 `axis` 平移（prismatic）或旋转（revolute/continuous），即 `T_origin @ T_motion(q)`。头部俯仰与两指是旁支，其值要读取但不参与这条链；固定关节没有输入量，仍贡献位置与朝向。
2. 把物体在底盘坐标系中的位姿换到夹爪的 `objects_link` 坐标系。**坐标变换计算**：`T_objects_link^scene = (T_base^objects_link)^{-1} · T_base^scene`，其中 `T_base^scene` 直接取自输入的 `scene_object_in_base_footprint`（`rpy` 按 Rz·Ry·Rx 转成旋转矩阵）。
3. 姿态表示转换。内部一律走 4×4 齐次矩阵，只在输出时把旋转矩阵转成 `rpy`（`pitch = asin(−R20)`，`roll = atan2(R21, R22)`，`yaw = atan2(R10, R00)`）与 `quat_xyzw`（按 `[x, y, z, w]` 顺序输出，写入前归一）。两组位姿的 `rpy` 与 `quat_xyzw` 互相自洽，实测互推差 < 1e-18 rad。

求解不依赖 TF，只用 Python 与 NumPy。另起一个 `robot_state_publisher` + TF2 进程（`frame_prefix=q2_verify/`）当第二套实现，阈值 `1e-8 m` / `1e-7 rad`。三组用例（正式零位、只把 `lift_joint` 抬 0.1 m、弯臂且物体带 `rpy=[0.2,−0.3,0.5]`）实测误差都在 1e-15 量级。

**输出文件**：`answers/q2_demo_output.jsonl`，1 行，字段为 `id`、`objects_link_in_base_footprint`、`scene_object_in_objects_link`；两组位姿均含 `xyz`、`rpy`、`quat_xyzw`。

**运行说明**：

```bash
# 执行目录：作品目录
export BAJIE_URDF="$PWD/src/question2_project/generated/bajie_robot.urdf"
python3 src/question2_project/solve_q2.py --urdf "$BAJIE_URDF" \
  --input inputs/q2_demo_input.jsonl --output answers/q2_demo_output.jsonl 2>&1 | tee assets/q2_run.log

bash src/question2_project/run_q2.sh            # 同上，并自动补编译与日志
bash src/question2_project/run_q2.sh --verify   # 追加 robot_state_publisher + TF2 独立复核
```

求解本身不需要 ROS；`--verify` 才启动 `robot_state_publisher`，建议在干净的 ROS 域内执行。脚本先算完所有行再写文件（先写临时目录、成功后 `replace`），解析或计算失败也不会覆盖已有作答。**预期完成标志**：终端出现求解完成的提示与 `answers/q2_demo_output.jsonl` 路径，作答 1 行；`--verify` 时三组复核误差均 < 1e-15。实测耗时 < 5 s。

**验收与演示**：核对两组位姿的参考坐标系（`base_footprint` / `objects_link`）、单位（米、弧度）与四元数顺序（`[x, y, z, w]`）；完整终端日志见 `assets/q2_run.log` 与 `assets/q2_humble_verify.log`。题目二视频按题面为可选，本作品未录制。

### 4.3 题目三 在 Isaac Sim 中触碰玩具

**源码目录**：`src/question3_project/`：`run_touch.py`（仿真主程序）、`touch.py`（纯逻辑：输入解析、增量 IK 规划、smoothstep 采样、接触判定、作答组装与原子写文件）、`bridge.py`（ROS 2 桥接与关节指令发布）、`run_q3.sh`（原生入口与启动器）、`run_docker.sh`（容器入口）、`Dockerfile`、`tests/`（单元测试）。

**输入文件**：`inputs/q3_demo_input.jsonl`，含 `touch_toy_1`（魔方）、`touch_toy_2`（恐龙）、`touch_toy_3`（软玩具）三个测例。每个测例给出 `base_footprint_in_world`、`scene_object_in_world`、初始 `joint_state` 与玩具名。

**实现要求**（题面要求与对应实现）：

1. 在 Isaac Sim 4.5 中开官方场景、配 ROS 2 Bridge。`run_touch.py` 只开一次 `scene/abr_scene.usd`，既有 `OmniGraph` 全部置 inactive，只留本题自建的桥接图；`bridge.py` 走 ROS 2 发关节反馈与指令（默认 `rmw_fastrtps_cpp`）。CPU PhysX，60 Hz 定步长。
2. 读底盘与目标玩具的世界位姿，换到规划用的坐标系。底盘按 `base_footprint_in_world` 摆位，用内存里的 `FixedJoint` 锚住；玩具位姿取 `scene_object_in_world`。规划在底盘系做：`T_base^toy = (T_world^base)^{-1} · T_world^toy`；收尾时再乘夹爪位姿，得到玩具在 `objects_link` 里的位姿。
3. 每个测例都从输入给的初始关节状态起步，独立规划、独立控制。碰完**不归位**：沿当前腕姿竖直抬最多 8 cm，连着 4 帧不碰该玩具，再拿同一组实测十关节位姿做起点规划下一件，先竖着抬到悬停高度，再横向对齐。三例是连着的，末例停在接触位姿。
4. 目标位姿处理、运动规划、控制与触碰判定，见下。

**目标位姿处理**：玩具世界位姿先换到 `base_footprint` 系，再按玩具轮廓生成一批候选接触姿态（表面法线方向与腕部朝向的组合），逐个做增量 IK 求出十关节目标。

**运动规划**：起点用当前实测关节位姿，不用指令值。增量 IK（`scipy.optimize.least_squares`）求出关节解，按 smoothstep 采成 18 个路点，限位与速度在规划阶段就校验掉。接近姿态是两指张开、由上而下贴目标。

**控制**：路点一条条下发，等实测到位。手腕位置与速度连着四步稳住，才允许合指；合指期间腕部目标冻结。指 2 是场景里的随动关节，由指 1 带动。

**触碰判定（双判据）**：同一根夹指要连着 6 个仿真步同时满足两件事：对目标有带冲量的非正分离接触，夹指落在玩具的可见网格上。只碰到碰撞外壳不算。首次满足时保持指令，连续 6 步无有效接触才沿原方向微调，最多 4 次（合指每次 0.003 rad、下探每次 0.25 mm）。合指类动作的玩具位移上限 2 mm，`*_press` 垂直下压是有意轻碰，上限放宽到 6 mm。出现非目标碰撞即中止。

**按玩具选择姿态**：魔方用 `cube_side`，合拢方向对准一条侧面法线。恐龙用 `dinosaur_press`，腕部垂直下压、全程不合指，接触由下降产生，位移 0.823 mm。恐龙侧脸近乎垂直，贴侧面合指会沿闭合圆弧把玩具扫走（实测 2.14 mm），改成垂直下压只推 0.82 mm。软玩具同样优先 `soft_toy_press`，够不着再退到 `soft_upper` / `surface_close`。

**输出文件**：`answers/q3_demo_output.jsonl`，三行，每行沿用输入 `id`，报告触碰完成时的 `final_joint_state`（10 个关节实测值）、夹爪相对底盘的 `objects_link_in_base_footprint`，以及玩具在 `objects_link` 坐标系中的位姿 `scene_object_in_objects_link`。三例在同一会话里碰完、逐例确认脱离之后，才按输入顺序**原子替换**作答文件。哪一例没确认接触，就不写半份。

**运行说明**：

```bash
# 执行目录：作品目录（需要可用的图形桌面）
bash src/question3_project/run_q3.sh --check                              # 只探测 Isaac Sim / ROS 2 / 资源 / 内存 / 显存
bash src/question3_project/run_q3.sh --gui --case rubik_cube              # 单例排查，约 2~3 分钟，打开 Isaac Sim 原生窗口
Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_q3.sh --gui   # 整轮三例，带原生窗口并写正式作答

# 无 Isaac Sim 时的容器路径（同样开原生窗口：Q3_GUI=1）
docker build -f src/question3_project/Dockerfile -t bajie/sim:humble45 src/question3_project
Q3_GUI=1 Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_docker.sh
```

原生路径需要 `export ISAAC_PATH=<Isaac Sim 4.5 安装根目录>`（该目录须含 `python.sh` 与 `exts/`），首次无头启动建议同时 `export ACCEPT_EULA=Y PRIVACY_CONSENT=Y`。`--gui` 打开 Isaac Sim 原生窗口，需要可用的 `DISPLAY`；没有图形桌面时去掉 `--gui` 走无界面模式，接触判定与作答完全一致。启动器自动定位 Isaac Sim、`source` ROS 2 Humble、预检内存与显存，超时自动终止（单例 600 s、整轮 1800 s，可用 `Q3_TIMEOUT` 覆盖）。默认策略 `adaptive-fallback`，日志逐例标注实际生效的 `strategy=`。**预期完成标志**：末尾出现 `run_finish exit=ok written_rows=3`，三例均 `outcome=contact`、`visible_surface=touch`，作答 3 行。实测整轮 151.0 s（GUI 模式，着色器缓存已热）。

**验收与演示**：仿真里能看到三例各自的触碰过程，作答与运行记录对得上。本次提交的作答出自 `20261124T102429Z-1b639e` 这一次整轮运行（`exit=ok`、`written_rows=3`、`misses=[]`），run 目录里的 `run.json` 和 3 份 `contact-*.json` 就是同源证据，逐例对应关系见 `assets/q3_evidence_index.json`。三例作答的关节值与接触快照逐值一致，偏差 0.0 rad；从快照三方位姿反算出来的作答位姿，与文件相差不到 2e-15 m。该次整轮的结构化日志见 `assets/q3_logs/20261124T102429Z-1b639e/`（早期 docker 整轮终端日志另见 `assets/q3_docker_run.log`），Isaac Sim 触碰视频见 `video/q3_touch.mp4`。

## 五、作答格式与单位

- 输入与输出均为 **JSONL**，每行一个完整 JSON 对象；作答的 `id` 与对应输入一致，行数与输入一致（1 / 1 / 3）。
- 完整关节状态包含 `lift_joint`、`pitch_joint`、`joint1` 至 `joint6`、`arm_finger_joint1`、`arm_finger_joint2`，共 10 个字段。
- 升降位置与位姿中的 `xyz` 以米为单位；旋转关节角与 `rpy` 以弧度为单位。
- `rpy` 依次表示滚转、俯仰、偏航；`quat_xyzw` 按 `[x, y, z, w]` 顺序填写，写入前已归一（模长 1.000000000000）。
- `base_footprint` 是底盘坐标系，`objects_link` 是夹爪的抓取参考坐标系。
- 字段清单：

| 题目   | 每行字段                                                                                                               |
| ------ | ---------------------------------------------------------------------------------------------------------------------- |
| 题目一 | `id`、`step`、`keyframes`（5 帧，每帧 `name` + `joint_state`）、`final_joint_state`                        |
| 题目二 | `id`、`objects_link_in_base_footprint`、`scene_object_in_objects_link`（各含 `xyz` / `rpy` / `quat_xyzw`） |
| 题目三 | `id`、`final_joint_state`、`objects_link_in_base_footprint`、`scene_object_in_objects_link`                    |

- 当前三份作答的 SHA-256：`answers/q1_demo_output.jsonl` = `6e7db6baf7f1f5a2…9fdc2081`，`answers/q2_demo_output.jsonl` = `98fd3d8092265eef…42474cc37`，`answers/q3_demo_output.jsonl` = `374922903c427b31…6c6dd921`（完整值见 `assets/manifest.json`）。
- 题目三的玩具位姿为**接触时刻的实测值**，不是输入的初始位姿：玩具被碰到后本身会位移（日志记 `max_toy_shift_mm` 0.15~1.61 mm），这一点在核对时需注意。

## 六、复现步骤与材料

从克隆仓库开始，到三题各自写出作答为止，按顺序给命令。每步都写了预期输出；跑不出来就照着那行提示往下查。

**第 0 步　确认前置**

| 项 | 要求 | 怎么查 |
|---|---|---|
| 系统 | Ubuntu 22.04（题一、题二 24.04 也可；题三必须 Humble） | `lsb_release -ds` |
| 内存 | 题一、题二不限；题三可用 ≥ 12,800 MiB | `free -m \| head -2` |
| 显卡 | 题三：空闲显存 ≥ 6,000 MiB，驱动 535~580 | `nvidia-smi --query-gpu=driver_version,memory.free --format=csv,noheader` |
| 磁盘 | 克隆约 1.2 GB，跑题三另留 ≥ 20 GB | `df -h . \| tail -1` |

题一、题二不碰 GPU，没有显卡也能跑完；题三没到门槛会在启动前直接拒绝，不会跑到一半才失败。

**第 1 步　克隆仓库**

```bash
git clone <仓库地址>
cd <仓库名>/靳喻宝-曾煜翔
ls
```

预期列出 `README.md  src  inputs  answers  docs  assets  video` 七项。作品目录名是报名姓名，必须 `cd` 进去，后面所有命令都在这一层执行。

拿到的是压缩包时，解压后同样 `cd` 进去，效果一样。

**第 2 步　装依赖**

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions \
                 ros-humble-rmw-fastrtps-cpp python3-numpy python3-scipy
```

只装 `ros-humble-ros-base` 的话，题一的 `--gui` 还需要补 `ros-humble-rviz2`。

题三走容器路径时，另装 Docker 与 NVIDIA Container Toolkit（`docker run --gpus all` 能出显卡即可），不必装 Isaac Sim。

**第 3 步　环境自检**

```bash
bash src/question1_project/check_env.sh
```

预期：逐项打印 `[可用]` / `[缺失]`，缺失项后面直接给出安装命令；末尾核对两个官方资源指纹。全部可用时退出码 0。

题三的内存、显存项在这一步可能报缺失，属正常——它不影响题一、题二。

**第 4 步　构建**

首次运行脚本会自动 `colcon build`。想先单独构建一次、把错误提前暴露：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/question1_project src/bajie_robot \
  --packages-select bajie_robot bajie_tasks --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

预期：`Summary: 2 packages finished`。生成的 `build/`、`install/`、`log/` 由 `.gitignore` 排除，不随仓库分发，也不影响作答。

**第 5 步　跑题一**

```bash
bash src/question1_project/run_q1.sh          # 无界面，约 30 秒
bash src/question1_project/run_q1.sh --gui    # 同时打开 RViz 看五个阶段
```

预期：终端五段各打印一行净空下界，末尾 `[q1] 完成：… 共 1 行`；`answers/q1_demo_output.jsonl` 1 行、5 个关键帧。

`--gui` 需要可用的图形桌面；没有就留默认，作答内容一致。

**第 6 步　跑题二**

```bash
bash src/question2_project/run_q2.sh            # 求解，5 秒内
bash src/question2_project/run_q2.sh --verify   # 追加 TF2 独立复核
```

预期：`--verify` 时三组用例复核误差均小于 1e-15；`answers/q2_demo_output.jsonl` 1 行、两组位姿。

题二的求解不需要 ROS，`--verify` 才启动 `robot_state_publisher`。

**第 7 步　部署题三（两条路选一条）**

原生 Isaac Sim 4.5：

```bash
export ISAAC_PATH=$HOME/.local/share/ov/pkg/isaac_sim-4.5.0   # 该目录须含 python.sh 与 exts/
export ACCEPT_EULA=Y PRIVACY_CONSENT=Y
bash src/question3_project/run_q3.sh --check                  # 只探测，不启动
```

容器（本机无需安装 Isaac Sim）：

```bash
docker build -f src/question3_project/Dockerfile -t bajie/sim:humble45 src/question3_project
bash src/question3_project/run_docker.sh --check
```

预期：两种 `--check` 都会打印 Isaac 版本或镜像标签、ROS 来源、可用内存、空闲显存与资源路径。任一项不达标会直接说明不启动，不会进入仿真。

**第 8 步　跑题三**

```bash
# 先跑单例，确认场景能起、机械臂能动（约 2~3 分钟，不写正式作答）
bash src/question3_project/run_q3.sh --case rubik_cube

# 再跑整轮三例，写正式作答
Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_q3.sh
```

容器路径把 `run_q3.sh` 换成 `src/question3_project/run_docker.sh`；要开窗口就在前面加 `Q3_GUI=1`。

预期：末尾 `run_finish exit=ok written_rows=3`，三例均 `outcome=contact`、`visible_surface=touch`；`answers/q3_demo_output.jsonl` 3 行。首轮冷启动要编译着色器，耗时比记录的 151.0 秒长不少。

**第 9 步　校验产出**

```bash
python3 src/question1_project/check_delivery.py   # 10 项结构/限位/四元数/视频/指纹
sha256sum answers/*.jsonl                        # 与第五节记录的哈希对照
bash src/question1_project/run_tests.sh           # 58 项单测
```

预期：自检 10 项 OK / 0 FAIL；三个哈希与第五节一致（题三换机器会变，属正常，判据看 `outcome=contact`）；单测 58 项全过（5 + 8 + 45）。

改过作答或日志之后，用 `python3 src/question1_project/check_delivery.py --manifest assets/manifest.json` 重生成哈希清单，这是整套脚本里唯一的写操作。

想一条命令跑完三题：

```bash
bash src/question1_project/run_all.sh
```

批量模式题一、题三自动走无界面，适合无人值守；需要录演示时再单独跑带 `--gui` 的命令。

**材料核对**

`assets/` 里是三题完整终端日志与题三 7 次运行的结构化产物（`run.json`、`contact-<case>.json`、`events.jsonl`），`video/` 里是题一与题三的演示视频，`docs/` 里是提交自查与三题的解法说明。这些都已随仓库提供，复跑时会被覆盖同一路径，属正常。

`inputs/` 的测例与正式题面一致：题一 1 例 `q1_all_seed7`、题二 1 例 `demo_zero`、题三 3 例。

### 6.1 作品视频要求

| 文件                   | 对应题目 | 内容                                           | 规格                                                                              |
| ---------------------- | -------- | ---------------------------------------------- | --------------------------------------------------------------------------------- |
| `video/q1_rviz.mp4`  | 题目一   | RViz 内完整展示五个阶段的关节驱动              | 32.6 s，976×720，25 fps，H.264 + AAC，888,772 字节（0.89 MB）                    |
| `video/q3_touch.mp4` | 题目三   | Isaac Sim 4.5 内魔方、恐龙、软玩具三例触碰实录 | 2 min 10 s（129.6 s），1104×720，30 fps，H.264 + AAC，24,723,064 字节（24.7 MB） |

- 两个必交视频均不超过 **30,000,000 字节**（合计 25,611,836 字节），容器 MP4、视频 H.264、音频 AAC，符合题面的推荐格式，导出后已核对可正常播放。
- 题目二视频按题面为可选，本作品未录制，不影响提交。
- 录制方式：题目一 `bash src/question1_project/run_q1.sh --gui`（需可用图形桌面，无图形栈时去掉 `--gui` 作答一致但无法录制）；题目三在 Isaac Sim 原生窗口下录制：`Q3_RESULT_DIR="$PWD/answers" bash src/question3_project/run_q3.sh --gui`（需可用 DISPLAY，先验证单帧再整轮录制）；无 Isaac Sim 时容器路径等价写法为 `Q3_GUI=1`。视频为实际运行录屏，未用渲染探针或截图替代。

### 6.2 仓库登记与提交

1. 在 **AtomGit** 上重新创建独立的作品仓库，把 `靳喻宝-曾煜翔/` 目录放到仓库根，推送整理好的材料（只推这一个目录，不要带开发工作区中的中间产物、归档目录与内部核对文档）。
2. 在飞书作品提交表中填写作品信息与 AtomGit 仓库的完整主页链接，例如 `https://atomgit.com/用户名或组织名/仓库名`。
3. 按表单说明配置评审访问权限，确认组委会能够访问所登记的仓库；提交、链接登记与信息更新统一以飞书表单说明为准。
4. 推送后在干净 clone 中复跑一次自检：`python3 src/question1_project/check_delivery.py`，期望 10 项 OK / 0 FAIL。

仓库体积约 1.2 GB，其中官方仿真资源（`src/bajie_sim/`）占 1.1 GB；若仓库有单文件或总量限制，可按 3.1 的地址把该目录改为外链并在本节注明版本与 SHA-256。

## 七、提交前自查

- [ ] 已填写作品信息、成员分工、功能与创新点及作品版本（作品版本 v1.0.0 已填；作品仓库地址需建仓推送后回填，见第一节）。
- [ ] 已在 AtomGit 上重新创建独立作品仓库，并按飞书作品提交表要求登记仓库链接、作品信息及评审访问权限。
- [X] 三道题的完整源码和必要配置均放入对应的 `src/` 目录。
- [X] 已列明依赖版本、安装方法、各题执行目录及实际启动命令（第三、四节）。
- [X] 每道题的命令均能找到对应输入并生成作答文件。
- [X] 输出采用 JSONL，测例编号、关节字段、位姿字段与单位符合题面要求（第五节）。
- [X] 已在 `assets/` 提供各题完整运行日志，在 `video/` 提供题目一、题目三的演示视频。
- [X] 单个视频文件大小不超过 30 MB，视频能够正常播放，关键动作与结果清晰（6.1）。
- [X] 已在文档中说明实际验证结果、尚未完成的功能和运行限制（第一节「实现状态」与「运行限制」）。
- [X] 外部材料的来源、版本、许可与访问条件已写明（3.2 与第八节）。
- [X] 提交材料范围已检查，密钥、访问令牌、个人隐私数据及无权公开的材料已清理；`build/`、`install/`、`log/`、`__pycache__` 已由 `.gitignore` 排除，交付目录内也不再保留。

## 八、原创内容与引用来源

**自行实现的内容**：

| 范围   | 自行实现的部分                                                                                                                                                                                                                 |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 题目一 | `bajie_tasks/q1_drive.py`（五阶段驱动、`seed` 抽样、关键帧与作答组装、原子写文件）、`bajie_tasks/q1_safety.py`（URDF 包围盒构造、关节树正解、整段插值净空估计）、`q1_rviz.launch.py`、单元测试；`run_q1.sh` 启动脚本 |
| 题目二 | `fk.py`（URDF 关节链提取与正向运动学）、`solve_q2.py`（JSONL 读写、位姿变换、`rpy`/四元数互转、原子写出）、`verify_tf.py`（TF2 独立复核）、单元测试                                                                    |
| 题目三 | `run_touch.py`、`touch.py`（增量 IK、smoothstep 路点、按玩具形态的候选姿态、接触判定与作答原子写入）、`bridge.py`（ROS 2 桥接）、`run_q3.sh` / `run_docker.sh` 启动器、`Dockerfile`、单元测试                      |
| 工程化 | `src/question1_project/run_all.sh`、`run_tests.sh`、`check_env.sh`、`check_delivery.py`、`assets/manifest.json` 与 `assets/q3_evidence_index.json` 的生成逻辑                                                      |

**引用的第三方材料**：

| 材料                                                                                | 来源                                                     | 版本 / 标识                                                                             | 许可与获取条件                                                                                                                              |
| ----------------------------------------------------------------------------------- | -------------------------------------------------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| 八界机器人 URDF 模型包（`src/bajie_robot/`）                                      | `https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf` | 上游提交 `907b061`；入口 `urdf/bajie_robot.urdf.xacro` SHA-256 `6dc26b06…0d8f49` | 随比赛提供，按比赛要求在本次作答范围内使用；上游仓库未附明确许可声明，故本作品原样引用、未作几何或 `origin` 修改                          |
| 八界仿真资源包（`src/bajie_sim/`）                                                | `https://atomgit.com/ecovacs-bajie/ecovacs-bajie-sim`  | 上游提交 `2fcd186`；入口 `scene/abr_scene.usd` SHA-256 `42d98824…628735`         | 同上；`scene/` 与 `models/` 为官方原件，仅补 `package.xml`、`CMakeLists.txt` 两个 shim 以便 colcon 识别资源目录，资源文件本身未改动 |
| 官方容器基础镜像                                                                    | `nvcr.io/nvidia/isaac-sim:4.5.0`                       | 4.5.0                                                                                   | NVIDIA NGC 许可；首次拉取需 `docker login nvcr.io`，镜像仅用于复现，不作为提交内容                                                        |
| NumPy                                                                               | PyPI / Ubuntu 包                                         | 实测 1.21.5、1.26.0                                                                     | BSD-3-Clause                                                                                                                                |
| SciPy                                                                               | PyPI / Ubuntu 包                                         | 实测 1.8.0、1.10.1                                                                      | BSD-3-Clause                                                                                                                                |
| ROS 2 Humble（`rclpy`、`tf2`、`robot_state_publisher`、`xacro`、`rviz2`） | ROS 2 官方发行版                                         | Humble                                                                                  | Apache-2.0 / BSD-3-Clause 等，按发行版条款使用                                                                                              |

未使用其他开源算法库、预训练模型或数据集；题目三的运动规划、接触判定均为本作品自行实现，未移植第三方代码。
