# R1-7A MuJoCo 键盘抓取环境

[English](README.md) | **简体中文**

![银白色 R1-7A 机械臂抓取演示](grasp_preview.png)

使用相邻 `r1-7a_sdk` 的原始 URDF、质量/惯性、关节限位及 STL 外观，搭建固定底座机械臂、桌面、植物盆栽和绿色放置区。末端为 **XLeRobot / SO101 TPU95A Fin Ray 夹爪**及腕部 D435 相机。花盆直径60 mm、质量估计 80 g；一侧夹指固定，另一侧由旋转关节驱动。

网格保持原始尺寸，仅作毫米转米和刚体坐标变换。R1 法兰转接件、安装位置和 TCP 是本项目的仿真适配，尚未按真实安装标定。

键盘遥操作与 `yolo_grasp/` 现在共用同一套场景、盆栽、初始姿态、运动控制和电机参数，详见 [共用控制说明](SHARED_CONTROL.zh-CN.md)。末端质量和质心集中在 `end_effector.json`，总质量估计为 265 g。`--object cube` 可选择原方块，`--no-wrist-camera` 可关闭相机负载。SDK 对齐依据见 [SIM2REAL.zh-CN.md](SIM2REAL.zh-CN.md)。

机械臂外观参考 [Unitree R1-ARM 官网产品图](https://www.unitree.com/R1-ARM/)，主体使用略带冷灰的银白色及金属高光，末端接口保留深色。颜色和光泽为根据产品图设置的视觉近似，并非官方色号；SO101 TPU 夹爪维持原有配色。

## 本次更新 — 2026-10-29

- 默认抓取物替换为下载的 04 号盆栽，花盆外径等比缩放为 **60 mm**、盆高约 **55.4 mm**。键盘和 YOLO 共用场景、腕部 D435、观察姿态、末端负载与抓放控制器。
- 新增真实 YOLOE 图像推理和掩膜内 RGB-D 定位：**最多六个腕部观察位扫描 → 检测目标 → 确认停稳 → 重新采集 RGB、深度与外参 → 拟合盆沿 → 抓取放置**。定位失败继续换观察位，全部失败后停止。
- 支持 `--randomize --seed 42` 随机位置及朝向；已测试区域为 **X=0.10–0.18 m、Y=0.25–0.31 m**。G 开始扫描，随机模式下 R 复位并重新摆放，V 仅在当前位置检测。
- 统一受限力矩的 `q/dq/tau/kp/kd` 控制、100 Hz 主机发令与 500 Hz 内环/物理更新、可配置重力前馈、SDK 指令量化、渐进开合及可选 CSV 记录。
- 根据 6 cm 盆径调整固定指进场间隙与 20 mm 抓取深度，新增触桌松爪及五次摩擦求解收敛迭代。目标仍是自由刚体，靠接触摩擦抓取。
- 增加电机、SDK、共享场景、搜索和 GUI 验证。六组随机朝向的 RGB-D 抓放测试通过，定位误差约 0.66–1.81 mm；其中两组换观察位后成功。

```bash
python -m pip install -r yolo_grasp/requirements.txt
./yolo_grasp/run.sh --auto --randomize --seed 42
# 无窗口运行（需要 EGL）：
./yolo_grasp/run.sh --headless --randomize --seed 42
```

**感知数据来源：** YOLOE 对渲染 RGB 图像实际推理，识别和抓取定位不读取仿真物体 ID 或 XYZ。抓取坐标由模拟深度图和相机内参反投影得到；深度为 MuJoCo 理想渲染，相机世界位姿也来自仿真。筛选使用已知桌面高度、花盆尺寸及浅色盆体条件。物体真值与接触 ID 用于结果验证和接触反馈。这不是实物 D435 测量，毫米级误差不代表真机精度。

本次 Git 发布只包含运行用 OBJ/PNG、代码、配置、验证、文档和许可文件；模型权重、文本编码器、缓存、运行输出、录制数据、生成场景、旧盆栽与重复原始下载均不上传。

## 启动

需要 Python 3.10+，图形窗口需要可用的 OpenGL 桌面环境。已在 Linux / Python 3.12 环境验证。

```bash
git clone https://github.com/llros007/unitree_r1-7a-mujoco.git
cd unitree_r1-7a-mujoco
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run.sh
```

也可在激活的 conda 环境中安装依赖后运行 `python main.py`。设置 `PYTHON=/path/to/python ./run.sh` 可指定解释器。

## YOLO RGB-D 自动抓取（可选）

新增 [yolo_grasp/](yolo_grasp/README.zh-CN.md)，使用 YOLOE 和夹爪腕部 D435 RGB-D 仿真相机自动定位、抓取、搬运与放置，暂不接入 GraspNet。

```bash
python -m pip install -r yolo_grasp/requirements.txt
./yolo_grasp/run.sh
```

按 **G** 扫描、停稳复测并抓取，按 **V** 仅检测。扩展依赖单独安装，原有键盘仿真可继续使用。

## 键盘与鼠标

先点击仿真窗口，让窗口获得键盘焦点；按住按键连续移动，松开保持目标位置。移动采用**世界坐标系**，与相机视角无关。

| 按键 | 功能 |
| --- | --- |
| W / S | 末端沿 Y 正 / 负方向移动 |
| A / D | 末端沿 X 负 / 正方向移动 |
| Q / E | 末端上升 / 下降（Z 正 / 负） |
| I / K | 绕世界 X 轴正 / 负旋转 |
| J / L | 绕世界 Y 轴正 / 负旋转 |
| U / O | 绕世界 Z 轴正 / 负旋转 |
| Shift + 移动或旋转 | 慢速微调 |
| Space | 切换夹爪开 / 合 |
| R | 重置机械臂、物体及夹爪，回到共用观察姿态 |
| P | 暂停 / 继续 |
| H | 终端打印帮助 |
| Esc | 退出 |
| 鼠标左键拖动 | 旋转相机 |
| 鼠标右键拖动 | 平移相机 |
| 滚轮 | 缩放 |

移动速度 0.10 m/s，Shift 微调速度 0.025 m/s。角速度为 0.75 rad/s，微调为 0.25 rad/s。左上角显示实际 TCP、目标 TCP、物体位置和双侧接触状态。绿色小点是实际 TCP，黄色小球是目标 TCP。

## 抓取步骤

1. 初始为斜视盆栽的相机观察姿态，TCP 约 `(0.12,0.08,0.83)` m。先用 I/K 将夹爪转正朝下，再用 W 移到花盆附近 Y≈0.25 m。
2. 张爪下降前给固定指一侧留几毫米间隙，参考 TCP X≈0.104 m；按 E 下降到 Z≈**0.635 m**，避免固定指先撞到盆沿。
3. 按 Space 合拢，等待约 3～4 秒。夹爪按 0.30 rad/s 平滑开合，`contacts 2/2` 表示两侧都接触盆体。
4. 按 Q 抬起，成功后显示 `OBJECT LIFTED!`。
5. 搬运至绿色放置区 `(-0.12,0.25)` m，下降到 TCP Z≈0.642 m 后释放，等待张开再抬起。

按键按得过久可能超过可达空间，IK 会限制目标；接近关节限位时换方向或按 R 复位。IK 只求末端位姿，不做障碍物路径规划，绕行和姿态由操作者控制。

## 验证与截图

无界面的物理抓取、搬运、释放验证（失败返回非零退出码）：

```bash
./run.sh --headless --demo
python verify.py
python verify_shared.py
```

`verify.py` 还验证五参数控制、力矩限幅、零力矩时的重力运动、静态跟踪误差、电机响应滞后，以及实际键盘处理逻辑中的下降、合拢、抬升、暂停和复位。

生成抓起瞬间的截图：

```bash
./run.sh --headless --demo --snapshot grasp_preview.png
```

只有渲染截图和图形窗口需要 OpenGL；无桌面服务器可用 `MUJOCO_GL=egl` 配合截图命令。纯 `--headless --demo` 不创建图形上下文。

图形窗口冒烟测试：`./run.sh --seconds 3`。

## 电机参数与真机控制接口

七个机械臂关节使用 `gear=1` 的力矩电机，每个 2 ms 物理步计算：

```text
tau_target = clip(tau_ff + kp * (q_des - q) + kd * (dq_des - dq), -tau_limit, tau_limit)
```

机械臂和夹爪均已关闭 MuJoCo 的 `gravcomp`。键盘 IK 模式在每个 100 Hz 发令周期按平滑后的目标角计算 `tau_ff = gravity_feedforward_scale × g(q_des)`。`g(q_des)` 来自独立保存的名义模型在零速度下的重力项，包含已建模的机械臂、夹爪和相机质量。五参数经过 SDK 标定转换、整数编码和解码后保持至下个发令周期。500 Hz 内层 PD 使用实时关节状态计算力矩，和前馈合并后统一限幅。松键后冻结 IK 解。窗口显示前馈力矩、实际电机力矩和最大关节误差。夹爪仍使用受限力的位置舵机。

修改 `motor_config.json` 后重新启动生效，数组顺序为 J1～J7：

| 参数 | 单位 / 默认值与来源 |
| --- | --- |
| `kp` | Nm/rad；默认均为 60，取自当前相邻 SDK 的 `test_arm_lowlevel.cpp` 默认值 |
| `kd` | Nm·s/rad；默认 6、6、4、4、2、2、1.5，为仿真运动停止后的收敛调节；不是实测标定值 |
| `tau_limit` | Nm；默认 60、60、33、33、33、10、10，来自 URDF effort；可调低，运行时不会超过 MJCF 上限 |
| `damping` | Nm·s/rad；被动关节阻尼，默认 0.1，是未辨识的估计值，区别于控制器 `kd` |
| `frictionloss` | Nm；关节干摩擦，默认 0，尚未辨识 |
| `armature` | kg·m²；折算到关节端的附加转动惯量，默认 0.015，尚未辨识 |
| `torque_time_constant_s` | s；一阶力矩响应时间常数，默认 0 表示即时响应；设为实测值后启用滞后 |
| `cartesian_reference_time_constant_s` | s；默认 0.0625，以临界阻尼二阶滤波生成连续的 IK 位置与速度参考；设 0 可关闭；只影响 IK，直接五参数指令绕过此滤波。旧配置省略时默认关闭 |
| `gravity_feedforward_scale` | 无量纲；默认 1.0，键盘 IK 模式启用模型重力前馈；设 0 可关闭，旧配置省略时默认关闭。范围 0～1；直接五参数指令不受影响 |
| `command_period_s` | s；默认 0.01，按 SDK 运动示例的 100 Hz 发令；必须为物理步长的整数倍。旧配置省略时使用每物理步发令 |
| `sdk_protocol_calibration` | 默认 `assets/joint_calib.txt`，相对电机配置文件定位；启用 SDK 标定转换和指令量化。设 null 或省略可关闭 |
| `gripper_speed_rad_s` | 默认 0.30 rad/s；两入口共用的夹爪目标角速度 |
| `grasp_approach_clearance_m` | 默认 0.006 m；共用自动抓取状态机给固定指侧保留的进场间隙 |

也可选择另一份完整配置：

```bash
./run.sh --motor-config /absolute/path/to/motors.json
```

`motor_config.sdk.json` 对应先前低层保持示例的 `kp=60、kd=0.5`、50 Hz 发令，关闭参考滤波及重力前馈，保留 SDK 指令量化。可用 `./run.sh --motor-config motor_config.sdk.json` 对照。默认配置保持重力开启，通过 `tau` 补偿。相同名义模型下悬停误差很小，不能代表真机精度；见 [对齐记录](SIM2REAL.zh-CN.md)。`verify.py` 验证带腕部相机时 12 个方向松键后的收敛、键盘抓取与搬运。

用 `./run.sh --record outputs/keyboard_trace.csv` 记录命令和响应，路径必须为新文件；同名 `.metadata.json` 保存模型、标定、参数和末端质量。`python verify_sim2real.py` 额外检查 SDK 编码一致性、100 Hz 指令保持、记录与复位，以及控制器模型和被控模型的质量差异。

在本目录的 Python 控制代码中直接发送五参数指令：

```python
from environment import ArmEnvironment
from wrist_scene import build_scene

env = ArmEnvironment(scene_path=build_scene())
env.set_joint_command(q=env.home_q, dq=0, tau=0)  # kp/kd 使用配置值。
for _ in range(1000):
    env.step(0.002)

# 零力矩：取消位置/速度反馈，手臂会受重力影响。
env.set_joint_command(q=env.home_q, kp=0, kd=0, tau=0)
env.step(0.01)
env.use_cartesian_control()  # 从当前位置恢复 IK 控制；reset() 也会恢复。
```

`q` 为七元素数组；其余参数支持标量或七元素数组。`q/dq/tau` 单位为 rad、rad/s、Nm，均在 URDF 关节坐标、关节输出端定义，`tau` 是前馈力矩。直接接口调用时立即提交一份指令，经已启用的 SDK 量化后保持至替换或复位；调用频率由调用方决定。外层 IK 不覆盖或平滑直接关节指令，**不会再额外叠加重力前馈**。默认键盘 IK 模式的 `dq_des` 来自平滑参考的解析导数。

这实现了 SDK 风格的 PD + 前馈控制律、标定转换与指令量化，尚未复现固件的电流环、总线延迟、噪声、间隙、温度限制或力矩随转速变化。末端质量和惯量仍是估计值，目前没有真机日志，不能量化与真机的差距。需要称重和采样后校验负载、摩擦与响应。原有截图产生于旧的位置伺服配置。

## 文件与建模说明

- `main.py`：GLFW 窗口、键盘持续输入、鼠标相机及状态显示。
- `environment.py`：500 Hz MuJoCo 动力学与内层电机控制、100 Hz 外层 IK、直接关节指令、抓取验证。
- `motor_control.py`、`motor_config.json`：七关节 PD + 前馈力矩、限幅、可选响应滞后和参数。
- `wrist_scene.py`、`motion.py`：两入口共用的盆栽场景和抓放状态机。
- `build_scene.py`：从 SDK URDF 生成场景；可选重建步骤见下文。
- `xle_gripper.py`：SO101 单侧旋转夹爪、开合角度、TCP 和 TPU 接触参数。
- `prepare_gripper.py`：从 XLeRobot 原始 3MF 工程提取夹爪网格；正常运行无需重新提取。
- `assets/gripper/`：官方夹爪网格的坐标转换版本、Apache-2.0 许可证及完整来源记录 `SOURCE.md`。
- `assets/scene.xml`、`assets/meshes/`：已生成场景和复制的原始网格；直接运行不依赖 ROS，也不再依赖相邻 SDK。
- `assets/SDK_LICENSE`：原模型所在 SDK 的许可证；网格来源为相邻 `r1-7a_sdk/urdf/`。
- `grasp_preview.png`：实际仿真中夹起盆栽的截图。

机械臂使用受限幅的力矩控制，机械臂和夹爪均承受自身重力。保留机械臂对桌面、地面及盆体的碰撞；机械臂自身碰撞关闭，机械臂网格碰撞使用 MuJoCo 凸包。**盆栽始终是自由刚体，通过接触摩擦抓取，没有焊接、吸附或抓起时修改物体位置**。叶片与土壤仅作外观；来源见 [盆栽模型说明](yolo_grasp/assets/plant04/SOURCE.md)。

SO101 夹爪默认打开角度为 0.8 rad，合拢目标为 -0.13 rad，关节范围 [-0.20, 1.05] rad；这些是本仿真安装下的设定，不是实物舵机零点。TCP 在转接后夹爪坐标的 `(0, 0, 0.106)` m，距原 Link7 原点共 0.146 m。

TPU 手指外观保留 Fin Ray 内部肋条，网格本身仍是刚体。抓取面使用分段碰撞代理和软接触（`solref=0.012 1`、摩擦系数 1.8）近似柔顺性，**不包含 TPU 弹性体或肋条弯曲的有限元模拟**。夹爪质量、惯性、摩擦、柔顺性与驱动力矩均未经过实物辨识，不能据此判断真实抓力或形变。

夹爪网格来自 [XLeRobot hardware](https://github.com/Vector-Wangel/XLeRobot/tree/3d14695e40c9c68229c0aacffca6053c75cd3eb6/hardware) 的 `XLeRobot_0_3_0.3mf`，对象 47 为固定座、39 为活动座、49 为 TPU 手指。仓库已有独立的 `SO101_soft_fin.stl` 打印布局；这里从同一 3MF 提取独立部件，避免把打印排版位置当作装配位置。详细转换和来源哈希见 `assets/gripper/SOURCE.md`。

这些设置适用于仿真交互和算法原型，不构成真实 R1-7A 的电机控制标定。启动程序不会打开串口或连接真实机械臂。

MuJoCo API 参考：[Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)、[MJCF XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。

## 可选：重新生成资产

仓库已包含运行所需的场景和网格，无需下载 SDK 或 XLeRobot。

重新生成机械臂场景时，将 [Unitree SDK](https://github.com/unitreerobotics/r1-7a_sdk) 克隆到本仓库旁的 `../r1-7a_sdk`，再运行 `python build_scene.py`。

重新提取夹爪网格时，准备 `assets/gripper/SOURCE.md` 中指定版本的 XLeRobot，然后运行：

```bash
python prepare_gripper.py --source /path/to/XLeRobot
```

## 许可与来源

独立仿真核心采用 [BSD-3-Clause](LICENSE) 许可；可选 Ultralytics 集成及组合运行的 YOLO 应用按 [AGPL-3.0-or-later](yolo_grasp/LICENSE) 发布。另行下载的权重遵循上游许可，仓库不分发权重。见 [第三方许可说明](THIRD_PARTY_LICENSES.md)。宇树机械臂网格来自 Unitree SDK，保留其 [BSD-3-Clause 许可](assets/SDK_LICENSE)。XLeRobot 夹爪网格采用 [Apache-2.0](assets/gripper/LICENSE)，具体来源和变换记录见 [SOURCE.md](assets/gripper/SOURCE.md)。本项目为社区仿真项目。
