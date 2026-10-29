# R1-7A YOLO RGB-D 自动抓取

[English](README.md) | **简体中文**

默认目标为从 [Poly Haven 社区](https://polyhaven.com/a/potted_plant_04) 下载的带纹理植物盆栽（CC0），花盆缩放至60 mm 直径。参考 reBot-DevArm-Grasp 的普通抓取流程，在当前 MuJoCo 环境中实现：**腕部 D435 RGB-D → YOLO 检测/分割 → 深度反投影 → 盆沿圆拟合定位花盆 → 自动抓取、搬运与放置**。暂不接入 GraspNet，原有键盘仿真可独立运行。

![YOLO 检测与抓取中心](preview_detection.png)

![根据视觉定位完成物理抬升](preview_lift.png)

## 安装与启动

在仓库根目录，激活原来运行仿真的 Python 环境：

```bash
conda activate le312
python -m pip install -r yolo_grasp/requirements.txt
./yolo_grasp/run.sh
```

| 按键 | 功能 |
| --- | --- |
| G | 多观察位扫描，检测后停稳复测，再抓取放置 |
| V | 仅检测与显示抓取估计，不执行动作 |
| R | 取消任务、重置并回到腕部相机观察位姿 |
| P | 暂停 / 继续 |
| Esc | 退出 |

鼠标旋转、平移、缩放与原仿真相同。空闲时可使用原来的手动控制；自动抓取过程中屏蔽手动位移和夹爪开合。右下角显示随腕部运动的实时 RGB 与对齐深度（5 Hz），另一个小图显示最近一次 YOLO 检测结果。G 会多观察位扫描并停稳复测；V 只在当前位置拍照，不启动扫描。R 回到固定工作区观察位姿，TCP 约 `(0.12, 0.08, 0.83)` m，腕部绕世界 X 轴倾斜 40°，从斜上方观察花盆。

自动开始、无窗口抓取、仅检测：

```bash
./yolo_grasp/run.sh --auto
./yolo_grasp/run.sh --headless
./yolo_grasp/run.sh --headless --detect-only
```

`--headless` 仍需要渲染 RGB-D，因此需要可用的 EGL / OpenGL 环境；它与原项目纯物理的无界面演示不同。默认 CPU 推理，兼容的 CUDA 环境可加 `--device 0`。脚本支持 `PYTHON=/path/to/python` 指定解释器，也可从其他工作目录调用。

## 模型

默认权重为 `yolo_grasp/models/yoloe-26s-seg.pt`，缺失时由 Ultralytics 下载。首次文本提示还会下载约 254 MB 的 MobileCLIP2 文本编码器，可能自动安装 Ultralytics CLIP 分词依赖，需要联网。权重和编码器保存在 `yolo_grasp/models/`，Git 已忽略此目录。

模型权重和文本编码器不随仓库分发。可以通过 `--weights /实际路径/yoloe-26s-seg.pt` 指定已有权重；相对路径以运行命令的当前目录为基准。

默认提示词为 `potted plant`、`plant` 和 `flower pot`，置信度阈值为 **0.20**：

```bash
./yolo_grasp/run.sh --classes 'potted plant' 'plant' 'flower pot' --conf 0.20
```

零样本置信度会受光照、视角和目标位置影响。扫描耗尽仍无有效定位时停止，不会退回到读取仿真方块真实坐标。也支持 YOLO-World 和普通自训练 YOLO 检测/分割模型；自训练模型要用 `--classes` 指定训练时的真实类别名。修改类别名并不代表能抓取任意物体；盆栽规划器专门筛选浅色陶瓷盆体和圆形盆沿。

## D435 腕部安装

![夹爪腕部 D435 安装效果](preview_mount.png)

布局参考 [TheRobotStudio 社区 D435/D435i 腕部支架](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D435)：相机横向安装在固定腕部/夹爪上方，镜头朝向夹取区域。社区方案使用 5 颗腕部小螺丝及 2 颗 M3×6 相机螺丝。这里为 TPU 夹爪重新搭建了简化支架几何，不分发原 STL，也没有验证打印件的真实机械适配。

`wrist_mount.py` 调用根目录共用构建器 `wrist_scene.py`，默认生成 `../assets/scene_shared_plant.xml`；键盘入口也使用同一文件。相对 `hand` 坐标的仿真参数如下：

| 参数 | 仿真设置 |
| --- | --- |
| 外壳 | 90 × 25 × 25 mm |
| 外壳中心 | `(0, -0.050, 0.018)` m |
| 朝手指方向倾角 | 绕局部 −X 轴 15° |
| RGB 光心相对外壳 | `(0.035, 0, 0.014)` m |
| 相机与支架质量 | 0.090 kg，近似值 |
| 对齐 RGB-D 输出 | 640 × 480 |
| RGB 投影 | 理想化 4:3 中心裁剪，垂直视场角 42.5° |
| 有效光轴深度 | 0.175–3.0 m，超出范围置零 |

175 mm 最近深度来自 [D400 数据手册](https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf) 中的 D435 640×480 档位；外壳尺寸来自[厂商规格](https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html)。这里的光学内参、安装偏移、倾角及支架尺寸都是仿真近似，不是出厂标定或手眼标定。真实 D435 的 RGB 与深度有各自的内外参；本模块输出理想化、已对齐到裁剪 RGB 投影的深度，不模拟双目匹配、畸变、投射器纹理、噪声或 USB 采集。

先在观察位姿拍照，目标距离高于深度盲区；随后将抓取位姿保存到世界坐标执行。下降时近物体可能变成黑色无效深度，机械臂继续执行已估计的抓取位姿。腕部运动会改变相机画面，旋转外部观察视角不会改变传感器。工作区限制只是候选目标筛选范围，不代表单个腕部位姿能看到整张桌面。

详细来源与坐标说明见 [CAMERA_SOURCE.md](CAMERA_SOURCE.md)。

## 实现方式

1. 固定在 SO101 腕部的 D435 仿真相机输出对齐的 RGB 与深度，深度单位为米，表示光轴距离。内参由所选 RGB 投影计算，每次采集都根据当前腕部/相机位姿更新外参，完成 OpenGL 到 OpenCV 右、下、前坐标转换。
2. YOLOE 只对 RGB 推理，获得检测框和实例掩膜；普通检测模型使用框区域。
3. 掩膜腐蚀后筛选有效深度，反投影到世界坐标。桌面标定平面为 Z=0.60 m，只保留 Z 在 `(0.608, 0.70)` m 的物体表面点。
4. 盆栽模式在 YOLO 掩膜内部用浅色陶瓷及高度条件筛选花盆，再对顶部盆沿深度点拟合世界 XY 圆，估计中心和直径，避免斜视下可见前半盆体造成的中心偏移。圆形花盆使用固定开合朝向；TCP 默认位于估计盆口下方 0.020 m（方块为 0.014 m），并限制不低于 Z=0.622 m。方块模式保留最小矩形短轴方法。
5. 复用原有 IK 与受限力伺服，状态机执行对齐、预抓取、下降、合拢、抬升、搬运、放置、松开、退回，并检查位姿跟踪及双侧接触。

目标坐标来自 RGB-D，规划不读取 `cube_position`、仿真分割 ID 或写死的物体 XY。真实物体状态和接触 ID 仅用于判断动作是否成功。目标始终为自由刚体，抓取流程不修改物体位置，不使用焊接或吸附；验证脚本仅在布置初始测试场景时改变目标位置。

拾取区域为 X `[-0.22, 0.24]`、Y `[0.12, 0.38]` m；允许短边宽度 0.015–0.065 m，长边不超过 0.12 m。无检测、无有效深度、不可达或接触失败都会停止；失败后保持当前控制目标，按 R 可重置。此流程没有避障规划，也不是实物安全控制器。

当前放置高度和成功判定已验证微型盆栽及原 4 cm 方块。盆栽采用花盆凸包碰撞，土壤和叶片只有外观、不参与碰撞，不模拟叶片形变或损伤。下载来源、缩放和物理近似见 [模型来源](assets/plant04/SOURCE.md)。适用于静止桌面物体的俯视抓取，未实现任意 6D 抓取、运动目标跟踪、堆叠物体处理、完整 TPU 弹性或真实相机标定；无需 GraspNet 和真实机器人/相机驱动。

## 速度与目标

默认 `--speed 0.5`：移动速度约为原流程一半，抓放动作约 44 秒仿真时间（另加搜索与停稳时间）；夹爪角速度由共用配置固定为 0.30 rad/s；`--speed` 只改变任务时长，开合等待至少覆盖完整行程。可进一步减慢：

```bash
./yolo_grasp/run.sh --speed 0.3
./yolo_grasp/run.sh --object cube --speed 0.5
```

速度范围 0.1–1.0。G 开始完整任务，V 仅查看识别。图中的黄色轮廓是盆沿拟合，红色标记是拟合中心的相机投影。重启进程后新模型和速度设置才生效。


## 共用运动控制与物理仿真

本入口和键盘遥操作共用根目录的 `environment.py`、`wrist_scene.py`、`motion.py`、`motor_config.json` 和 `end_effector.json`。默认场景、盆栽、初始/复位姿态、碰撞、质量、夹爪角速度、电机增益、重力前馈及 SDK 指令量化完全一致。

默认 `kp=60`，`kd=[6,6,4,4,2,2,1.5]`，100 Hz 发令、500 Hz 仿真内环。此前独立的高增益配置已取消；本目录的 `motor_config.json` 是指向根目录配置的兼容链接。`wrist_mount.py` 和 `controller.py` 也只转发到共用实现。YOLO 只提供感知抓取位姿，不覆盖电机或物理参数。

用 `--motor-config`、`--end-effector-config` 指定自定义共用参数，用 `--record /新路径/trace.csv` 记录命令与响应。完整说明见 [共用控制说明](../SHARED_CONTROL.zh-CN.md)。

## 输出和验证

默认输出到 `yolo_grasp/outputs/latest/`，可用 `--output /路径/任务名` 保留独立结果。

| 文件 | 内容 |
| --- | --- |
| `rgb.png` | 原始 RGB |
| `detection.png` | 检测框、拟合矩形和抓取中心 |
| `rgbd.npz` | 米制深度、内参 K、相机到世界变换 |
| `grasp.json` | 检测类别、置信度、世界坐标 TCP 位姿及估计宽度 |
| `lift.png`、`placed.png` | 实际抬升与最终放置截图 |
| `result.json` | 抓取验证结果或失败原因 |

运行端到端验证：

```bash
python yolo_grasp/verify.py --conf 0.15
```

三位置测试使用 `--conf 0.15`，其中一个位置的检测置信度约 0.18；应用默认阈值仍为 0.20。使用真实 YOLO 推理、RGB-D 渲染和接触动力学，对三个目标位置测试定位和抓取，还检查相机随腕部运动、手眼刚体变换保持不变、D435 近距离深度无效，以及无检测、深度无效、夹空时是否停止。结果写入 `yolo_grasp/outputs/verification/verification.json`，失败返回非零退出码。原回归测试仍可用 `python verify.py` 运行。

有桌面环境时，可运行 `python yolo_grasp/verify.py --gui`，额外检查真实窗口、检测小图、G/V/R/P 回调、完整抓取及截图、重置和退出清理。如终端已导出其他渲染后端，请用 `MUJOCO_GL=glfw` 运行该窗口检查。

## 文件与来源

- `camera.py`：腕部 RGB-D 相机、当前光心外参及反投影。
- `wrist_mount.py`：共用场景构建器的兼容入口。
- `detector.py`：YOLOE / YOLO-World / 自训练 YOLO 加载与检测结果解析。
- `planner.py`：深度筛选、最小矩形与俯视抓取位姿估计。
- `controller.py`：根目录共用抓取状态机的兼容入口。
- `motor_config.json`：指向根目录唯一默认配置的链接。
- `main.py`：命令行、GLFW 交互窗口和检测小图。
- `verify.py`：端到端集成验证及失败场景。

设计参考 [reBot-DevArm-Grasp](https://github.com/EclipseaHime017/reBot-DevArm-Grasp) 的 `utils/yolo_utils.py`、`utils/ordinary_grasp.py` 和 `scripts/ordinary_grasp_pipeline.py`。针对 R1 仿真重新实现了相机、TCP 朝向和运动执行，没有引入 reBot 硬件驱动或标定文件。

Ultralytics 及模型遵循其自身的[许可条款](https://docs.ultralytics.com/help/FAQ/#ultralytics-license)，本仓库不分发第三方权重。接口参考：[YOLOE 文档](https://docs.ultralytics.com/models/yoloe/)、[MuJoCo Python 文档](https://mujoco.readthedocs.io/en/stable/python.html)。

## 随机摆放与扫描

```bash
./yolo_grasp/run.sh --auto --randomize --seed 42
./yolo_grasp/run.sh --headless --randomize --seed 42
python yolo_grasp/verify.py
python yolo_grasp/verify_search.py
```

`--randomize` 在复位时将盆栽中心摆在 X `[0.10,0.18]`、Y `[0.25,0.31]` m 内，
并随机旋转盆栽。G 从六个观察位依次寻找目标：TCP 高 0.86 m，X 为
0.12/0.04/0.20 m，Y 为 0.08/0.16 m，沿用 40° 斜视姿态。
每个观察位先停稳再检测；发现目标后继续保持稳定至少 0.5 s，重新采集 RGB、深度和当前外参定位。
停稳判据为关节速度小于 0.025 rad/s、末端位置误差小于 3 mm、姿态误差小于 0.03 rad，
连续满足 0.5 s。搜索帧只决定是否发现目标，抓取坐标来自第二次采集。
定位无效则换观察位；全部扫描结束仍无有效目标则失败，不会沿用上次坐标。
`result.json` 的 `search` 字段记录观察位、检测/定位事件及仿真时间。

随机区域是本次验证范围；规划器较宽的候选坐标边界不表示所有位置都能看到或成功抓取。

本次回归：六组目标位置（含随机朝向）均完成检测、抓取、搬运和放置，RGB-D 定位误差约 0.66–1.81 mm；其中两组在定位被拒绝后继续扫描，分别在第 2 和第 5 个观察位成功。另验证空目标、无效深度、无接触拒绝以及两入口共享动力学。

## 许可

本可选 Ultralytics 集成及组合运行的 YOLO 应用按 [AGPL-3.0-or-later](LICENSE) 发布；独立仿真核心保留 BSD-3-Clause。另行下载的权重和依赖遵循各自的上游许可。见 [第三方许可说明](../THIRD_PARTY_LICENSES.md)。
