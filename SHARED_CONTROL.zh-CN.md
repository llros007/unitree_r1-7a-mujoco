# 共用场景和运动控制

键盘遥操作和 YOLO 抓取默认使用同一个盆栽场景、XLeRobot/SO101 夹爪、D435 腕部相机、
末端负载、初始观察姿态和复位姿态。盆栽使用下载的 `potted_plant_04_4k.blend.zip`（Poly Haven CC0）：
花盆直径约 60 mm、高约 55.4 mm，质量估计为 80 g；叶片和土壤仅作外观。

| 共用文件 | 职责 |
| --- | --- |
| `wrist_scene.py` | 场景、盆栽、相机、末端质量和惯量；两入口生成同一 `assets/scene_shared_plant.xml` |
| `environment.py` | 同一个 MuJoCo 模型、IK、重力前馈、指令发送与 PD 内环 |
| `motor_config.json` | 唯一的默认电机/控制配置；`yolo_grasp/motor_config.json` 是指向它的兼容链接 |
| `end_effector.json` | 唯一的末端质量、质心及可选惯量配置 |
| `motion.py` | 共用抓放状态机，键盘 `--demo` 和 YOLO 感知抓取都调用同一个 `PickPlace` |
| `recording.py` | 两入口共用的命令和状态记录器 |

`yolo_grasp/wrist_mount.py`、`yolo_grasp/controller.py` 只保留兼容入口，不再维护第二套实现。
YOLO 模块负责图像、检测和抓取位姿估计；不再覆盖物理参数或加载更高的默认 PD 增益。

## 默认参数

- 七关节 `kp=60`，`kd=[6,6,4,4,2,2,1.5]`。
- 重力前馈比例 1.0、100 Hz 主机发令、SDK 标定转换及指令量化、500 Hz 仿真 PD 内环。
- 夹爪目标角速度固定为 0.30 rad/s，由 `gripper_speed_rad_s` 控制。
- 共享自动抓取动作按花盆半径修正固定指位置，再向固定指外侧留 6 mm 进场间隙，由 `grasp_approach_clearance_m` 控制。
  这避免视觉中心存在毫米级偏差时固定指先撞上盆沿；夹紧时由活动指与接触动力学推动物体。
  该偏移来自估计抓取姿态，不读取真实物体坐标来修正视觉结果；实际执行 TCP 写入结果报告。
- `--speed` 只调整自动任务的动作时长，不改变物理参数、PD 或夹爪角速度。
  开合等待时间始终覆盖夹爪行程与稳定时间。

相机观察位姿为 TCP `(0.12,0.08,0.83)` m、朝下基准绕世界 X 轴旋转 40°。
初始 IK 在多个可行解中选择关节余量较大的解，避免一启动就贴着腕部限位。
手动夹取时先将夹爪转正并移至盆体上方，给固定指侧留间隙，再下降与闭合。

## 入口与验证

```bash
./run.sh                                  # 键盘遥操作，默认盆栽
./run.sh --headless --demo                 # 已知工位目标，调用共用状态机
./yolo_grasp/run.sh                        # 同一环境，加 YOLO 和 RGB-D
./yolo_grasp/run.sh --headless             # 感知驱动完整抓取
./run.sh --object cube                     # 可选原方块，两入口都支持
./run.sh --record outputs/manual.csv
./yolo_grasp/run.sh --record outputs/yolo.csv
```

两入口均可用 `--motor-config` 和 `--end-effector-config` 指定同一份自定义配置。
键盘入口不需要安装 YOLO 或加载模型权重，场景构建只使用基础依赖。

```bash
python verify_shared.py                    # 两入口模型、配置和100步响应逐值一致
python verify.py                           # 五参数、键盘夹取、停稳和共用状态机
python verify_sim2real.py                   # SDK 编码、日志、模型误差检查
python yolo_grasp/verify.py --conf 0.15    # 实际YOLO/RGB-D、多位置及失败情形
```

这些测试证明代码与仿真行为共用，并不证明未测量的质量、摩擦或相机标定已与真机一致。
三位置感知验证使用 `--conf 0.15`；第三个位置的有效盆栽检测置信度约 0.18，应用默认
阈值仍为 0.20，低于阈值时会停止。可在 YOLO 启动命令中同样传入 `--conf 0.15`。

花盆外径按导出网格验证为 60 mm。共享 MuJoCo 配置启用 5 次 `noslip_iterations` 摩擦收敛迭代，降低软接触求解的数值蠕滑；仍使用自由刚体及摩擦接触。盆栽默认抓取深度为盆口下 20 mm，放置检测到花盆触桌后松爪。YOLO 搜索层使用 `yolo_grasp/search.py`，定位成功后调用同一个 `motion.PickPlace`。
