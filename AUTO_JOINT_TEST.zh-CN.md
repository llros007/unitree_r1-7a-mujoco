# 自动关节反馈测试

`real_keyboard.py` 支持 `--auto-ik`。它以启动姿态为起点，保持工具方向，沿 X、Y、Z 分别做一个
5 mm 往返，每段移动 3 秒、保持 4 秒。每轴先检查正方向，再尝试负方向，跳过不满足 IK、关节限位或
工具间隙检查的路径。可用 `--ik-distance-mm` 选择 1 至 15 mm，用 `--ik-output` 指定新输出目录。
位移超过 5 mm 时，移动时间随位移等比例增加。

```bash
python real_keyboard.py --mode sync --hardware --port /dev/ttyUSB0 --auto-ik \
  --motor-config motor_config.real_tuned.json
```

窗口左侧由编码器角度计算姿态，右侧用独立动力学仿真执行同一组电机指令。Esc 可以中止。
报告记录 IK 指令的正运动学残差、编码器推算 TCP 与目标的误差，以及编码器推算 TCP 与仿真的差值。
默认报告位于 `outputs/ik_test_*/comparison.html`，原始采样为同目录的 `trace.csv`。
纯仿真预演使用 `--mode sim --auto-ik`。

加 `--ik-highest` 可测试从当前姿态沿 Z 轴上升，再沿原路返回。路径保持起点的 XY 坐标和工具方向，
规划高度受工作区上限、IK 可达性、模型间隙及单轴相对起点 60° 行程约束。报告记录实际采用的高度和轨迹。
真机入口为 `python real_keyboard.py --mode sync --hardware --port /dev/ttyUSB0 --ik-highest --motor-config motor_config.real_tuned.json`。

`auto_joint_test.py` 无需逐点按键。默认无窗口且仅运行仿真，显式添加 `--hardware` 才连接串口。
它测量指令与编码器反馈的差异，不测真实关节角、真实 TCP、手眼标定或抓放精度。

## 离线运行

```bash
.venv/bin/python auto_joint_test.py
# 更短的功能检查
.venv/bin/python auto_joint_test.py --cycles 1 --move-seconds 1 --hold-seconds .5
.venv/bin/python verify_auto_joint_test.py
# 带左右对照窗口的离线运行，不连接硬件
.venv/bin/python auto_joint_test.py --view --cycles 1
```

仿真从 J4=45° 的展开姿态开始；默认只测 J4、幅度 ±1°、3 轮。
每轮依次为起点→正向目标→起点→反向目标→起点，每段移动 2 秒、保持 2 秒。
默认运动与保持约 48 秒，另有建模与启动时间；离线仿真不按墙钟时间等待。

## 真机运行

先确认固定底座、标定、末端安装、负载、控制参数和周围空间适合测试，并把机械臂放到
具有双向余量的姿态。脚本不会自动从零位展开；默认 J4 测试在 J4 零位会因负向目标越界而拒绝。
配置中的 PD 和重力补偿来自项目配置，不是已验证的硬件推荐参数。首次真机测试应现场监护。

```bash
.venv/bin/python auto_joint_test.py --hardware --port /dev/ttyUSB3 \
  --calib r1-7a_sdk/config/joint_calib.txt \
  --joints 4 --amplitude-deg 1 --cycles 3 \
  --output outputs/hardware_j4_test
```

## 真机与独立仿真同步对照

```bash
.venv/bin/python auto_joint_test.py --hardware --port /dev/ttyUSB0 \
  --joints 3 --view --output outputs/hardware_j3_compare
```

`--view` 自动启用独立仿真。左侧用真机编码器反馈驱动模型，仅表示模型正运动学姿态；右侧是
独立动力学仿真，从启动实测角度/速度初始化一次，之后执行相同的五参数命令，绝不逐周期注入实测姿态。
两侧共享观察角度，显示七关节目标、反馈、仿真角度及反馈减仿真的差值。
无窗口比较可用 `--sim-compare`；无 `--hardware` 的窗口测试按实时节拍运行。

真机对照按主机发送时间切换仿真命令，按主机反馈接收时间推进独立仿真，使用固定物理步与指令保持；
模拟时间与主机时间之差小于一个物理步（默认2ms）。驱动/总线内部时延未知，因此这是主机时间对齐，
不等于各电机传感器的硬件同步。仿真不参与修改真机目标或判断真机保护阈值。

渲染与控制采样分线程，渲染读取副本；窗口超过0.5秒无心跳会中止真机测试。
Esc、关闭窗口，或终端输入 `q` 后回车均取消测试并停驱动。正常测试完成后驱动已停止，
窗口保留最后一帧供观察，再按 Esc 关闭。测试窗口只供观察，不接受手动运动按键。
需要全自动结束窗口可添加 `--close-on-finish`。

完成后打开输出目录的 `comparison.html`，查看目标/反馈/独立仿真角度曲线；
`analysis.json` 包含真机减仿真的误差统计和分方向返回点统计。
较大的模型差异可能来自未辨识参数、安装/负载、初始状态或时延，不能直接归因于电机误差。

旧日志也可离线重分析，不会打开串口，不覆盖原始 `report.json` 和 `trace.csv`：

```bash
.venv/bin/python auto_joint_test.py --analyze outputs/hardware_j3_test
```

旧日志没有独立仿真列时，曲线只显示目标和反馈，不补造历史仿真结果。

确认所有关节的实物安装与运动空间后，可用 `--joints 1 2 3 4 5 6 7` 逐关节测试。
每次从启动实测姿态做局部往返，其余关节保持同一目标。输出目录必须不存在，避免覆盖结果。

需要更明显的转动，可用 `--amplitude-deg 5 --move-seconds 3 --hold-seconds 1 --cycles 1`。
默认 `--direction both` 要求两个方向均可达；也可选 `positive` / `negative`。
`--direction auto` 会明确打印并跳过被关节限位或工具间隙挡住的方向，不缩小5°幅度；
每个被选关节必须至少有一个方向通过检查，否则整次计划拒绝执行。
例如零位附近的 J2、J4 通常只允许正向5°再返回，不能越过其零位下限。

启动先检查全部目标和采样路径的模型限位及工具/臂体间隙，再保持起点，连续 20 周期 FOC 后
开始轨迹。插值采用五次曲线，限制计划峰值速度 5°/s、峰值加速度 10°/s²，幅度上限 5°。
路径检查只覆盖现有工具/臂体碰撞模型，不覆盖真实环境或完整全臂碰撞。

反馈错误、退出 FOC、超时、过大跟踪误差、模型限位越界（反馈容差 0.5°）、工具/臂体间隙异常、
估算总力矩超限会中止。没有补发追赶；调度落后超过 100 ms 会停止。Python/Linux 不保证硬实时，
估算总力矩也不替代驱动保护。正常结束、异常、Ctrl-C 和 SIGTERM 均尝试停驱动；停驱动后可能失去
主动支撑，需要合适的支撑/防坠安排。SIGKILL、掉电等不能依赖 Python 清理。
完成准备后的采样过程可自动执行，这些软件检查不能据此证明适合无人值守。

## 输出与判定

- `trace.csv`：每周期原始主机五参数、七关节反馈角/速度/力矩、状态、错误码；角度单位 rad。
  仿真力矩列是模拟施加力矩，真机力矩列是总线回传值，不视为独立测力。
  五参数为发送给桥接器的值，底层 SDK 仍会编码量化。
  启用比较时附加 `q_sim_rad_J*`、`dq_sim_rad_s_J*` 和 `sim_time_s`。
- `report.json`：数据来源、运行状态、参数、模型与标定哈希、标定内容、逐段统计和返回点反馈散布。
  统计角度单位为度，包含逐关节 RMSE、最大绝对误差、末尾窗口偏差与标准差。
- 稳态窗口为保持末尾至多 0.5 秒（保持期的一半）。逐关节分别报告位置、原始速度和联合判定；
  停稳要求同时满足角度误差与速度阈值，并一直保持至该段结束、至少持续 0.2 秒；未满足返回 `null`。
  `passed` / `settling_seconds` 针对该段被测关节；其他关节保持单独报告。
  整次运行仍要求被测与保持关节都通过，才返回 `passed` 和退出码0。
- `analysis.json` / `comparison.html`：分段指出位置不合格和原始速度不合格的具体关节；
  速度跳值不做隐式平滑或放宽阈值。返回点按被测关节与接近方向分别统计，并报告两方向平均位置之差。
  启动失败、没有完成运动段时标为 `NOT TESTED`，不计算空数组的精度或仿真误差。
  `acquisition` 记录实际主机发令频率及间隔分布；配置100Hz不保证实际达到100Hz，窗口和仿真负载会影响节拍。
- 默认阈值为 0.5°、0.5°/s，可用 `--tolerance-deg` 和 `--velocity-tolerance-deg-s` 修改。
  这是本次工程验收条件，不是厂家精度规格。退出码：通过为 0，超阈值或异常为 1。
- 返回点标准差基于各次返回的末尾平均编码器读数，不等于外部测量的机械重复定位精度。
  `report.json` 的历史兼容字段 `return_feedback_std_deg` 混合接近方向；分析时优先用
  `analysis.json` 的 `directional_returns`，不把方向偏差混入同方向散布。

真机参考轨迹按实际墙钟推进，CSV 的 `reference_time_s` 是当前段参考时间；发送/接收时间
相对整次运行开始。它们是主机时间，不是各电机同步采样时刻，不能据此精确辨识驱动内部时延。
仿真参考时间按物理步推进，发送/接收时间只反映计算耗时。

要测末端毫米级绝对精度或重复定位，需要增加独立外部测量设备及其标定、同步和测量不确定度评估。
