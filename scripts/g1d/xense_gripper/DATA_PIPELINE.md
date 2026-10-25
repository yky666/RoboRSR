# 四路触觉数据链路

通道顺序固定为 left_positive、left_negative、right_positive、right_negative；不能把四路 WS 当成旧版 Dex1 的左右腕 RGB。控制仍为左臂7+左开度+右臂7+右开度，16维。

1. `full_task_runtime.py` 在执行下一动作前保存机器人状态、动作目标、实际胶面和物体位姿、PhysX 接触力，并在同一状态采三路审阅图；JSONL 每行完整写入。
2. 独立 Python 3.10 `tactile_worker.py --follow` 消费完整行，校验厂家 NPZ，通过 SimpleSensor/FemSensor 输出四路 RGB、深度、Marker、节点力与原生接触力。原子发布 NPZ，以 frame_id 和物理时间关联。进程退出前要求生产方完成凭据。未命中为正间隙，不重置胶体或伪造零图。
3. `XenseTactileBridge.wait_observation()` 支持有限时等待；缺帧、错帧、旧时间或非有限值拒绝。`policy_observation()` 输出四个显式图像键和16维状态，图像为 CHW float32 [0,1]。这是输入适配，不代表已有策略权重理解触觉。
4. `tactile_dataset.py` 保存无损 HDF5：绝对物理时间、状态/动作、四路 Simple/FEM RGB、深度、节点力、Marker、原生力。使用官方 LeRobot 0.4.4 的 Dataset API 导出 v3：四路 FEM AV1 视频、16维状态/动作和12维原生法向力；LeRobot 时间从回合首帧起算。完整数值留在 HDF5，避免误把 FEM 单位解释为 N。
5. `--audit-only` 逐字段读回 HDF5；`verify_policy_dataset.py` 用实际 LeRobotDataset 读取首、中、末等帧、核对状态动作、图像范围/误差，并完整解码四段视频。

此为显式 `g1d_four_pad_episode.v1` 试验格式，10 Hz 录制、240 Hz 物理。它不是仓库原有三路 RGB、30 Hz、XPolicyLab 转换器产生的数据集。没有修改旧转换器或子模块，也没有上传数据、训练或重载已有策略。物理仿真可在触觉进程之后按同帧等待形成锁步；本次完整任务使用脚本动作，触觉为单向观测，并未声称实时触觉闭环控制。

运行时：Isaac Sim 5.1；独立 CPython3.10 + 已有 Xensesim 分发包。LeRobot 导出需独立 Python3.10 环境安装 `lerobot==0.4.4 h5py==3.16.0`；审计/视频使用 ffmpeg。厂家资源摘要和本机路径见 `real2sim/grippers/aurora_fusion/private_runtime_manifest.json`，可用 `runtime_resources.py` 全量复核。禁止用最新 XenseSDK 2.x 替换旧仿真分发包的1.6.8运行时后声称兼容。
