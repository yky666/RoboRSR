# G1-D + Aurora Lite / G1-WS · 融合仿真版

新增本体 ID：`g1d_aurora_lite_ws`。交付的是完整桌面版 G1-D，双腕各安装一个极光 Lite，每个夹爪内有两个官方装配匹配的 G1-WS。京东传感器、极光 Lite 产品页和用户 STEP 是同一系统的参考资料，不建成三个重复本体。

2026-10-24 已增量接入本仓库，文件归属、来源清单、本次检查及未复测范围见
[仓库交付记录](INTEGRATION.md)。不要再次用旧基线覆盖包覆盖当前仓库。

用户已同意由官方 CAD 生成 URDF，并明确标注待确认参数。这不是厂家发布的 URDF，也不是已完成真机标定的模型。

## 2026-10-24 补齐范围

本轮增加**带重力自由物体的双手拾取、抬升、搬运、放置、松爪和撤离**，不使用物体绑定、复位或额外托举力。原生记录、四路触觉、三路审阅视频、HDF5、LeRobot v3 和实际策略输入读回均有对应证据。变体与失败保留，不能将一次成功扩展为通用抓取成功率。最新结果见 `validation/task_campaign.json` 和交付包 `evidence/full_task/`。

装配与标定资料已重新审计：找到另一台 G1-D 的相机/零位辨识样例，但“我们这台”的记录明确尚未求参，故不套用。厂家通用法兰没有明确的 G1-D 适配依据。当前腕端孔位、转接板、安装实测/TCP、当前硬件零位/手眼/动力学/材料响应仍待测，见本体 `calibration/README.md`。公开 CAD 与厂家通用 FEM 不能替代这些数据。

本机厂家原包和 9,351 个运行资源已核对摘要；已验证 CPython3.10.20 的独立进程。`private_runtime_manifest.json` 支持原目录复核或从原 ZIP 私有安装。资源不是缺失，而是按厂家许可与可公开 Git 文件分开；四只真实 SN 的标定原件仍未取得。不要把本轮成果称为真机校准终版。

完整任务入口（先按前文安装本体，输出目录必须不存在）：

```bash
python scripts/g1d/xense_gripper/make_full_task_plan.py --out output/aurora_plan.json
PYTHONPATH=. python scripts/g1d/xense_gripper/full_task_runtime.py \
  --plan output/aurora_plan.json --out output/aurora_task --device cpu --fast-exit
python scripts/g1d/xense_gripper/audit_full_task.py output/aurora_task --out output/task_audit.json
```

物理为240Hz、录制10Hz。`--block-width` / `--mass` / `--friction` / `--grip-overtravel` 是显式候选参数。审阅相机是仿真外部观察视角，不是真实腕相机。主引擎胶面刚体、单向 FEM 观测方案不变；未验证双向可变形耦合或布料实物对应。低摩擦失持、放置翻转均按实际记录报告。

### 完整任务结果

| 回合 | 宽度 / 质量 / 静摩擦 | 结果 |
| --- | --- | --- |
| 最终默认10 | 30mm / 20g / 0.5；1mm总预紧 | 双手完整通过，最大抬升约120mm，最终竖直放稳；四路触觉154帧 |
| 修正变体09 | 45mm / 50g / 0.3；2mm总预紧 | 双手完整通过，最大抬升约119mm，落点最大平面偏差1.35mm；四路触觉154帧 |
| 边界07 | 35mm / 80g / 0.05；源/目标x偏移10mm | 双手滑落，未通过；完整四路触觉仍保留 |
| 旧轨迹06 | 45mm / 50g / 0.3 | 物理放置完成，但触觉frame15超过已测1mm压入域；不通过整体验收 |
| 轨迹修正08 | 同06；改逐点笛卡尔轨迹、1mm预紧 | 四路触觉全程有效，但最终翻倒，严格放置不通过 |

原05回合保留为初始完整任务参照；以上每条都有独立连续状态来源。判据：抬升>60mm、平移>60mm、目标平面半径25mm、底面距桌面<3mm、最后10帧速度<0.03m/s、开度>80mm、最终物体+Z朝上10°内。底面高度按实际四元数与尺寸计算，不能用固定半高误判翻转。单次仿真数值不代表实物精度或统计成功率。

09的无损HDF5和四路LeRobot视频随新版完整包交付；05也已独立导出读回并保留在工作目录。两者各154帧，状态/动作16维；四视频完整解码，实际Dataset抽取首、中、末等5帧核对。未训练或验证触觉策略权重。

## 来源与对齐

| 来源 | 本次用途 |
| --- | --- |
| [官方夹爪模型页](https://docs.xenserobotics.com/docs/latest/hardware/gripper-models/)及其中的 [Aurora Lite STEP](https://docs.xenserobotics.com/assets/models/aurora-lite-gripper.STEP) | 完整装配、零件坐标、滑轨、滑块、WS 外壳及硅胶面 |
| [极光 Lite 产品页](https://www.xenserobotics.com/product/385/detail/11) | 85 mm 开度、615 g 总质量、每指 40 N、140 mm/s 产品规格 |
| [官方传感器模型页](https://docs.xenserobotics.com/docs/latest/hardware/sensor-models/) | 单独 WS 模型作为交叉参考 |
| [夹爪 SDK](https://docs.xenserobotics.com/docs/latest/sdk/xensegripper/api/) | 0 mm 闭合、85 mm 全开；SDK 指令上限不作为额定硬件指标 |
| [触觉 SDK](https://docs.xenserobotics.com/docs/latest/sdk/xensesdk/api/) | 触觉数据种类参考，未连接 SDK 或硬件 |
| 用户 `g1-ws-带螺丝孔位.step` | 检查不同 WS 修订的几何；未替换官方夹爪装配中的匹配零件 |
| `real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf` | 完整 G1-D 基体，保留原非 Dex1 关节及质量惯量，轮子锁为 fixed |

源文件摘要见 `real2sim/grippers/source_manifest.json`；生成结果逐文件摘要、部件归属和参数依据见 `real2sim/robot/g1d_aurora_lite_ws/{sha256,model_contract}.json`。

官方 STEP 有 56 个顶层装配实例，原生状态的两个硅胶内表面间距为 85 mm。拆分固定本体、正负滑块承载组、传感器外壳和硅胶面，保留供应商坐标，再将后端底座平面移为安装原点。STL 单位为米。视觉网格来自 CAD 曲面；齿条按 4 mm 轴向 CAD 分区分别取凸包，其余承载件按原零件取凸包，外壳和硅胶面分别取凸包。这保留弯折支架的凹区，避免虚假的凸包挡住闭合；仍不能代替精确接触或软体模型。

## 文件与使用

- 完整机器人：`real2sim/robot/g1d_aurora_lite_ws/g1d_aurora_lite_ws.urdf`。
- 单独夹爪（仅供检查）：同目录 `aurora_lite_ws.urdf`。
- 本地管理登记：`real2sim/embodiments/g1d_aurora_lite_ws/registration.json`。
- 仿真绑定：`env/robot_manager/{robot_class,robot_config}/g1d_aurora_lite_ws.py`、`env_cfg/g1d_aurora_lite_ws.yml`。

先在独立副本中安装，避免写共享资产库：

```bash
python scripts/g1d/xense_gripper/install_asset.py --assets ./Assets
python scripts/g1d/xense_gripper/audit_model.py real2sim/robot/g1d_aurora_lite_ws \
  --base-urdf real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf --output model_audit.json
```

安装器先校验摘要，已有同名目标会拒绝覆盖。登记 JSON 需导入所选本地数据库：`python scripts/g1d/xense_gripper/register_local.py --db .cache/real2sim_platform.sqlite3 --assets Assets`，再启动平台或点击同步。已有用户填写的登记不会被覆盖。使用完整任务环境时，还需按仓库流程在私有 Assets 中准备场景、物体与该配置的布局。`build_asset.py` 的 `SCENES` 已关联办公室场景；不要在没有目标物体资产的情况下声称任务成功。

在已安装仓库规定 Isaac Sim / IsaacLab 的隔离环境中运行纯仿真运动检查：

```bash
CUDA_VISIBLE_DEVICES=0 PYTHONPATH=. python scripts/g1d/xense_gripper/runtime_probe.py \
  --out output/xense_probe --device cuda:0 --headless
```

这会检查全身关节和双侧夹爪，保存实际状态及渲染图，不输出抓取任务成功率。输出目录必须不存在。无需加载真实机器人、DDS、真实相机或触觉 SDK。

如需从原始 STEP 重建，另建 Python 3.13 CPU 环境，安装 `requirements-cad.txt`，然后运行：

```bash
python scripts/g1d/xense_gripper/build_from_cad.py --out output/xense_rebuild
python scripts/g1d/xense_gripper/preview_model.py output/xense_rebuild --out output/xense_preview
```

生成器固定源 SHA，源版本或装配拓扑变化时拒绝继续。预览图及 GLB 的简化只用于展示，完整 URDF 使用的 STL 不被简化。

## 控制与待确认参数

| 项目 | 当前值 / 状态 |
| --- | --- |
| 开度 | 归一化 0 闭合、1 全开；两指各行程 0–0.0425 m，轴相反、等量 mimic；开度 = 两指行程之和 |
| 手臂 | 左右各 7 rad 关节目标；加两个夹爪标量仍为 16 维；腰和升降沿用可选独立动作键 |
| 腕部安装 | 沿用旧腕接口的 41.5 mm 原点；CAD +Z 朝向腕 +X、CAD +X 朝向腕 +Y。没有 G1-D 转接板 CAD/孔位实测，属于候选位姿 |
| TCP | 距安装原点约 146.277 mm，按硅胶面轴向包围盒中点推导，非工具实测 |
| 质量 / 惯量 | 每夹爪 615 g 按 CAD 实体体积分配，以均匀等效密度估算各部件；虚拟坐标链接各 1e-6 kg；非部件实测 |
| 力与速度 | 每指 40 N；将产品 140 mm/s 保守解释为总开度速度，各指 70 mm/s，需实测确认 |
| 接触 / 触觉 | 主物理引擎保留刚体凸包；独立 XENSE FEM / SimpleSensor 从同帧接触几何生成四路观测，不向主引擎重复施力；摩擦、弹性、光学和力值仍未实测标定 |
| 相机 | 只保留原头部双目；没有借用 Dex1 腕相机。新整机相机、关节零位、夹爪和动力学标定均 pending |

几何与仿真检查通过也不能将这些待确认项改成 done。本次没有连接真实设备。后续若要建立实物对应关系，仍需材料摩擦、压入/恢复和法向/剪切数据。纯仿真结果与真实标定分别记录。

## 融合内容与复现

保留完整 G1-D 的 URDF、控制增益、碰撞体和碰撞过滤。对外统一实际开度 `opening_m ∈ [0, 0.085]`；URDF 每指为 `opening_m/2`，MuJoCo 原生 CAD 参考每指闭合量为 `(0.085-opening_m)/2`。原 CAD 到本体安装基准沿原 +Z 平移 78.5000001 mm。原 16 维归一化动作保持 0 闭合、1 全开。

`real2sim/grippers/aurora_fusion/` 冻结本机 v2 参考网格、零件分组与来源审计、通用法兰 STEP/PDF/网格。75 个叶部件的分组逐项对应；不导入参考模型的占位惯量、机壳碰撞关闭和整对夹指接触排除。通用法兰仍是**未安装参考件**，G1-D 腕端孔位及转接变换未确认，不虚构可装配结论。MuJoCo 是单爪独立核对工具，完整机器人只有一个本体 ID。

CPU 几何和原生 MuJoCo 核对：

```bash
python scripts/g1d/xense_gripper/check_fusion_alignment.py --out output/fusion_alignment
python scripts/g1d/xense_gripper/check_mujoco_reference.py --out output/fusion_alignment/mujoco.json
python -m unittest scripts.g1d.xense_gripper.test_fusion_contract
```

在已安装 Isaac Sim 5.1 / IsaacLab 2.3 的隔离环境中，安装本资产后运行：

```bash
PYTHONPATH=. python scripts/g1d/xense_gripper/fusion_runtime.py --out output/fusion_recording --headless
```

此脚本记录整机、双爪、四个胶面、物体姿态和主引擎法向/摩擦力。夹持件为 30/50 mm 宽、20 g 的盒体；放置/夹紧阶段无重力，保持阶段施加等效 0.1962 N 向下负载，30 mm 件再施加 5 N 切向载荷观察脱离。这是受控仿真夹具检查，不是完整拾取任务或实测夹持精度。平头/球头为规定位置的探头；摩擦 0.5/0.4 是显式候选参数。

触觉使用外部厂家 Isaac 4.5 分发包中的 **CPython 3.10** 库，独立进程运行，不将二进制导入 Isaac 5.1 的 Python 3.11。厂家二进制、标定和 FEM 原文件不放进公开覆盖目录；按随包许可单独准备，工具会校验两份资源 SHA。`requirements-tactile.txt` 为本次独立 CPU 环境的依赖锁定；厂家的 Qt/Xensim 依赖来自分发包。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/厂家包/pip_prebundle xvfb-run -a /你的/python3.10 \
  scripts/g1d/xense_gripper/tactile_worker.py --recording output/fusion_recording --out output/fusion_tactile
```

四路顺序为 `left_positive / left_negative / right_positive / right_negative`。从实际球/盒几何及保存姿态做射线观测；CAD 胶面不缩放，有效 FEM 面裁取 17.3×29.14 mm。SimpleSensor 用负压入深度；FEM 保留负压入和正间隙，射线未命中为 +5 mm。**不能把未命中也填 0**，否则等价于整张胶面被一个平面贴住，卸载后会残留接触。光学翻转/颜色及真实 SN 对应仍待标定。

`XenseTactileBridge` 和左右机器人类的 `configure_tactile_recording()` / `tactile_observation(frame_id, simulation_time_s)` 提供可选观测接口，严格拒绝错帧、旧时间戳和非有限值。不会改变旧动作维度或自动给旧策略添加输入。输出含四路 RGB、压入/间隙、FEM 节点力/标记点及单独的 PhysX 力。FEM 节点数组范数不是整面合力；其力值单位/精度尚未独立校准。现提供离线及跟随生产进程的按帧链路，缺帧/错帧拒绝、可有限时锁步等待；没有 30 Hz 实时或触觉反馈控制验收。四路采集/导出/策略读回详见 `DATA_PIPELINE.md`。

`--replay-recording <记录目录>` 可重新取景，不重跑物理轨迹。胶面可视姿态使用保存的实际位姿，包含固定关节求解的微小偏差；记录源 SHA 和每帧误差随媒体交付。

## 早期验证（2026-10-22，保留历史范围）

- 仓库 CPU 检查：35 项通过，ruff 0.11.7 和任务清单检查通过；另外对新增建模工具单独做了 lint。
- 生成模型：47 链接、46 关节，75 个实际网格文件；源 SHA、URDF 树、正定惯量及 0 / 42.5 / 85 mm 几何开度核对通过。
- 隔离 Isaac Sim：22 个可动关节，40 个运动/返回阶段通过所列容差；双侧全开约 85.000 mm，半开约 42.500 mm，闭合时实测剩余约 0.162 mm。最大单指跟踪误差 0.0902 mm，最大手臂/腰部误差 0.0334 rad，不能当作精细操作或厂家重复精度验收。
- 首轮全闭合因齿条凸包填满凹区而受阻约 11 mm；仅按原零件拆分仍有问题。最终采用轴向 CAD 分区，保留外部硅胶面接触。纠错经过及实际关节记录见本体目录 `validation/`。
- 虚拟坐标链接没有可视网格，导入器有对应的视觉引用告警；必需的链接和可动关节均实际导入并读回。这一组是初版整机运动检查，融合接触/触觉检查单独保存在 `validation/fusion_*`，没有完整拾取或真实传感器精度验收。

## 提交包

交付压缩包包含仓库覆盖文件、真实二进制 CAD/STL、补丁、摘要和检查证据；不含 Git 历史、账号数据库或 SSH 密钥。应用到基准提交后先检查差异，再由用户自己执行 Git LFS、提交和推送。预览在仓库中默认被忽略，包中另外保留，重建后也会被本地平台读取。
