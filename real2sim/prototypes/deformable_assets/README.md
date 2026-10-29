# 柔性物体生成与物理检查

平台登记 → 确认并生成 prompt → agent 建模 → 独立物理检查 → 发布报告与预览。
沿用原照片、资产 ID 和参数来源；生成器不填写实测值，也不修改共享资产库。
登记支持布料/衣服/绳索/布袋/体积软体；本 PR 的参数化生成器只覆盖矩形布片与规则软块。
其他结构仍需 agent 按实物制作裁片、缝线、袋口、提手或绳中心线，不能用矩形/凸包替代。
机器人抓取、折叠和开袋验证尚未接通。

## 环境和上游

物理使用独立 Newton 环境，精确依赖见 `../genie_deformables/requirements.txt`。
Genie 固定源码与许可说明见 `../genie_deformables/upstream.json`。
只执行 hash 校验的外部 `_ClothMixin._inject_cloth`；没有移植完整 ROS 引擎，没有把官方参数当作实测。
布料需要 `--genie-root` 指向固定版本的外部源码；软块不执行 Genie loader。
平台与 Isaac Python 无需安装 Newton，通过独立进程读取状态；显示 mesh 不携带 PhysX schema，避免双重求解。

## 生成与检查

`spec.deformable.geometry.size_mm`：布片宽/长或软块宽/长/高；`cells`：每轴分段数。
质量取 `mass_g` 或明确的 `estimated_mass_g`，没有输入就拒绝生成。
`solver_parameters` 保留候选参数，`parameter_source` 记录来源。只在有实测拟合证据时才标定材料。

```bash
# 在 pinned Newton Python 下运行；每次使用新输出目录。
python scripts/objects/make_deformable.py --spec real2sim/prototypes/deformable_assets/example_spec.json --output /path/to/isolated/assets --device cuda:0
python scripts/objects/check_deformable.py --asset /path/to/isolated/assets/Dynamic/demo_soft_block/00000 --output /path/to/isolated/check --device cuda:0 --frames 240
```

输出：`object.usdz`、显示网格、metadata、runtime_config、输入副本及 mesh.npz。
`metadata.measurements_mm` 留空，须由 agent 按登记的尺寸名称从实际几何测量补齐；不复制输入值冒充模型测量。
检查保存原始节点、刚体轨迹、接触关闭对照、翻转/溢出/地面界限/状态复位/退出检查。
`probe_passed` 只说明软件与数值检查；`asset_accepted=false`，材料、自交、实物外观和机器人操作仍独立验收。
`contact` 对照差异证明出现接触响应，不证明真实材料精度。无溢出不等于无几何自交。

把 `deformable_check.json`、报告和视频整理到原资产的 `preview/`，用 `r2s publish SRC real2sim/objects/<id>/preview` 发布。
不要手工覆盖平台登记；共享库安装仍遵循原有授权/备份流程。

## 新版接触示例

```bash
python scripts/objects/demo_deformables.py --genie-root /path/to/pinned/genie --output /path/to/isolated/demos --device cuda:0 --frames 240
# 以下改用 Isaac Sim Python，RTX 只渲染保存轨迹，不重复求解物理。
python scripts/objects/render_deformable.py --run /path/to/isolated/demos/cloth_over_sphere --output /path/to/isolated/cloth_video --gpu 0
python scripts/objects/render_deformable.py --run /path/to/isolated/demos/sphere_soft_block --output /path/to/isolated/soft_video --gpu 0
```

布片固定一边后下垂到球面；自由球撞击软块并观察形变/恢复。球与物体在 Newton 内部统一求解，
关闭球-粒子接触作对照。视频不是脚本移动球体伪造碰撞。材料是合成候选，不计已验收实物。
外部 PhysX 机器人仍只有单向代理，双向反力回传另行实现。

## 复位与重复性

`reset_state` 检查初始位置/速度恢复；`diagnostics.replay` 单独报告最大/RMS 节点重放误差和帧数。
没有通用 1 mm 门槛：历史 1 mm 是自定义调试线，不是 Genie/SoftMimicGen 标准。
轨迹不同不能单独否定资产；需要任务重复性时先定义操作容差和实物波动基线。
旧报告只有 `reset_replay` 时，不能据此声称已验证 `reset_state`；需补初始状态恢复证据。
网格 SHA、登记指纹与材料校准独立核对，改变质量/尺寸/材料输入会使旧证据失效。

## 复杂柔性结构

| 类型 | agent 构建 | 专用验证 |
|---|---|---|
| 布料 | 平铺轮廓、经纬、厚度/面密度、三角网格 | 悬垂、弯曲、摩擦、自接触 |
| 衣服 | 裁片、缝线配对、领口/袖口/下摆 | 接缝、开口、多层接触 |
| 绳/线缆 | 中心线、截面、端点，明确杆/体积路线 | 弯曲、扭转、绕物、自接触 |
| 布袋/薄膜袋 | 裁片、袋口、褶、提手、缝线/热封边 | 开口保持、提手连接、内部接触 |
| 体积软体 | 四面体网格，质量按单元体积分配 | 翻转、压缩/释放、碰撞恢复 |

各向同性弹性不代表经纬差异、塑性折痕或复合材料；不支持的行为需要明确限制用途。
