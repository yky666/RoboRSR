# Aurora v3 仓库交付记录

2026-10-24，将 `g1d_aurora_fusion_v3_20261024.zip` 接入 `main@8e0d7a7`，分支为
`komari/feat-g1d-aurora-tactile`。本体 ID 为 `g1d_aurora_lite_ws`。

## 文件归属

| 内容 | 位置 |
| --- | --- |
| 冻结整机、单爪 URDF、STL、物理契约及摘要 | `real2sim/robot/g1d_aurora_lite_ws/` |
| CAD、独立 MuJoCo 对照、法兰参考与来源审计 | `real2sim/grippers/` |
| 本体登记、实测边界、交付方验证记录 | `real2sim/embodiments/g1d_aurora_lite_ws/` |
| 控制、仿真加载、触觉观测接口 | `env/robot_manager/` 中对应 Aurora 模块 |
| 环境和机器人配置 | `env_cfg/g1d_aurora_lite_ws.yml`、`env_cfg/robot/g1d_aurora_lite_ws.yml` |
| 建模、安装、复现、数据工具 | `scripts/g1d/xense_gripper/` |
| 原交付清单及接入来源 | 本体目录下 `delivery/` |
| 本地完整交付包、evidence、HDF5/LeRobot 样例 | `.cache/aurora-v3-delivery/g1d_aurora_fusion_v3_20261024/`，不入库 |
| 本地 GLB 预览 | 本体 `preview/`，按现有忽略规则不入库 |

`delivery/apply_manifest.json` 和 `delivery/SOURCE_README_zh.md` 是原始交付记录。
原应用器面向旧基线，不能再次覆盖当前仓库；当前接入采用新增文件复制及已有文件的增量合并。
2026-10-25 用户明确要求完整资料随分支上传，因此原始 ZIP 已作为例外放到本体目录的
`delivery/g1d_aurora_fusion_v3_20261024.zip`，使用 Git LFS 保存。它包含全部审阅视频、
轨迹、HDF5/LeRobot 样例、预览和原覆盖文件；不另行将解压输出散落到源码目录。
厂家私有运行包不在原 ZIP 内，本地数据库及临时测试环境仍不入 Git。

## 2026-10-25 同步与完整资料上传

- 将远端 `main@961a9f5` 合并到已有 Aurora 分支，保留双方的机器人映射、检查目标和
  HANDOFF 历史；主线新增场景、RSI 与控制更新均纳入。
- 主线给 Dex1 新增了 `finger_coupling`，其逻辑假设关节值增加代表闭合；Aurora 的方向
  相反，因此在 Aurora 类中显式禁用继承的 Dex1 限制。回归测试覆盖双侧 Aurora 与 Dex1。
- 完整 ZIP 的预期 SHA256 见 `delivery/provenance.json`。在 Linux/macOS 上可从仓库根执行：

  ```bash
  git lfs pull --include="real2sim/embodiments/g1d_aurora_lite_ws/delivery/*.zip"
  mkdir -p .cache/aurora-v3-delivery
  python -m zipfile -e real2sim/embodiments/g1d_aurora_lite_ws/delivery/g1d_aurora_fusion_v3_20261024.zip .cache/aurora-v3-delivery
  ```

  Windows 也可用同一组 Python/Git 命令。解压后的样例路径与 `registry.json` 中的记录一致。

## 与覆盖包的必要差异

- 保留当前 `HANDOFF.md` 的结构和历史，仅写入本次交付状态。
- README、数据契约、机器人映射、平台测试只加入 Aurora 条目。
- `build_asset.py` 只扩展 `SCENES`，保留主线已有的 `appearance.load/apply`。
- STEP/STP 大小写与 GLB 纳入 LFS。冻结模型文本和带源摘要的工具使用 LF，防止
  Windows 自动换行造成安装器 SHA256 校验失败。
- 新增 `test_aurora_integration.py`，默认检查包含它及交付包中的两组触觉测试。
  CI 选择性拉取新模型的 LFS；补齐原 X5 布局检查依赖 `tqdm`。
- 平台生成器/配置相对路径和物体索引使用 POSIX 字符串，在 Windows/Linux 保持一致。

## 验证范围

接入前已独立核对覆盖包 215 个文件摘要；从交付的六组原始轨迹重算任务审计，结果均与
交付报告一致：05/06/09/10 的原生任务通过，07/08 失败；06 触觉越界使整体仍不接受。
上一轮审阅还独立运行了 5 个触觉契约测试。它们均不等于重新运行 Isaac 仿真。

本次交付检查的实际结果记录在本体 `delivery/integration_validation.json`。
用户在接入过程中明确取消新增运行时安装及仿真复测，因此没有重新运行 Isaac、厂家
Python 3.10 触觉进程、LeRobot 导出或训练策略，没有写共享 Assets、数据库或远程服务。
包内 `validation/` 属于交付方证据，保留原记录与日期，不改写成本次复测结果。

## 后续使用

本次只交付仓库源码与资产源。将来需要运行时，在已具备 Isaac 环境和私有 Assets 副本后，
依照 [README](README.md) 安装本体；本次未执行资产安装器。完整任务的默认/宽块参数与
计划见本体 `simulation_profiles.json`、`validation/full_task_plan.json`，四触觉试验数据见
[DATA_PIPELINE](DATA_PIPELINE.md)。

厂家运行包需单独准备；包内旧工作站路径仅作来源记录。当前为刚体胶面加单向 FEM 观测，
保留实测标定 pending，未验收实时触觉闭环或现有策略权重兼容性。
