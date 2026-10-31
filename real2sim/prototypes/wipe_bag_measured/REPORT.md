# 湿巾袋实测建模 · 研究资产

2026-10-30；署名：刘小凡 / Codex。原始登记、照片取自平台状态目录，确认版本 `13fbcc90080f426434b76059f60cf51ffa0631d4e1dbac275c8f89dda584d8d5`。
本体 `Rigid/wipe_bag/00000`，外观泛化 `00001–00003`。三张原照的字节数与 SHA-256 已逐一核对；登记输入保持原样。

## 尺寸、质量与来源

| 项目 | 实测 | 模型 | 说明 |
| --- | ---: | ---: | --- |
| 长度 | 190 mm | 190.0 mm | 含两端封边的全部可见网格 X 包围盒 |
| 宽度 | 96.02 mm | 96.02 mm | 含印刷网格的全部可见网格 Y 包围盒 |
| 高度 | 38 mm | 38.0 mm | 从袋底至翻盖卡扣最高点 |
| 总质量 | 240.2 g | 240.2 g | 整包质量；USD 属性另有原生读回，不声称读到节点质量张量 |

单位米，Z 上，X 为长度，原点为完整显示包围盒中心。变体继承同一尺寸、形状及质量，不作为新的实测。
翻盖 120×64 mm 等局部比例为照片拟合假设，独立保存在 `estimated_dimensions_mm`；薄膜厚度、各部分质量与实测摩擦保持 null。

## 外观与贴图

独立袋身、翻盖外框、盖板、缝隙、铰接外形、卡扣及三块印刷网格。封边和袋身有显式轮廓褶皱。
`label_lid`、`label_instructions`、`label_count` 均由原照四角透视校正，去除低频亮度梯度；画布 10 px/mm。
原图有效采样估计盖面 12–15 px/mm、侧面 12–17 px/mm，增加画布不代表增加真实细节。
每块来源角点、SHA、物理范围与标注图在资产 `label_sources.json`、`textures/*_source_bounds.jpg` 中。
本体保留宠物图案、文字和数量印刷；变体保留同样图案，仅改变膜背景、盖子颜色与表面粗糙度。
运行时换色只指定 `model/geometry/mesh`，名称以不会随机换色的翻盖颜色区分。
底面、另一长侧和开盖内部缺照片，保留纯色，不编造文字。三组同视角对照为近似机位，未做相机标定。
外观验收待审核：端部折叠简化、侧面贴图底色仍可见边界、照片残余褶皱/阴影尚存；未将这些记为通过。

## 实际运行的物理检查

通过本分支的 `SceneManager.instantiate_object_wrapper(obj_type="rigid")` 实际加载，显式 FEM 后端避免错误加载成刚体。
复用项目已有 prototype 的 `asset_defaults`、`deformable`、`tetra_visuals` 实现并纳入本分支，未改共享工作区。
整包等效体积 FEM：E=80000 Pa、ν=0.3；这些是未标定的数值假设，不是塑料薄膜本构。
仿真网格 1619 节点、5730 四面体；碰撞网格另存于 `mesh08/cooked_fem.npz`。
显示袋身网格、烹制仿真网格与碰撞网格分别记录。显示翻盖依附 FEM 随动，不是独立刚体翻盖或可开合关节。

- 落地：最低碰撞节点 -0.00516 mm，碰撞袋身高度 34.779 mm。
- 实体阻挡与压缩：高度 34.787 → 29.497 mm。
- 卸载恢复：34.783 mm；全过程每步检查有限值与单元 Jacobian，最低 0.60989。
- 复位：节点最大误差 8.24e-09 m，速度归零，所有节点自由；再次落地通过。
- 同一初态两次 720 步重放：最大节点误差 0.02304 mm。
- 八块随动视觉网格最大误差 7.83e-09 m；更新 3243 次，回调已注销。
- 四个资产均在独立进程重新加载，USD 总质量属性读回与 240.2 g 一致，贴图随动通过。
- 夹板代理：30 mm 间隙，命令抬高 80 mm，实际质心抬升 75.051 mm；释放后回落。
  下夹板提供支撑，未固定节点；不能据此声称机器人摩擦抓取、滑移标定或任务成功。

完整结果、模型 SHA、质量属性与实际摩擦/材料属性读回见 `native_report.json`。
原生检查通过仅覆盖上述研究测试，不覆盖独立硬盖、薄膜、内容物分离或真实机器人复用。

## 生成与复验

```bash
CUDA_VISIBLE_DEVICES=7 python scripts/objects/make_wipe_bag.py ovl/Rigid
python scripts/objects/object_names.py apply ovl
python scripts/objects/object_names.py check
CUDA_VISIBLE_DEVICES=7 PYTHONPATH=. python scripts/objects/check_wipe_bag.py --asset ovl/Rigid/wipe_bag/00000 --output validation
CUDA_VISIBLE_DEVICES=7 PYTHONPATH=. python scripts/objects/render_wipe_bag.py --assets ovl/Rigid/wipe_bag --registration real2sim/objects/Rigid/wipe_bag/00000 --output renders
```

CPU `scripts/check.sh` 已通过；初次 CRLF 与 LFS 指针失败均保留原日志，补齐 LFS 真文件后复跑。
未装共享资产库，未宣称基础资产验收通过。后续需补拍缺失面、核对端部折叠和贴图、完善独立翻盖物理并运行完整机器人场景。

## 全部指令名称

- `Rigid/wipe_bag/00000`
  - en: bag / wipe bag / pet wipe bag / pet wipe bag with white lid / sealed plastic pet wipe bag with white lid and animal label
  - zh: 袋子 / 湿巾袋 / 宠物湿巾袋 / 带白色翻盖的宠物湿巾袋 / 带白色翻盖和动物标签的封口塑料宠物湿巾袋
- `Rigid/wipe_bag/00001`
  - en: bag / wipe bag / pet wipe bag / pet wipe bag with green lid / sealed plastic pet wipe bag with green lid and animal label
  - zh: 袋子 / 湿巾袋 / 宠物湿巾袋 / 带绿色翻盖的宠物湿巾袋 / 带绿色翻盖和动物标签的封口塑料宠物湿巾袋
- `Rigid/wipe_bag/00002`
  - en: bag / wipe bag / pet wipe bag / pet wipe bag with red lid / sealed plastic pet wipe bag with red lid and animal label
  - zh: 袋子 / 湿巾袋 / 宠物湿巾袋 / 带红色翻盖的宠物湿巾袋 / 带红色翻盖和动物标签的封口塑料宠物湿巾袋
- `Rigid/wipe_bag/00003`
  - en: bag / wipe bag / pet wipe bag / pet wipe bag with blue lid / sealed plastic pet wipe bag with blue lid and animal label
  - zh: 袋子 / 湿巾袋 / 宠物湿巾袋 / 带蓝色翻盖的宠物湿巾袋 / 带蓝色翻盖和动物标签的封口塑料宠物湿巾袋

## 模型 SHA-256

- `Rigid/wipe_bag/00000`: `987304b2584900c2e755866f2d5663abfdf3c97c33cefa028d15e681d9038b8e`
- `Rigid/wipe_bag/00001`: `6c93d992aca4f4db868cb9d1004591971dcab936305addeeba253d7e7a35eb98`
- `Rigid/wipe_bag/00002`: `ca166d2245b58eaeeddc3628ffab3d7018325aeb99029b4d8bb8a0d2060d0b80`
- `Rigid/wipe_bag/00003`: `00b0fe6dc4dedb8a555b561bfe519dd912c47c284e4d8102e1f1d0247854f8d8`
