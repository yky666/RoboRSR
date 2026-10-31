# 裁纸刀实物资产交付报告

2026-10-30。状态：研究资产，待外观审核/缺面补拍；共享资产库未部署。平台原始照片来源为实物，不把渲染计作实拍。

本体 `Rigid/paper_knife/00000`，三件仅颜色泛化 `00001–00003`。黄色 PP 外壳、开放钢导轨、分段刀片、条纹黑色滑块、贯通尾孔；固定为照片中姿态的刚体，不含切割/滑块驱动。泛化分别为青绿/象牙边框、深蓝/橙边框、紫色/浅灰边框；其质量继承本体作为设计值，非重新实测。

## 输入、坐标和尺寸

初始 revision 与四张照片的 bytes/SHA256 一致。用户补测厚度17.14 mm 并指示继续后，按真实对话来源更新登记及确认：新 revision `e249245a730f5980ca6d5b91285c3dad45f476e8a4ded6581e561aa9bbf0fdd4`；没有冒用原登记人签名。旧登记及平台文档保留于部署机备份，见 registration_audit.json。

米制，Z向上，X从刀头指向尾部，原点为全部显示几何包围盒中心。长度是全件X跨度；最大宽度是全件Y跨度；中间宽度是长度中点处外壳截面Y跨度；厚度是全部显示几何Z跨度。metadata 的 aligned/oriented bbox 包住全部部件。

| 项 | 实测 | 模型 | 绝对误差 |
| :-- | --: | --: | --: |
| 长度 mm | 152.88 | 152.88 | 0.00 |
| 最大宽度 mm | 39.44 | 39.44 | 0.00 |
| 中间宽度 mm | 32.83 | 32.83 | <0.01 |
| 厚度 mm | 17.14 | 17.14 | 0.00 |
| 质量 g | 58.9 | 58.9 | <0.00001 |

厚度在本次对话中实测补充，其余登记实测不变。局部壁厚、倒角、齿距、挂孔与滑块轮廓是按照片拟合的重建参数，不是额外实测；单独记录于 reconstruction_parameters_mm。摩擦未知，不在 metadata 中冒写实测系数，使用项目默认0.6/1.5，未经标定。

## 外观与标签

四张实拍均为正面或正面斜视；背面和底面缺照片，以纯色PP表示，未编造标识，需补拍后才能确认这些面。圆孔旁的浅压印从原图提取，独立 `label_blade_stamp` 网格，贴图512×384用于5.3×3.6 mm覆盖区；去除平滑明暗并保留原有浅刻痕，不猜写文字。它是金属上的压印，使用不透明金属材质；无证据表明此处是纸贴纸。源图四角和覆盖范围见 derived/texture_provenance.json、preview/photo_regions_annotated.jpg。磨损钢面也取自原图裁剪。

preview/ 中有中性灯光多视角、四张逐一照片对照 comparison_0–3.jpg、夹持抬升图。照片机位按同侧方向近似匹配，未标定内外参，不宣称像素级配准。颜色/粗糙度目视匹配，非光度或材料标定。运行时换色仅 `visual.color_parts=["model"]`；尾孔边框、钢件、滑块和压印不换色。rt_scale=0；任务尚未打开随机换色，未修改任务布局。

## 实际验证

生成器每件124个显式凸包，单包最多64顶点；显示与碰撞网格分开，挂孔通道自检未封。每个显示部件正体积断言通过。

从部署项目 `SceneManager.instantiate_object_wrapper` 独立加载四件，实际质量回读58.900002 g；四件落地稳定。标准 check_object.py 也四件OK（倾角约0.04°）。native_report.json 标明实际加载器路径和内容哈希，以识别部署工作树实现；运行时默认值加载功能属于该部署代码，不在本次修改范围内。

原生报告：实体推动约47.32 mm；1.2 mm半径探针穿过尾孔、在实体上方受阻；两块规定运动的夹块抬升50 mm，物体抬升49.94 mm、相对滑移约0.057 mm，释放后落地；复位位置误差约2.9e-9 m，速度归零并再次静置。夹块为接触代理，未完成机器人站位/控制/抓取场景验证，不能据此声明机器人任务成功或实物标定。未通过此层验收前留研究资产。

仓库检查 scripts/check.sh：91 passed、1 skipped、82 subtests passed，退出0；ruff、名称检查通过。首次LFS指针导致的资源哈希失败及启动/测试脚本故障保留在交付包 evidence/，随后修复后重跑。模型SHA和全部原生数据见 delivery_report.json。

## 复现

```bash
python scripts/objects/make_paper_knife.py ovl/Rigid
python scripts/objects/object_names.py apply ovl
python scripts/objects/object_names.py check
python scripts/objects/check_object.py ovl/Rigid/paper_knife/00000 --device cpu --image-dir real2sim/objects
python scripts/objects/check_paper_knife.py --root ovl/Rigid/paper_knife --output validation-new
```

环境需 numpy/scipy、Shapely2.2、Pillow/OpenCV、pxr及项目Isaac环境。runtime_config只包含项目支持的physics/visual；随机化意图保存在metadata，不向配置加无效字段。

## 全部指令名称

### Rigid/paper_knife/00000

- en: knife → paper knife → plastic paper knife → plastic paper knife with metal guide → plastic paper knife with ribbed slider and yellow tail rim
- zh: 刀具 → 裁纸刀 → 塑料裁纸刀 → 带金属导轨的塑料裁纸刀 → 带条纹滑块和黄色挂孔边框的塑料裁纸刀

### Rigid/paper_knife/00001

- en: knife → paper knife → plastic paper knife → plastic paper knife with metal guide → plastic paper knife with ribbed slider and ivory tail rim
- zh: 刀具 → 裁纸刀 → 塑料裁纸刀 → 带金属导轨的塑料裁纸刀 → 带条纹滑块和象牙色挂孔边框的塑料裁纸刀

### Rigid/paper_knife/00002

- en: knife → paper knife → plastic paper knife → plastic paper knife with metal guide → plastic paper knife with ribbed slider and orange tail rim
- zh: 刀具 → 裁纸刀 → 塑料裁纸刀 → 带金属导轨的塑料裁纸刀 → 带条纹滑块和橙色挂孔边框的塑料裁纸刀

### Rigid/paper_knife/00003

- en: knife → paper knife → plastic paper knife → plastic paper knife with metal guide → plastic paper knife with ribbed slider and light gray tail rim
- zh: 刀具 → 裁纸刀 → 塑料裁纸刀 → 带金属导轨的塑料裁纸刀 → 带条纹滑块和浅灰色挂孔边框的塑料裁纸刀

## 模型 SHA256

- 00000: `7631ee56711970bb465036c893dd347be52259893b1385327166d3b2c8e6770c`
- 00001: `634bd07d938b8ac48ea654a94b75a67de110e145fb3d795343b9e2322750d70e`
- 00002: `7f86affab6cb926ad0d486bd8f44d6b1210161797793cf7ce153f324843cdfbf`
- 00003: `32e391a38bfad27d319fe898500566fd64735ab3c9a4ca4de0eb95be58c6950b`
