# RoboDojo 实物资产建模任务 · real-object-v3
资产：Rigid/paper_knife/00000；登记版本：e249245a730f5980ca6d5b91285c3dad45f476e8a4ded6581e561aa9bbf0fdd4。
先读取 scripts/objects/SKILL.md 和该资产 spec.json；核对确认版本与当前字段/照片一致。过期时停止使用旧 prompt 并重新确认。

## 已确认输入（单位：mm、g；未提供项为 null）
{
  "asset_origin": null,
  "photo_reference": null,
  "name": "裁纸刀",
  "name_en": "Paper Knife",
  "material": "PP 塑料",
  "color": "黄色",
  "dimensions": "长度152.88，最大宽度39.44，中间宽度32.83，厚度17.14",
  "real_mass": 58.9,
  "real_friction": null,
  "features": "黄色外壳、金属刀身",
  "requirements": "可夹持",
  "notes": null,
  "gen": {
    "count": 3,
    "requirements": "参数由 coding agent 设计，彼此差异明显",
    "rt_color": true,
    "rt_scale": 0,
    "color": true,
    "material": false,
    "size": false,
    "shape": false
  },
  "estimated_dimensions_mm": null,
  "estimated_mass_g": null,
  "reference_source": "实物照片",
  "dimension_policy": "measured",
  "mass_policy": "measured",
  "physics_requirements": "刚体；夹持",
  "id": "Rigid/paper_knife/00000",
  "photos": [
    {
      "path": "/root/RoboDojo/real2sim/objects/Rigid/paper_knife/00000/reference/03fc5f3d5bd16f7edd80c9819179e94d.jpg",
      "bytes": 2983697,
      "sha256": "da93366117f0f1268aeb29d5c741c037e06c383353a49b806d698a567d3ef21b"
    },
    {
      "path": "/root/RoboDojo/real2sim/objects/Rigid/paper_knife/00000/reference/12e9a0db3d97a1934cf4a12e6844e472.jpg",
      "bytes": 2970449,
      "sha256": "8f22237fb9454d08738ea4b12398f38dcd3258b5979ab87b08899389bae7e742"
    },
    {
      "path": "/root/RoboDojo/real2sim/objects/Rigid/paper_knife/00000/reference/5e19da097d9027d7b604fc3e822fe316.jpg",
      "bytes": 3055378,
      "sha256": "6a7592708ce4a2ef745b610aa673ccb5e00aefcf2ed8b249ee77e67257ba3994"
    },
    {
      "path": "/root/RoboDojo/real2sim/objects/Rigid/paper_knife/00000/reference/655acc5d276cd24019659874101a64a8.jpg",
      "bytes": 3236710,
      "sha256": "edd2d8f5ee11531aa5fde3206536dc2c3fb2f80cdc84b83c559e6b92ebec0dd8"
    }
  ],
  "existing_model": {
    "size": null,
    "mass": null,
    "generator": null
  }
}
尺寸来源：实测；质量来源：实测。

## 几何与外观
以实物照片/官方CAD为依据，统一米制、Z向上、记录原点与包围盒定义；保留可见孔洞、开口、关节、把手和薄壁。截面（圆、方、倒角多边形、收腰）、肩颈比例和盖子结构逐张对照照片，不套用相近物体的现成模板。材质、颜色、粗糙度、透明度逐项与参考核对，输出统一中性光照多视角预览。
只在允许估计时估计，并写明来源；沿用现有值时保留模型参数。估计尺寸/质量不写成实测，未知值保持未知。照片来源与仿真用途分别记录。

## 标签、贴纸与印刷图案
- 实物上的标签、贴纸、logo、文字、花纹必须建成照片贴图：平面从参考照片裁剪并做透视校正，曲面按柱面展开，去掉照片的明暗和反光后作纹理。不得省略，也不得用程序绘制的通用文字、插图或纯色块代替。
- 先在照片上标出每块标签的范围（起止高度、所在面或包角），只贴在实物有标签的位置，不要包满整件；每块单独一个网格（如 label_front、label_back），贴图 ≥ 10 px/mm、文字可读，略高出表面避免闪烁，用不透明的纸/膜材质。透明主体透出的标签背面用纸背颜色，不镜像正面图案。
- 照片没拍到的标签面列出缺的角度，请登记人补拍；补拍前该面写“缺照片”并用纯色底，不编造内容。泛化变体可换虚构品牌的绘制标签，但位置和覆盖范围沿用本体。
- 按参考照片的视角各渲染一张，与照片并排做成对照图；标签缺失、位置或颜色与照片不符，都不算完成。

## 指令名称
- 训练指令里的 <标签> 由物体名称填入。本体和每个泛化变体各写英文（en）、中文（zh）名称，每种至少 5 个，从简到详排列：大类泛称（cup / 杯子）→ 类别名（measuring cup / 量杯）→ 类别加一个最显著的属性（clear measuring cup / 透明量杯）→ 再加两三个区分属性（大小、形状、材质、标签）→ 能在同族里唯一指认它的完整短语。中间各级可以补同义说法。
- 写名词短语，不带冠词、数量词和句末标点，能直接填进「pick up the <x>」「拿起<x>」；英文小写（品牌等专名除外）。
- 只写模型上看得见的固有属性：不写位置、左右、比较级和任务角色。颜色和标签以生成的模型为准；变体写它自己的属性，前两级通常和本体一致。登记开了运行时换色（rt_color）时，不写会被换色的部件的颜色（metadata.json 的 visual.color_parts，没写就是整件）。
- 写进 scripts/objects/object_names.json（键是资产 id）。每次生成后运行 `python scripts/objects/object_names.py apply <物体根目录>`，写入 description.json 的 names；`object_names.py check` 不通过不算完成。交付说明列出全部名称。

## 物理与复用
按登记选择刚体、关节体、布料、绳索或体积软体后端；分开记录显示网格与碰撞网格。关节必须有轴、原点、限位、驱动；柔性物体必须有实际变形求解，材料未辨识时写明假设。
从项目实际工厂加载，验证落地/静置、碰撞通道与实体阻挡、复位/再次加载；按物体类型补开合、悬垂、压缩、卸载恢复或夹持测试。仅报告实际运行项目，保留失败；独立模型/静态渲染不能证明任务可用或实物标定。

## 交付与入库
代码及输入：real2sim/objects/Rigid/paper_knife/00000/spec.json、reference/、生成脚本、参数来源、object_names.json 条目；生成结果（Real2Sim物体库Assets/Object/Real2Sim/Rigid/paper_knife/00000/）：object.usdz、metadata.json、description.json（含 names）、runtime_config.json（适用时）。
用同名 measurements_mm 对照已实测尺寸；估计值独立保存。交付原生报告、模型SHA、多视角渲染和同视角照片对照图。
通过对应类型复用与物理检查后加入基础资产；未通过项留研究资产。统一名称/ID/单位/字段格式。共享库部署遵守用户授权范围，保留备份；更新 ASSETS.md、HANDOFF.md 与平台署名。
