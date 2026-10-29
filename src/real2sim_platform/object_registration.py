"""One versioned modeling prompt, issued only for an explicitly reviewed registration."""

import hashlib
import json
from pathlib import Path
import time

VERSION = "real-object-v3"
FIELDS = (
    "asset_origin",
    "photo_reference",
    "name",
    "name_en",
    "material",
    "color",
    "dimensions",
    "real_mass",
    "real_friction",
    "features",
    "requirements",
    "notes",
    "gen",
    "estimated_dimensions_mm",
    "estimated_mass_g",
    "reference_source",
    "dimension_policy",
    "mass_policy",
    "physics_requirements",
)
POLICIES = {"measured", "existing", "estimate", "unknown"}
LABELS = {
    "measured": "实测",
    "existing": "沿用现有值（未实测）",
    "estimate": "允许估计并注明依据",
    "unknown": "待测，保留未知",
}


def version(doc):
    """Changing the flexible protocol invalidates only flexible modeling confirmations."""
    return VERSION + "-deformable-v2" if (doc.get("deformable") or {}).get("family") else VERSION


def physical_origin(doc):
    """Physical provenance is independent of measurement or simulation validation."""
    if doc.get("twin") is not None:
        return bool(doc["twin"])
    return doc.get("asset_origin") == "physical_object" or bool(doc.get("auto", {}).get("physical_source"))


def public_object(doc):
    return {**doc, "twin": physical_origin(doc), "modeling": state(doc)}


def inputs(doc):
    data = {key: doc.get(key) for key in FIELDS}
    if (doc.get("deformable") or {}).get("family"):
        data["deformable"] = doc["deformable"]
    data["id"] = doc["id"]
    photos = []
    for path in sorted(doc.get("auto", {}).get("photos", [])):
        p = Path(path)
        stat = p.stat() if p.is_file() else None
        photos.append(
            {
                "path": str(p),
                "bytes": stat.st_size if stat else None,
                "sha256": hashlib.sha256(p.read_bytes()).hexdigest() if stat else None,
            }
        )
    data["photos"] = photos
    # Existing model defaults can be retained without claiming a physical measurement.
    data["existing_model"] = {key: doc.get("auto", {}).get(key) for key in ("size", "mass", "generator")}
    return data


def revision(doc):
    raw = json.dumps({"version": version(doc), "inputs": inputs(doc)}, sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(raw.encode()).hexdigest()


def missing(doc):
    labels = {
        "name": "中文名",
        "name_en": "英文名",
        "material": "材质（可写待核实）",
        "color": "颜色",
        "features": "结构与外观",
        "requirements": "建模要求",
        "reference_source": "照片/实物来源",
        "physics_requirements": "物理类型与测试要求",
    }
    todo = [label for key, label in labels.items() if not str(doc.get(key) or "").strip()]
    for key, label in (("dimension_policy", "尺寸依据"), ("mass_policy", "质量依据")):
        if doc.get(key) not in POLICIES:
            todo.append(label)
    if doc.get("dimension_policy") == "measured" and not doc.get("dimensions"):
        todo.append("实测尺寸")
    if doc.get("mass_policy") == "measured" and not isinstance(doc.get("real_mass"), int | float):
        todo.append("实测质量")
    if not physical_origin(doc):
        todo.append("实物登记")
    return todo


def confirm(doc, by):
    todo = missing(doc)
    if todo:
        raise ValueError("请先核对登记字段：" + "、".join(todo))
    return {"version": version(doc), "revision": revision(doc), "by": by, "at": time.time()}


def state(doc):
    confirmation = doc.get("modeling_confirmation") or {}
    valid = (
        not missing(doc)
        and confirmation.get("revision") == revision(doc)
        and confirmation.get("version") == version(doc)
    )
    return {
        "confirmed": valid,
        "missing": missing(doc),
        "version": version(doc),
        "confirmed_by": confirmation.get("by"),
        "prompt": prompt(doc) if valid else None,
    }


def prompt(doc):
    """Generate from current inputs only; never expand missing values into invented facts."""
    data = inputs(doc)
    return "\n".join(
        [
            f"# RoboDojo 实物资产建模任务 · {version(doc)}",
            f"资产：{doc['id']}；登记版本：{revision(doc)}。",
            "先读取 scripts/objects/SKILL.md 和该资产 spec.json；核对确认版本与当前字段/照片一致。"
            "过期时停止使用旧 prompt 并重新确认。",
            "",
            "## 已确认输入（单位：mm、g；未提供项为 null）",
            json.dumps(data, ensure_ascii=False, indent=2),
            f"尺寸来源：{LABELS.get(doc.get('dimension_policy'), '待确认')}；"
            f"质量来源：{LABELS.get(doc.get('mass_policy'), '待确认')}。",
            "",
            "## 几何与外观",
            "以实物照片/官方CAD为依据，统一米制、Z向上、记录原点与包围盒定义；保留可见孔洞、开口、关节、把手和薄壁。截面（圆、方、倒角多边形、收腰）、肩颈比例和盖子结构逐张对照照片，不套用相近物体的现成模板。材质、颜色、粗糙度、透明度逐项与参考核对，输出统一中性光照多视角预览。",
            "只在允许估计时估计，并写明来源；沿用现有值时保留模型参数。估计尺寸/质量不写成实测，未知值保持未知。照片来源与仿真用途分别记录。",
            "",
            "## 标签、贴纸与印刷图案",
            "- 实物上的标签、贴纸、logo、文字、花纹必须建成照片贴图："
            "平面从参考照片裁剪并做透视校正，曲面按柱面展开，去掉照片的明暗和反光后作纹理。"
            "不得省略，也不得用程序绘制的通用文字、插图或纯色块代替。",
            "- 先在照片上标出每块标签的范围（起止高度、所在面或包角），只贴在实物有标签的位置，不要包满整件；"
            "每块单独一个网格（如 label_front、label_back），贴图 ≥ 10 px/mm、文字可读，"
            "略高出表面避免闪烁，用不透明的纸/膜材质。透明主体透出的标签背面用纸背颜色，不镜像正面图案。",
            "- 照片没拍到的标签面列出缺的角度，请登记人补拍；补拍前该面写“缺照片”并用纯色底，不编造内容。"
            "泛化变体可换虚构品牌的绘制标签，但位置和覆盖范围沿用本体。",
            "- 按参考照片的视角各渲染一张，与照片并排做成对照图；标签缺失、位置或颜色与照片不符，都不算完成。",
            "",
            "## 指令名称",
            "- 训练指令里的 <标签> 由物体名称填入。本体和每个泛化变体各写英文（en）、中文（zh）名称，每种至少 5 个，"
            "从简到详排列：大类泛称（cup / 杯子）→ 类别名（measuring cup / 量杯）→ 类别加一个最显著的属性"
            "（clear measuring cup / 透明量杯）→ 再加两三个区分属性（大小、形状、材质、标签）→ "
            "能在同族里唯一指认它的完整短语。中间各级可以补同义说法。",
            "- 写名词短语，不带冠词、数量词和句末标点，能直接填进「pick up the <x>」「拿起<x>」；"
            "英文小写（品牌等专名除外）。",
            "- 只写模型上看得见的固有属性：不写位置、左右、比较级和任务角色。颜色和标签以生成的模型为准；"
            "变体写它自己的属性，前两级通常和本体一致。登记开了运行时换色（rt_color）时，"
            "不写会被换色的部件的颜色（metadata.json 的 visual.color_parts，没写就是整件）。",
            "- 写进 scripts/objects/object_names.json（键是资产 id）。每次生成后运行 "
            "`python scripts/objects/object_names.py apply <物体根目录>`，写入 description.json 的 names；"
            "`object_names.py check` 不通过不算完成。交付说明列出全部名称。",
            "",
            *(
                [
                    "## 柔性专用验收",
                    "按 real2sim/prototypes/deformable_assets/README.md 建模；"
                    "布/衣/袋保留裁片、开口和接缝，绳单独验证弯曲和扭转。",
                    "另写 preview/deformable_check.json，包含 mesh_sha256、registration_sha256、"
                    "material_calibrated 和各项 checks 的状态及证据。",
                    "刚体落地检查不能替代柔性验收。实测材料未标定、初始状态恢复失败或证据过期时不得标通过。",
                    "reset_state 验证初始位置/速度；diagnostics.replay 单独报告重放偏差，无通用 1 mm 门槛。",
                    "",
                ]
                if (doc.get("deformable") or {}).get("family")
                else []
            ),
            "## 物理与复用",
            "按登记选择刚体、关节体、布料、绳索或体积软体后端；分开记录显示网格与碰撞网格。关节必须有轴、原点、限位、驱动；柔性物体必须有实际变形求解，材料未辨识时写明假设。",
            "从项目实际工厂加载，验证落地/静置、碰撞通道与实体阻挡、复位/再次加载；按物体类型补开合、悬垂、压缩、卸载恢复或夹持测试。仅报告实际运行项目，保留失败；独立模型/静态渲染不能证明任务可用或实物标定。",
            "",
            "## 交付与入库",
            f"代码及输入：real2sim/objects/{doc['id']}/spec.json、reference/、生成脚本、参数来源、"
            "object_names.json 条目；"
            f"生成结果（Real2Sim物体库Assets/Object/Real2Sim/{doc['id']}/）：object.usdz、metadata.json、"
            "description.json（含 names）、runtime_config.json（适用时）。",
            "用同名 measurements_mm 对照已实测尺寸；估计值独立保存。"
            "交付原生报告、模型SHA、多视角渲染和同视角照片对照图。",
            "通过对应类型复用与物理检查后加入基础资产；未通过项留研究资产。统一名称/ID/单位/字段格式。"
            "共享库部署遵守用户授权范围，保留备份；更新 ASSETS.md、HANDOFF.md 与平台署名。",
        ]
    )
