"""Presentation policy; keep source files, runtime configs and audit records intact."""

HIDDEN_SCENES = {"g1d_green_table", "g1d_green_table_isaac"}
RESEARCH_BODIES = {"aurora_lite", "ur5_official", "g1d_aurora_upper"}


def reusable_body(doc):
    auto = doc.get("auto", {})
    return doc["id"] not in RESEARCH_BODIES and bool(auto.get("in_code") and auto.get("setups"))


def visible(kind, docs):
    if kind == "scene":
        return [d for d in docs if d["id"] not in HIDDEN_SCENES]
    if kind == "embodiment":
        return [d for d in docs if reusable_body(d)]
    return docs


def research_bodies(bodies, registrations=None):
    """Mirror components into research without deleting their registered records."""
    result = {}
    for name, auto in bodies.items():
        if reusable_body({"id": name, "auto": auto}):
            continue
        reg = (registrations or {}).get(name, {})
        result[name] = {
            "name": reg.get("name")
            or {"aurora_lite": "极光 Lite 视触觉夹爪", "ur5_official": "UR5 官方经典裸臂"}.get(name, name),
            "kind": "机器人部件与装配研究",
            "status": "部件模型已有接入；完整场景复用待完成" if auto.get("in_code") else "已登记；模型与场景复用待完成",
            "summary": "保留已有模型、运动检查和登记；完成站位、控制、观测及场景加载/复位验证后再进入本体管理。"
            + (reg.get("notes") or ""),
            "limits": ["独立模型可加载不等同已完成机器人场景复用。"],
            "capabilities": {
                "模型入口": auto.get("config") or "待实现",
                "场景复用": "待完整站位、控制和观测接入",
                **({"实物来源": reg["reference_source"]} if reg.get("reference_source") else {}),
            },
            "runtime_ready": False,
            "renders": auto.get("renders", []),
            "photos": auto.get("photos", []),
            "videos": [],
            "model": None,
            "evidence": [],
            "sources": [],
        }
    return result
