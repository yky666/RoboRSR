import json
import re
import sys
import types

import object_names


def test_table_follows_the_conventions():
    assert object_names.problems(object_names.load()) == []


def entry(en, zh, category=("cup", "杯子")):
    return {"category": {"en": category[0], "zh": category[1]}, "en": en, "zh": zh}


def test_problems_catch_levels_order_articles_and_shared_detail():
    good = ["plastic cup", "clear plastic cup", "small clear plastic cup with a lid"]
    zh = ["塑料杯", "透明塑料杯", "带盖的小号透明塑料杯"]
    assert object_names.problems({"a": entry(good, zh)}) == []
    found = object_names.problems(
        {
            "short": entry(good[:2], zh),
            "order": entry([good[1], good[0], good[2]], zh),
            "article": entry(["the cup", *good[1:]], zh),
            "place": entry(good, [*zh[:2], "左边的小号透明塑料杯"]),
            "category": entry(good, zh, ("plastic cup", "杯子")),
            "wordy": entry(good, zh, ("small plastic drinking cup", "杯子")),
            "nocat": {"en": good, "zh": zh},
        }
    )
    assert [p.split(":")[0] for p in found[:6]] == [
        "short en",
        "order en",
        "article en",
        "place zh",
        "category en",
        "wordy en",
    ]
    assert "nocat: keys ['en', 'zh'], want category, en, zh" in found
    shared = "en 'small clear plastic cup with a lid' is the most detailed name of ['order', 'article', 'place', "
    assert any(p.startswith(shared) for p in found)


def test_apply_writes_names_and_keeps_the_rest(tmp_path):
    d = tmp_path / "Rigid/cup/00000"
    d.mkdir(parents=True)
    (d / "description.json").write_text(json.dumps({"uuid": "u", "caption": {}, "description": ["a cup"]}))
    names = entry(["plastic cup"], ["塑料杯"])
    table = {"Rigid/cup/00000": names, "Rigid/cup/00001": names}
    assert object_names.apply(tmp_path, table) == ["Rigid/cup/00000"]
    data = json.loads((d / "description.json").read_text(encoding="utf-8"))
    assert data == {"uuid": "u", "caption": {}, "description": ["a cup"], "names": names}


def test_templates_take_names_in_their_language(monkeypatch):
    seeding = types.ModuleType("env.seeding")
    seeding.seed_everywhere = lambda seed: None
    monkeypatch.setitem(sys.modules, "env.seeding", seeding)  # the real one imports torch and warp
    from env.description_manager.desc_manager import DescManager

    table = object_names.load()
    objects = {"src": table["Rigid/measuring_cup/00001"], "dst": table["Rigid/measuring_cup/00000"]}

    class Layout:  # what layout_manager.get_label_descriptions returns: the category, then the names
        def get_label_descriptions(self, env_idx, label=None, inst_name=None, lang=None):
            return [objects[label]["category"][lang], *objects[label][lang]]

    templates = {"en": "Pour the beans from the <src> into the <dst>.", "zh": "把<src>里的豆子倒进<dst>。"}
    for lang, template in templates.items():
        env = types.SimpleNamespace(
            scene_manager=types.SimpleNamespace(layout_manager=Layout()),
            gen_instruction=lambda env_idx, t=template: [t],
        )
        manager = DescManager(1, {"train": 30}, "train")
        manager.initialize(env)
        said = manager.instruction[0]
        assert said and all("<" not in s for s in said)
        pattern = re.escape(template).replace(re.escape("<src>"), "(.+)").replace(re.escape("<dst>"), "(.+)")
        for s in said:
            src, dst = re.fullmatch(pattern, s).groups()
            # the shared "cup" and "measuring cup" never name either cup
            assert src in objects["src"][lang][1:] and dst in objects["dst"][lang][1:], s


def test_aliases_share_names(tmp_path):
    path = tmp_path / "names.json"
    raw = {"Clutter/b/00000": entry(["b"], ["瓶"]), "Articulation/b/00000": "Clutter/b/00000"}
    path.write_text(json.dumps(raw))
    table = object_names.load(path)
    assert table["Articulation/b/00000"] == table["Clutter/b/00000"]
