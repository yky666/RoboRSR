import json
import re
import sys
import types

import object_names


def test_table_follows_the_conventions():
    assert object_names.problems(object_names.load()) == []


def test_problems_catch_order_articles_and_shared_detail():
    good = ["cup", "plastic cup", "clear plastic cup", "small clear plastic cup", "small clear plastic cup with a lid"]
    zh = ["杯子", "塑料杯", "透明塑料杯", "小号透明塑料杯", "带盖的小号透明塑料杯"]
    assert object_names.problems({"a": {"en": good, "zh": zh}}) == []
    found = object_names.problems(
        {
            "short": {"en": good[:4], "zh": zh},
            "order": {"en": [good[1], good[0], *good[2:]], "zh": zh},
            "article": {"en": ["the cup", *good[1:]], "zh": zh},
            "place": {"en": good, "zh": [*zh[:4], "左边的小号透明塑料杯"]},
        }
    )
    assert [p.split(":")[0] for p in found[:4]] == ["short en", "order en", "article en", "place zh"]
    assert "en 'small clear plastic cup with a lid' is the most detailed name of ['order', 'article', 'place']" in found


def test_apply_writes_names_and_keeps_the_rest(tmp_path):
    d = tmp_path / "Rigid/cup/00000"
    d.mkdir(parents=True)
    (d / "description.json").write_text(json.dumps({"uuid": "u", "caption": {}, "description": ["a cup"]}))
    names = {"en": ["cup"], "zh": ["杯子"]}
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

    class Layout:
        def get_label_descriptions(self, env_idx, label=None, inst_name=None, lang=None):
            return objects[label][lang]

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
            # the shared "cup", "measuring cup", "plastic measuring cup" never name either cup
            assert src in objects["src"][lang][3:] and dst in objects["dst"][lang][3:], s


def test_aliases_share_names(tmp_path):
    path = tmp_path / "names.json"
    raw = {"Clutter/b/00000": {"en": ["b"], "zh": ["瓶"]}, "Articulation/b/00000": "Clutter/b/00000"}
    path.write_text(json.dumps(raw))
    table = object_names.load(path)
    assert table["Articulation/b/00000"] == table["Clutter/b/00000"]
