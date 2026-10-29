"""Instruction names of the Real2Sim objects: object_names.json -> description.json "names".

Every object and every generalization variant has English and Chinese names, ordered from the bare category to a
phrase that singles it out within its family ("cup" ... "tall clear plastic measuring cup marked up to 500 ml"). An
instruction template fills `<label>` with one of them (env/description_manager/desc_manager.py), in the template's
language. object_names.json maps an asset id to {"en": [...], "zh": [...]}, or to another id with the same names
(the twist-cap Articulation bottle of a Clutter bottle).

Generators rewrite description.json without "names": run `apply` after every generation.

Usage:
    python scripts/objects/object_names.py check
    python scripts/objects/object_names.py apply OBJECT_ROOT   # e.g. ovl or Assets/Object/Real2Sim
"""

import argparse
import json
from pathlib import Path
import re

TABLE = Path(__file__).with_name("object_names.json")
LANGS = ("en", "zh")
MIN_NAMES = 5
# a name drops into "pick up the <x>" / "拿起<x>": no article, measure word, final punctuation or place in the scene
BAD = {
    "en": re.compile(r"^(a|an|the)\b|[.!?]$|\b(left|right|front|back|bigger|smaller|larger)\b", re.I),
    "zh": re.compile(r"^(一|这|那)|[。！？]$|左|右|更"),
}


def load(path=TABLE):
    """{asset id: {"en": [...], "zh": [...]}} with aliases resolved."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {oid: raw[v] if isinstance(v, str) else v for oid, v in raw.items()}


def length(name, lang):
    return len(name.split()) if lang == "en" else len(name.replace(" ", ""))


def problems(table):
    """What breaks the conventions above; empty when the table is fine."""
    found, last, seen = [], {lang: {} for lang in LANGS}, set()
    for oid, names in table.items():
        if id(names) in seen:  # an alias
            continue
        seen.add(id(names))
        if set(names) != set(LANGS):
            found.append(f"{oid}: languages {sorted(names)}, want {list(LANGS)}")
            continue
        for lang in LANGS:
            items = names[lang]
            if len(items) < MIN_NAMES or len(set(items)) != len(items):
                found.append(f"{oid} {lang}: want {MIN_NAMES}+ distinct names, got {items}")
            for name in items:
                if not name.strip() or name != name.strip() or BAD[lang].search(name):
                    found.append(f"{oid} {lang}: {name!r} is not a bare noun phrase")
            sizes = [length(n, lang) for n in items]
            if sizes != sorted(sizes):
                found.append(f"{oid} {lang}: not ordered brief to detailed {items}")
            if items:
                last[lang].setdefault(items[-1], []).append(oid)
    for lang in LANGS:
        shared = [(name, ids) for name, ids in last[lang].items() if len(ids) > 1]
        found += [f"{lang} {name!r} is the most detailed name of {ids}" for name, ids in shared]
    return found


def apply(root, table):
    """Write `names` into root/<id>/description.json of every object in the table found under root."""
    done = []
    for oid, names in table.items():
        path = Path(root) / oid / "description.json"
        if path.is_file():
            data = json.loads(path.read_text(encoding="utf-8"))
            data["names"] = names
            path.write_text(json.dumps(data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
            done.append(oid)
    return done


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("check", "apply"))
    parser.add_argument("root", nargs="?", help="object root holding <type>/<category>/<index>/ (apply)")
    args = parser.parse_args()
    table = load()
    found = problems(table)
    if found:
        raise SystemExit("\n".join(found))
    if args.command == "apply":
        if not args.root:
            parser.error("apply needs OBJECT_ROOT")
        done = apply(args.root, table)
        print(f"names written for {len(done)}/{len(table)} objects under {args.root}")
    else:
        print(f"{len(table)} objects ok")


if __name__ == "__main__":
    main()
