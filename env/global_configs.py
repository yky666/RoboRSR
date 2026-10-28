import os

current_file = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file)

ROOT_DIR = os.path.join(current_dir, "..")

BENCHMARK = "RoboDojo"
ASSETS_PATH = os.path.join(ROOT_DIR, "Assets")
OBJECTS_PATH = os.path.join(ROOT_DIR, f"Assets/Object/{BENCHMARK}")
OBJECT_LIBRARIES = ("Real2Sim", BENCHMARK)  # Real2Sim twins built in this fork, then the benchmark's own objects
ROBOTS_PATH = os.path.join(ROOT_DIR, "Assets/Robots")
ENV_CONFIG_PATH = os.path.join(ROOT_DIR, "env_cfg")

ENV_REGEX_NAMESPACE = "{ENV_REGEX_NS}"

BATCH_NUM = 10

USD_PATH = os.environ.get("ROBODOJO_USD_ASSET_PREFIX") or None


def object_dir(rel, assets=ASSETS_PATH):
    """<assets>/Object/<library>/<rel> (rel: "Rigid/cup/00003") in the first library that has it, else the benchmark's.

    Ids are unique across the libraries: layouts and task configs name only <type>/<category>/<index>."""
    for library in OBJECT_LIBRARIES:
        path = os.path.join(assets, "Object", library, rel)
        if os.path.exists(path):
            return path
    return os.path.join(assets, "Object", BENCHMARK, rel)
