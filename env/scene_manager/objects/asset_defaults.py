"""Optional per-asset flexible-object defaults; explicit layout values win."""

from collections.abc import Mapping
from copy import deepcopy
import json
from pathlib import Path


def with_asset_defaults(usd_path, instance):
    if not usd_path:
        return instance
    path = Path(usd_path).parent / "runtime_config.json"
    if not path.is_file():
        return instance
    defaults = json.loads(path.read_text())
    if not isinstance(defaults, dict) or set(defaults) - {"physics", "visual"}:
        raise ValueError(f"Invalid asset runtime defaults: {path}")

    def merge(base, override):
        result = deepcopy(base)
        for key, value in override.items():
            if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
                result[key] = merge(result[key], value)
            else:
                result[key] = deepcopy(value)
        return result

    return merge(defaults, instance)
