"""Turn CC0 PBR textures from Poly Haven (api.polyhaven.com) into OmniPBR MDL materials in an Assets/Material tree, for
the RSI table and ground pools (rsi/AGENTS.md): the three official eval seeds use 494 of the 504 benchmark materials.
Merge the folder with scripts/datagen/rsi_assets.py link.

    python scripts/scenes/polyhaven_materials.py --out extra_mat/Material --official Assets/Material [--res 1k]
Picks textures for a floor, a wall or an object (no aerial shots, terrain, roads, bark or roofing) whose name is not an
official material's name (the official set is NVIDIA vMaterials; their textures cannot be compared, 145 have none).
Writes <out>/material_ph_<id>/ph_<id>.mdl with ph_<id>_{BaseColor,ORM,Normal}.jpg (Poly Haven's Diffuse, arm =
AO/rough/metal as ORM, nor_gl) and <out>/polyhaven_materials.json (id, name, authors, licence, source URL,
dimensions in mm). Folders already there are kept, so a rerun resumes.
"""

import argparse
import json
from pathlib import Path
import re

from polyhaven_hdris import API, get

SURFACES = {"floor", "wall", "object"}
SKIP_CATEGORIES = {"terrain", "road", "asphalt", "bark", "roofing"}
MAPS = {"BaseColor": "Diffuse", "ORM": "arm", "Normal": "nor_gl"}  # our suffix: Poly Haven map
MDL = """mdl 1.4;

import ::OmniPBR::OmniPBR;
import ::anno::author;
import ::anno::description;
import ::anno::display_name;
import ::anno::key_words;
import ::anno::version;
import ::tex::gamma_mode;
import ::state::normal;

export material {name}(*)
[[
    ::anno::display_name("Omni PBR "),
    ::anno::description("Omni PBR, supports ORM textures"),
    ::anno::version(1, 0, 0, ""),
    ::anno::author("NVIDIA CORPORATION"),
    ::anno::key_words(string[]("omni", "PBR", "omniverse", "generic"))
]]
 = ::OmniPBR::OmniPBR(
    diffuse_color_constant: color(1.f, 1.f, 1.f),
    diffuse_texture: texture_2d("./{name}_BaseColor.{ext}", ::tex::gamma_srgb),
    albedo_desaturation: 0.f,
    albedo_add: 0.f,
    albedo_brightness: 1.f,
    diffuse_tint: color(1.f, 1.f, 1.f),
    reflection_roughness_constant: 1.f,
    reflection_roughness_texture_influence: 1.f,
    reflectionroughness_texture: texture_2d(),
    metallic_constant: 0.f,
    metallic_texture_influence: 1.f,
    metallic_texture: texture_2d(),
    specular_level: 0.5f,
    enable_ORM_texture: true,
    ORM_texture: texture_2d("./{name}_ORM.{ext}", ::tex::gamma_linear),
    ao_to_diffuse: 0.f,
    ao_texture: texture_2d(),
    enable_emission: false,
    emissive_color: color(0.f, 0.f, 0.f),
    emissive_mask_texture: texture_2d(),
    emissive_intensity: 0.f,
    bump_factor: 1.f,
    normalmap_texture: texture_2d("./{name}_Normal.{ext}", ::tex::gamma_linear),
    detail_bump_factor: 0.3f,
    detail_normalmap_texture: texture_2d(),
    project_uvw: false,
    world_or_object: false,
    uv_space_index: 0,
    texture_translate: 0.f,
    texture_rotate: 0.f,
    texture_scale: 1.f,
    detail_texture_translate: 0.f,
    detail_texture_rotate: 0.f,
    detail_texture_scale: 1.f);
"""


def family(name):
    """'Stone_Wall_02', 'stone wall 2' -> 'stonewall': a numbered variant counts as the same material."""
    return re.sub(r"\d+$", "", re.sub(r"[^a-z0-9]", "", name.lower()))


def select(assets, official_names):
    """Poly Haven texture ids fit for a table top or a floor, minus those named like an official material."""
    taken = {family(n) for n in official_names}
    return sorted(
        key
        for key, v in assets.items()
        if (a := v.get("attributes") or {}).get("surface_use") in SURFACES
        and not a.get("aerial")
        and not SKIP_CATEGORIES & set(v.get("categories") or ())
        and family(key) not in taken
        and family(v.get("name") or "") not in taken
    )


def material_name(key):
    """MDL identifier (= file stem, the loaders take the name from it) of a Poly Haven id."""
    return "ph_" + re.sub(r"\W", "_", key, flags=re.ASCII)


def mdl_text(name, ext="jpg"):
    return MDL.format(name=name, ext=ext)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--official", type=Path, required=True, help="the benchmark's Assets/Material, for the name rule")
    ap.add_argument("--res", default="1k", help="the eval cameras are 640x480")
    ap.add_argument("--format", default="jpg")
    ap.add_argument("--count", type=int, help="only the first N candidates")
    args = ap.parse_args(argv)
    official = [p.stem for p in args.official.glob("material*/*.mdl")]
    assets = json.loads(get(f"{API}/assets?type=textures"))
    picked = select(assets, official)[: args.count]
    print(f"{len(picked)} candidates", flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    record, missing = [], {}
    for key in picked:
        name = material_name(key)
        folder = args.out / f"material_{name}"
        if not (folder / f"{name}.mdl").is_file():
            files = json.loads(get(f"{API}/files/{key}"))
            try:
                urls = {s: files[m][args.res][args.format]["url"] for s, m in MAPS.items()}
            except KeyError as e:
                missing[key] = f"no {e.args[0]} map"
                print(f"{key}: {missing[key]}", flush=True)
                continue
            tmp = args.out / f".tmp_{name}"
            tmp.mkdir(exist_ok=True)
            for suffix, url in urls.items():
                (tmp / f"{name}_{suffix}.{args.format}").write_bytes(get(url))
            (tmp / f"{name}.mdl").write_text(mdl_text(name, args.format), encoding="utf-8")
            tmp.rename(folder)
            print(folder.name, flush=True)
        info = assets[key]
        record.append(
            {
                "dir": folder.name,
                "id": key,
                "name": info.get("name"),
                "authors": info.get("authors"),
                "license": "CC0-1.0",
                "url": f"https://polyhaven.com/a/{key}",
                "dimensions_mm": info.get("dimensions"),
            }
        )
    provenance = {
        "source": "Poly Haven (polyhaven.com) textures",
        "license": "CC0-1.0",
        "res": args.res,
        "dedup": "name rule only: ids whose name (digits stripped) equals an official material's are left out",
        "materials": record,
        "skipped": missing,
    }
    (args.out / "polyhaven_materials.json").write_text(json.dumps(provenance, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
