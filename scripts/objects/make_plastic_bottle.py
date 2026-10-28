#!/usr/bin/env python3
"""Capped plastic water bottles with printed labels as RoboDojo `Clutter` assets, plus twistable-cap twins as
`Articulation` assets.

- plastic_bottle/00000: Real2Sim platform registration real2sim/objects/Clutter/plastic_bottle/00000 (spec.json, by
  徐鹤鸣, 2026-10-23 10:54:15): 农夫山泉 550 ml bottle, transparent, red-and-white label, static friction 0.35,
  开口直径 29.8 mm, 底部直径 64.7 mm (measured). The user added on 2026-10-23: 总高 227 mm without cap, empty mass
  18 g (the registered 328 g included water; modelled empty, cap included), 开口直径 is the outer diameter. Shoulder,
  neck finish, label band, grip grooves and the cap are estimated from the four photos. The registration asked for the
  cap removed; the user asked on 2026-10-23 for a cap that can be twisted, so every bottle now has one.
- plastic_bottle/00001-00005: generalization variants of 00000 (not measured; colour, material, size and shape differ):
  380 ml mini, 1 L tall, frosted rounded-square, green waisted grip bottle, amber wide-mouth juice bottle.
- plastic_bottle/00006: Real2Sim platform registration real2sim/objects/Clutter/plastic_bottle/00006 (spec.json, by
  徐鹤鸣, 2026-10-28 11:08:58): 东方树叶 500 ml square tea bottle, transparent with a label, 33 g with its cap,
  瓶口外径 39.6 mm, 瓶身边长 62.2 mm (measured); 高度 205 mm with the cap (the registration says 20.5 mm, the user
  confirmed 205 on 2026-10-28, and asked for the real cap, removable). Built by TeaBottle from the four photos
  (estimated): chamfered-square body, short shoulder, long wide round neck, clear ribbed cap with a white top, round
  recess in the base; three separate labels (front face, back face, front half of the neck; the side faces are clear)
  whose textures are rectified from the registration photos in real2sim/objects/Clutter/plastic_bottle/00006/reference.
- plastic_bottle/00007-00012: generalization variants of 00006 (not measured; label, colour, material, size and shape
  differ; same construction and finish, so the 00006 cap fits all of them).
- Other labels are textures drawn by bottle_label.py (needs a CJK font, see bottle_label.FONT_DIRS); variants carry
  fictional brands.

Usage:
    python scripts/objects/make_plastic_bottle.py [TYPE_DIR]    # default Assets/Object/Real2Sim/Clutter
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Clutter/plastic_bottle/000{00..12}
    python scripts/objects/check_bottle_cap.py Assets/Object/Real2Sim/Articulation/plastic_bottle/000{00..12}
Writes TYPE_DIR/plastic_bottle/<index>/ and TYPE_DIR/../Articulation/plastic_bottle/<index>/, each {object.usdz,
metadata.json, description.json}. Needs what make_pitcher.py needs, and PIL.

Frame: Z up, metres, origin at bbox centre.
USD (Clutter): /root (RigidBodyAPI) -> visual/model (bottle shell), visual/label (textured band), visual/cap
                                    -> collision/hull_NN (bottom slab + wall staves up to the mouth, cap cylinder last)
USD (Articulation): /root (ArticulationRootAPI) -> body (shell, label, bottle colliders), nut, cap (cap and its
    collider), joints/cap_twist + joints/cap_lift (add_cap_joints); textures/label.png inside the usdz.
TeaBottle (00006-00012): visual/label_front, label_back, label_neck (textures/<name>.png) instead of visual/label, and
    visual/cap_top (the inset top disc) next to visual/cap.
Each bottle is a loft of sections r * ring(b) along Z: b = 1 is the variant's section (circle or rounded square), b = 0
a circle; the shoulder blends from b = 1 into the round neck.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np
from PIL import Image, ImageChops
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bottle_label  # noqa: E402
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (loft, unit_section, metadata)
import measuring_cup_shape as msc  # noqa: E402
from pxr import PhysxSchema  # noqa: E402

TYPE_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Clutter"
CATEGORY = "plastic_bottle"
LABEL_TEX = "textures/label.png"  # inside the usdz, next to object.usdc
LABEL_PX, LABEL_MAX_PX = 1024, 4096  # label texture height; width follows the label's perimeter, capped
LABEL_FRONT_U = 0.57  # artwork roll: its front panel (u 0.38) ends up facing check_object's preview camera (-17 deg)
NUT_MASS, NUT_INERTIA = 1e-4, 1e-8  # kg, kg m^2: the carrier link between cap_twist and cap_lift
CAP_DAMPING = 3e-4  # cap_twist drive damping, N m s / deg: a 0.05 N m twist turns the cap ~half a turn per second
CAP_ARMATURE = 1e-6  # kg m^2 on cap_twist: keeps the light cap and nut stable at the 1/250 s physics step
N_THETA = pc.N_THETA
T_WALL = 0.0004  # visual PET wall, estimated (real 550 ml bottles are ~0.25 mm)
T_COLL = 0.002  # collider wall: thicker than the PET so contacts stay robust; grows inward from the outer surface
STAVE_TOL = 0.0006  # max inner-chord sag of a collision stave (around the perimeter)
LEVEL_TOL = 0.0005  # max deviation of the collider from the outer profile between two collision levels (along Z)
MAX_COLS = 28  # perimeter columns per stave: 2 * 29 outer + 4 inner points stays <= 64 (PhysX GPU convex limit)
BOTTOM_SIDES = 24
STATIC_FRICTION = 0.35  # measured (spec static_friction)
DYNAMIC_FRICTION = 0.35  # estimated: not measured, same as static

# Bottle 00000. Radii are outer unless named otherwise; heights from the outside bottom.
NONGFU = dict(
    H=0.227,  # 总高, measured (2026-10-23, from the user): outside bottom to mouth, without cap
    R=0.03235,  # 底部直径 / 2, measured 64.7 mm: outer body diameter just above the bottom fillet
    SQ=2.0,  # section superellipse exponent: 2 = round
    FIL=0.004, FOOT=0.022, DOME=0.006,  # bottom edge fillet, inner edge of the standing ring, push-up height: estimated
    Z_SH=0.158, SH_P=1.3,  # shoulder start (body full width below), shoulder fullness exponent: estimated
    R_NECK=0.013,  # neck under the support ring, estimated (28 mm-class finish)
    NECK_H=0.005, RING_T=0.0015, R_RING=0.0165,  # neck height, support ring thickness and radius: estimated
    FINISH_H=0.0155,  # support ring top to mouth, estimated
    R_THREAD=0.0149,  # 开口直径 / 2, measured 29.8 mm: outer diameter of the threaded finish (thread crests)
    R_CORE=0.0138, R_BORE=0.011,  # finish root and bore radius: estimated
    THREADS=(0.0085, 0.0045),  # thread crest rings, depth below the mouth: estimated
    GROOVES=(0.008, 0.013, 0.018), GROOVE_D=0.0010, GROOVE_W=0.003,  # grip grooves under the label: estimated
    WAIST=None,  # (depth, z0, z1) of an hourglass waist
    LABEL=(0.026, 0.127, 0.60), RECESS=0.0006,  # label z0, z1, bottom-colour share; label panel recess: estimated
    LABEL_RGB=((0.42, 0.015, 0.02), (0.90, 0.90, 0.88)),  # bottom (red), top (white); linear RGB
    TINT=(0.97, 0.98, 0.98), FROST=0.0,  # FROST: milky share of a frosted wall, 0 = clear
    MASS=0.018,  # measured empty (2026-10-23, from the user; the registered 328 g was the bottle with water)
    # screw cap, all estimated from the photos: red, knurled, tamper band on the bead; sits from the support ring up
    CAP_RGB=(0.50, 0.012, 0.015), CAP_T=0.0016, CAP_TOP=0.0015,  # wall beyond the thread crests, top above the mouth
    CAP_RIBS=64, RIB_D=0.0003, BAND_H=0.0045,  # knurl count and depth, tamper band height
    CAP_MASS=0.002, PITCH=0.0027, OPEN_TURNS=2.0,  # part of MASS; thread pitch (28 mm PCO 1881 class); turns to loosen
    ART=dict(
        brand="农夫山泉", latin="NONGFU SPRING", tagline="天然水源 · 健康生活", product="饮用天然水", volume="550ml",
        note="天然弱碱性水", picture="landscape", table="water", trim=(0.01, 0.10, 0.05), ean="6 900000 123457",
        facts=dict(ingredients="水", shelf="24个月", standard="GB 19298"),
    ),
    WORDS=dict(
        color=["transparent", "red and white label", "red cap"],
        material=["plastic", "clear PP", "thin-walled plastic"],
        name=["water bottle", "plastic bottle", "Nongfu Spring bottle"],
        shape=["round cylindrical body", "rounded shoulder", "red screw cap"],
        function=["holding drinking water", "being picked up and placed", "recycling"],
        look="clear round water bottle with a red and white label and a red cap",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00001-00005: overrides of NONGFU, all unmeasured (estimated)
    dict(
        H=0.168, R=0.0290, Z_SH=0.110, GROOVES=(0.007, 0.012), LABEL=(0.030, 0.092, 0.55),
        LABEL_RGB=((0.02, 0.09, 0.40), (0.90, 0.90, 0.88)), CAP_RGB=(0.02, 0.10, 0.45),
        ART=dict(
            brand="清泉", latin="QING QUAN", tagline="清澈每一天", product="饮用纯净水", volume="380ml",
            note="多重过滤 纯净安心", picture="landscape", table="water", trim=(0.01, 0.03, 0.15),
            ean="6 900000 238014", facts=dict(ingredients="水", shelf="18个月", standard="GB 17323"),
        ),
        WORDS=dict(
            color=["transparent", "blue and white label", "blue cap"],
            material=["plastic", "clear PET", "thin plastic"],
            name=["small water bottle", "mini bottle", "380 ml bottle"],
            shape=["short round body", "rounded shoulder", "blue screw cap"],
            function=["holding drinking water", "carrying in a bag", "being picked up and placed"],
            look="small clear water bottle with a blue and white label",
        ),
    ),
    dict(
        H=0.262, R=0.0375, Z_SH=0.185, SH_P=1.1, GROOVES=(0.010, 0.016, 0.022, 0.028, 0.034),
        LABEL=(0.050, 0.150, 0.50), LABEL_RGB=((0.02, 0.22, 0.06), (0.90, 0.90, 0.88)), TINT=(0.84, 0.92, 0.98),
        CAP_RGB=(0.03, 0.25, 0.06),
        ART=dict(
            brand="山涧", latin="MOUNTAIN BROOK", tagline="源自深山的清冽", product="天然矿泉水", volume="1L",
            note="低钠 · 含偏硅酸", picture="landscape", table="water", trim=(0.01, 0.06, 0.02),
            ean="6 900000 410075", facts=dict(ingredients="水", shelf="24个月", standard="GB 8537"),
        ),
        WORDS=dict(
            color=["light blue tinted", "green and white label", "transparent"],
            material=["plastic", "blue-tinted PET", "thin plastic"],
            name=["large water bottle", "1 litre bottle", "family-size bottle"],
            shape=["tall round body", "long sloping shoulder", "green screw cap"],
            function=["holding drinking water", "sharing water", "refilling cups"],
            look="tall light blue tinted water bottle with a green and white label",
        ),
    ),
    dict(
        H=0.215, R=0.0310, SQ=4.0, Z_SH=0.138, GROOVES=(), LABEL=(0.040, 0.118, 0.50),
        LABEL_RGB=((0.012, 0.012, 0.014), (0.80, 0.22, 0.02)), TINT=(0.95, 0.95, 0.95), FROST=0.35,
        CAP_RGB=(0.015, 0.015, 0.015),
        ART=dict(
            brand="橙动", latin="ORANGE RUSH", tagline="补充电解质", product="运动饮料", volume="500ml",
            note="运动后适量饮用", picture="stripes", table="nutrition", trim=(0.80, 0.22, 0.02),
            ean="6 900000 552813",
            facts=dict(ingredients="水、白砂糖、柠檬酸、食用盐", shelf="12个月", standard="GB/T 10789"),
        ),
        WORDS=dict(
            color=["frosted white", "black and orange label", "black cap"],
            material=["plastic", "frosted PET", "matte plastic"],
            name=["square bottle", "frosted drink bottle", "plastic bottle"],
            shape=["rounded square body", "round neck", "black screw cap"],
            function=["holding drinks", "standing on a shelf", "being picked up and placed"],
            look="frosted rounded-square plastic bottle with a black and orange label",
        ),
    ),
    dict(
        H=0.230, R=0.0335, Z_SH=0.150, SH_P=1.0, GROOVES=(), WAIST=(0.0045, 0.035, 0.110),
        LABEL=(0.112, 0.146, 0.50), LABEL_RGB=((0.015, 0.03, 0.15), (0.85, 0.60, 0.02)), TINT=(0.66, 0.88, 0.70),
        CAP_RGB=(0.85, 0.60, 0.02),
        ART=dict(
            brand="汽泡", latin="SPARKLE", tagline="清爽气泡", product="柠檬味汽水", volume="500ml",
            note="冰镇后口感更佳", picture="bubbles", table="nutrition", trim=(0.85, 0.60, 0.02),
            ean="6 900000 667302",
            facts=dict(ingredients="水、白砂糖、二氧化碳、柠檬汁", shelf="12个月", standard="GB/T 10792"),
        ),
        WORDS=dict(
            color=["green tinted", "blue and yellow label", "yellow cap"],
            material=["plastic", "green PET", "thin plastic"],
            name=["soda bottle", "green plastic bottle", "grip bottle"],
            shape=["hourglass waist", "rounded shoulder", "yellow screw cap"],
            function=["holding soft drinks", "being gripped at the waist", "being picked up and placed"],
            look="green tinted soda bottle with an hourglass grip waist",
        ),
    ),
    dict(
        H=0.150, R=0.0340, Z_SH=0.100, SH_P=1.0, R_NECK=0.0205, R_RING=0.0235, FINISH_H=0.013,
        R_THREAD=0.0210, R_CORE=0.0200, R_BORE=0.0185, THREADS=(0.0065, 0.003), GROOVES=(0.008, 0.013),
        LABEL=(0.028, 0.090, 0.60), LABEL_RGB=((0.75, 0.16, 0.01), (0.90, 0.89, 0.84)), TINT=(0.86, 0.60, 0.28),
        CAP_RGB=(0.85, 0.85, 0.82),
        ART=dict(
            brand="果园", latin="ORCHARD", tagline="鲜榨原汁", product="NFC 橙汁", volume="300ml",
            note="非浓缩还原", picture="fruit", table="nutrition", trim=(0.75, 0.16, 0.01),
            ean="6 900000 781256", facts=dict(ingredients="橙汁", shelf="6个月", standard="GB/T 31121"),
        ),
        WORDS=dict(
            color=["amber tinted", "orange and white label", "white cap"],
            material=["plastic", "amber PET", "thick plastic"],
            name=["juice bottle", "wide-mouth bottle", "amber plastic bottle"],
            shape=["short wide body", "wide white screw cap", "short shoulder"],
            function=["holding juice", "storing small items", "being picked up and placed"],
            look="short amber wide-mouth juice bottle with an orange and white label",
        ),
    ),
]  # fmt: skip

# Bottle 00006 (东方树叶), all from the four registration photos unless marked measured. Section: a square with large
# chamfered corners (measuring_cup_shape.rounded_octagon), R is half the width across the flats; labels sit only on
# the front (+X) and back (-X) flat faces and on the front half of the round neck, the side faces are clear.
DONGFANG = dict(
    NONGFU,
    TEA=True,
    H=0.2035,  # mouth height: 高度 205 mm (measured, with the cap) minus the cap top CAP_TOP above the mouth
    R=0.0311,  # 瓶身边长 / 2, measured 62.2 mm across the flats
    CHAMFER=0.0084, ROUND=0.0012,  # corner cut leg and edge rounding: ~13 mm chamfer facets, ~43 mm flats (estimated)
    FLUTES=(3, 0.0003, 0.0016, 0.0032),  # fine vertical grooves per chamfer facet: count, depth, width, pitch (est.)
    FIL=0.005, FOOT=0.019, DOME=0.006,  # rounded bottom edge, circular recess radius and push-up height (estimated)
    Z_SH=0.127, Z_NECK=0.141, SH_P=1.0,  # straight body top, short shoulder into the round neck (estimated)
    R_NECK=0.0208, RING_T=0.0015, R_RING=0.0212,  # long wide neck (~41.6 mm), support ring under the cap (estimated)
    FINISH_H=0.019,  # support ring top to mouth (38 mm wide-mouth finish), estimated
    R_THREAD=0.0198,  # 瓶口外径 / 2, measured 39.6 mm: outer diameter of the threaded finish (thread crests)
    R_CORE=0.0188, R_BORE=0.0170, THREADS=(0.0040, 0.0075, 0.0110),  # estimated
    GROOVES=(), WAIST=None,
    PANEL=(0.006, 0.106, 0.021, 0.0045),  # front / back labels: z0, z1, half-width, corner cut (estimated)
    NECK_LABEL=(0.148, 0.176, 100.0),  # neck label z0, z1 and half-angle (deg) about the front (estimated)
    TINT=(0.97, 0.98, 0.98), FROST=0.0,
    MASS=0.033,  # measured, cap included
    CAP_T=0.0012, CAP_TOP=0.0015, CAP_RIBS=120, RIB_D=0.0003, BAND_H=0.005,  # 38 mm cap (estimated)
    CAP_CLEAR=True, CAP_RGB=(0.90, 0.92, 0.92), CAP_TOP_RGB=(0.86, 0.86, 0.85),  # clear ribbed skirt, white top
    CAP_MASS=0.0035, PITCH=0.0045, OPEN_TURNS=1.25,  # part of MASS, thread pitch, turns to loosen
    BRIM=(520, 640),  # ml: a 500 ml bottle with its headspace
    MEASURE="square", FRICTION=None,  # no measured friction: runtime defaults apply
    ART="photo",  # label textures rectified from the registration photos (PHOTO_LABELS)
    WORDS=dict(
        color=["transparent", "white cap top", "red horse label"],
        material=["plastic", "clear PET", "thin-walled plastic"],
        name=["tea bottle", "plastic bottle", "Oriental Leaf bottle"],
        shape=["chamfered square body", "long round neck", "screw cap"],
        function=["holding tea", "being picked up and placed", "being opened"],
        look="clear chamfered square tea bottle with front and back labels, a green neck label and a white-top cap",
    ),
)  # fmt: skip
REFERENCE = Path(__file__).resolve().parents[2] / "real2sim/objects/Clutter/plastic_bottle/00006/reference"
PHOTO_LABELS = dict(  # registration photos (1702 x 1276) and where the labels are in them, px (estimated by eye)
    front="64fcd19c8ba09a5e05c79f3d88b7eb80.jpg",  # square-on to the front face, from ~25 deg above
    back="a9a371200214e151cf59f91afd537127.jpg",  # back face, bottle leaning
    front_quad=[(660, 655), (910, 655), (906, 1229), (664, 1229)],  # front label: TL, TR, BR, BL
    back_quad=[(668, 694), (1046, 670), (1066, 1218), (730, 1246)],
    neck=dict(cx=790, rx=121, top=(367, 316), bottom=(512, 459), seen=66.0),  # in the front photo, see photo_band
)
TEA_PX = 10000  # label texture pixels per metre


def tea_art(brand, latin, product, volume, picture, paper, ink, accent, ean, ingredients, claim):
    return dict(
        brand=brand, latin=latin, product=product, volume=volume, picture=picture, paper=paper, ink=ink,
        accent=accent, ean=ean, claim=claim, note="无糖 · 0能量 · 0脂肪", tagline="冷泡原叶 清爽回甘",
        facts=dict(ingredients=ingredients, kind="原味茶饮料", shelf="12个月", standard="GB/T 21733", sodium="8 mg"),
    )  # fmt: skip


def tea_words(color, material, name, shape, look):
    return dict(
        color=color, material=material, name=name, shape=shape, look=look,
        function=["holding tea", "being picked up and placed", "being opened"],
    )  # fmt: skip


DONGFANG_VARIANTS = [  # generalization variants 00007-00012: overrides of DONGFANG, all unmeasured (estimated)
    dict(
        H=0.2255, R=0.0320, CHAMFER=0.0090, Z_SH=0.146, Z_NECK=0.160, PANEL=(0.007, 0.124, 0.0213, 0.0045),
        NECK_LABEL=(0.167, 0.198, 100.0), CAP_CLEAR=False, CAP_RGB=(0.10, 0.40, 0.12), CAP_TOP_RGB=(0.16, 0.50, 0.18),
        ART=tea_art("茗泉", "MING QUAN", "茉莉清茶", "600ml", "leaves", (0.80, 0.88, 0.74), (0.03, 0.20, 0.05),
                    (0.04, 0.30, 0.08), "6 900000 911423", "水、茉莉花茶", "茶多酚含量≥200mg/kg"),
        WORDS=tea_words(["transparent", "light green labels", "green cap"],
                        ["plastic", "clear PET", "thin plastic"],
                        ["tall tea bottle", "600 ml bottle", "green tea bottle"],
                        ["tall chamfered square body", "long round neck", "green screw cap"],
                        "tall clear chamfered square jasmine tea bottle with light green labels"),
    ),
    dict(
        H=0.1725, R=0.0290, CHAMFER=0.0075, Z_SH=0.100, Z_NECK=0.113, PANEL=(0.006, 0.088, 0.0195, 0.004),
        NECK_LABEL=(0.119, 0.146, 100.0), TINT=(0.90, 0.75, 0.50), CAP_CLEAR=False, CAP_RGB=(0.30, 0.12, 0.03),
        CAP_TOP_RGB=(0.36, 0.16, 0.05),
        ART=tea_art("岩韵", "YAN YUN", "乌龙茶", "400ml", "stripes", (0.85, 0.78, 0.62), (0.30, 0.12, 0.03),
                    (0.45, 0.20, 0.05), "6 900000 913502", "水、乌龙茶", "茶多酚含量≥300mg/kg"),
        WORDS=tea_words(["amber tinted", "cream and brown labels", "brown cap"],
                        ["plastic", "amber PET", "thin plastic"],
                        ["oolong tea bottle", "400 ml bottle", "short tea bottle"],
                        ["short chamfered square body", "round neck", "brown screw cap"],
                        "short amber chamfered square oolong tea bottle with cream and brown labels"),
    ),
    dict(
        R=0.0315, CHAMFER=0.0030, ROUND=0.0060, FLUTES=(0, 0.0, 0.001, 0.001), CAP_TOP_RGB=(0.55, 0.04, 0.06),
        ART=tea_art("荔香", "LI XIANG", "荔枝红茶", "500ml", "fruit", (0.92, 0.84, 0.82), (0.50, 0.03, 0.05),
                    (0.60, 0.05, 0.08), "6 900000 915277", "水、红茶、荔枝汁", "茶多酚含量≥160mg/kg"),
        WORDS=tea_words(["transparent", "pink and red labels", "red cap top"],
                        ["plastic", "clear PET", "thin plastic"],
                        ["lychee tea bottle", "black tea bottle", "plastic bottle"],
                        ["rounded square body", "long round neck", "clear screw cap"],
                        "clear rounded square lychee black tea bottle with red labels"),
    ),
    dict(
        CHAMFER=0.0060, ROUND=0.0030, FROST=0.3, TINT=(0.95, 0.95, 0.95), CAP_CLEAR=False,
        CAP_RGB=(0.05, 0.05, 0.05), CAP_TOP_RGB=(0.08, 0.08, 0.08),
        ART=tea_art("红袍", "HONG PAO", "大红袍", "500ml", "stripes", (0.80, 0.70, 0.30), (0.05, 0.05, 0.05),
                    (0.08, 0.08, 0.08), "6 900000 917046", "水、岩茶", "茶多酚含量≥250mg/kg"),
        WORDS=tea_words(["frosted white", "black and gold labels", "black cap"],
                        ["plastic", "frosted PET", "matte plastic"],
                        ["frosted tea bottle", "rock tea bottle", "plastic bottle"],
                        ["soft chamfered square body", "long round neck", "black screw cap"],
                        "frosted soft-cornered square tea bottle with black and gold labels"),
    ),
    dict(
        H=0.2135, R=0.0305, CHAMFER=0.0115, Z_SH=0.130, Z_NECK=0.146, FLUTES=(2, 0.0003, 0.0016, 0.0045),
        PANEL=(0.010, 0.104, 0.017, 0.003), NECK_LABEL=(0.152, 0.185, 100.0), CAP_CLEAR=False,
        CAP_RGB=(0.85, 0.75, 0.10), CAP_TOP_RGB=(0.90, 0.80, 0.15),
        ART=tea_art("柠檬时光", "LEMON TIME", "柠檬茶", "500ml", "fruit", (0.92, 0.90, 0.55), (0.03, 0.15, 0.45),
                    (0.05, 0.25, 0.60), "6 900000 919318", "水、红茶、柠檬汁", "维生素C≥10mg/100mL"),
        WORDS=tea_words(["transparent", "yellow and blue labels", "yellow cap"],
                        ["plastic", "clear PET", "thin plastic"],
                        ["lemon tea bottle", "octagonal bottle", "plastic bottle"],
                        ["octagonal body", "long round neck", "yellow screw cap"],
                        "clear octagonal lemon tea bottle with yellow and blue labels"),
    ),
    dict(
        H=0.1505, R=0.0270, CHAMFER=0.0070, Z_SH=0.085, Z_NECK=0.097, PANEL=(0.006, 0.076, 0.018, 0.004),
        NECK_LABEL=(0.102, 0.124, 100.0), CAP_TOP_RGB=(0.95, 0.62, 0.68),
        ART=tea_art("桃屿", "TAO YU", "白桃乌龙", "300ml", "bubbles", (0.96, 0.84, 0.84), (0.70, 0.20, 0.30),
                    (0.80, 0.30, 0.40), "6 900000 921564", "水、乌龙茶、白桃汁", "茶多酚含量≥180mg/kg"),
        WORDS=tea_words(["transparent", "pink labels", "pink cap top"],
                        ["plastic", "clear PET", "thin plastic"],
                        ["mini tea bottle", "300 ml bottle", "peach tea bottle"],
                        ["small chamfered square body", "round neck", "clear screw cap"],
                        "small clear chamfered square peach oolong tea bottle with pink labels"),
    ),
]  # fmt: skip

FAMILIES = [  # first index, base registration, variants
    (0, dict(NONGFU, BRIM=(550, 620), MEASURE="round", FRICTION=(STATIC_FRICTION, DYNAMIC_FRICTION)), VARIANTS),
    (6, DONGFANG, DONGFANG_VARIANTS),
]


class Bottle:
    """Profile of one bottle: outer radius r(z) and section blend b(z), visual and collider."""

    def __init__(self, g):
        self.g = g
        self.sec = pc.unit_section(g["SQ"])
        self.z_ring0 = g["H"] - g["FINISH_H"] - g["RING_T"]
        self.z_ring1 = g["H"] - g["FINISH_H"]
        self.z_neck = self.z_ring0 - g["NECK_H"]
        assert g["Z_SH"] < self.z_neck, "shoulder must end below the neck"
        assert g["R_BORE"] <= g["R_NECK"] - T_COLL + 1e-9, "neck too thin for the collider wall"

    def ring(self, r, b):
        rho = np.linalg.norm(self.sec, axis=1)
        return (r * ((1 - b) + b * rho) / rho)[:, None] * self.sec

    def body(self, z):
        """Outer body envelope (with the waist, without grooves and label recess)."""
        g = self.g
        r = g["R"]
        if g["WAIST"]:
            d, z0, z1 = g["WAIST"]
            if z0 < z < z1:
                r -= d * np.sin(np.pi * (z - z0) / (z1 - z0)) ** 2
        return r

    def visual_body(self, z):
        g = self.g
        r = self.body(z)
        for zc in g["GROOVES"]:
            if abs(z - zc) < g["GROOVE_W"] / 2:
                r -= g["GROOVE_D"] * np.cos(np.pi * (z - zc) / g["GROOVE_W"]) ** 2
        l0, l1, _ = g["LABEL"]
        ramp = min(z - (l0 - 0.004), (l1 + 0.004) - z) / 0.002
        return r - g["RECESS"] * float(np.clip(ramp, 0, 1))

    def shoulder(self, t):
        g = self.g
        r0 = self.body(g["Z_SH"])
        r = g["R_NECK"] + (r0 - g["R_NECK"]) * (1 + np.cos(np.pi * t ** g["SH_P"])) / 2
        return r, 1 - t * t * (3 - 2 * t)

    def envelope(self, z):
        """Collider outer radius and section blend at z (fillet top up to the shoulder end)."""
        g = self.g
        if z <= g["Z_SH"]:
            return self.body(z), 1.0
        return self.shoulder((z - g["Z_SH"]) / (self.z_neck - g["Z_SH"]))

    def body_samples(self):
        g = self.g
        z = list(np.arange(g["FIL"], g["Z_SH"], 0.002)[1:]) + [g["Z_SH"]]
        windows = [(zc - g["GROOVE_W"] / 2, zc + g["GROOVE_W"] / 2) for zc in g["GROOVES"]]
        l0, l1, _ = g["LABEL"]
        windows += [(l0 - 0.004, l0 - 0.002), (l1 + 0.002, l1 + 0.004)]
        if g["WAIST"]:
            windows.append(g["WAIST"][1:])
        for a, b in windows:
            z += list(np.linspace(a, b, max(int((b - a) / 0.00025), 2) + 1))
        return np.unique(np.round([v for v in z if g["FIL"] < v <= g["Z_SH"]], 7))

    def profiles(self):
        """Outer (bottom pole -> mouth lip) and inner (lip -> bottom pole) profiles of the shell as [(r, z, b)]."""
        g = self.g
        H, R, FIL, FOOT, DOME = g["H"], g["R"], g["FIL"], g["FOOT"], g["DOME"]
        dome = [(s * FOOT, DOME * (1 - s * s) ** 2) for s in (0.25, 0.5, 0.75)]
        t = np.linspace(0, 1, 25)[1:]
        sh = [(*self.shoulder(v), g["Z_SH"] + v * (self.z_neck - g["Z_SH"])) for v in t]
        zs = self.body_samples()

        outer = [(0.0, DOME, 1.0)] + [(r, z, 1.0) for r, z in dome]
        outer += [(r, z, 1.0) for r, z in pc.fillet(R, FIL)]
        outer += [(self.visual_body(z), z, 1.0) for z in zs]
        outer += [(r, z, b) for r, b, z in sh]
        z0, z1, rn, rr, rc, rt = self.z_ring0, self.z_ring1, g["R_NECK"], g["R_RING"], g["R_CORE"], g["R_THREAD"]
        finish = [(rn, z0)] * 2 + [(rr, z0)] * 2 + [(rr, z1)] * 2 + [(rc, z1)] * 2
        finish += [(rc, z1 + 0.002), (rt, z1 + 0.003), (rt, z1 + 0.004), (rc, z1 + 0.005)]  # tamper-band bead
        for d in g["THREADS"]:
            zt = H - d
            assert zt - 0.001 > z1 + 0.005, "thread overlaps the bead"
            finish += [(rc, zt - 0.001), (rt, zt - 0.0003), (rt, zt + 0.0003), (rc, zt + 0.001)]
        rb = g["R_BORE"]
        finish += [(rc, H - 0.001), (rc - 0.0004, H - 0.0001), ((rc + rb) / 2, H), (rb + 0.0004, H - 0.0001)]
        finish += [(rb, H - 0.001)]
        outer += [(r, z, 0.0) for r, z in finish]

        inner = [(rb, self.z_neck, 0.0)] * 2 + [(rn - T_WALL, self.z_neck, 0.0)]
        inner += [(r - T_WALL, z, b) for r, b, z in sh[::-1]]
        inner += [(self.visual_body(z) - T_WALL, z, 1.0) for z in zs[::-1]]
        inner += [(r, z, 1.0) for r, z in mp.arc(R - FIL, FIL, FIL - T_WALL, 0, -np.pi / 2, 6)]
        inner += [(FOOT, T_WALL, 1.0)] + [(r, z + T_WALL, 1.0) for r, z in dome[::-1]] + [(0.0, DOME + T_WALL, 1.0)]
        return outer, inner

    def loft(self, profile):
        return pc.loft([(0.0, z) if r == 0 else (self.ring(r, b), z) for r, z, b in profile])

    def label(self):
        """Label band (thin closed shell over the recess), its per-face-vertex UVs (u around from +X, v up) and the
        texture size in pixels that keeps the artwork's aspect on the bottle."""
        l0, l1, _ = self.g["LABEL"]
        z = np.linspace(l0, l1, max(int((l1 - l0) / 0.002), 1) + 1)
        out = [(self.visual_body(v) + 0.00025, v, 1.0) for v in z]
        inn = [(self.visual_body(v) + 0.00005, v, 1.0) for v in z[::-1]]
        pts, faces = self.loft(out + [out[-1]] + inn + [inn[-1], out[0]])
        uv = []
        for f in faces:  # no poles: vertex i is column i % N_THETA of its ring, rings are uniform by arc length
            col = np.array(f) % N_THETA
            if np.ptp(col) > N_THETA / 2:  # the face straddles the seam at +X
                col[col < N_THETA / 2] += N_THETA
            uv += [(c / N_THETA, (pts[i, 2] - l0) / (l1 - l0)) for c, i in zip(col, f)]
        ring = out[len(out) // 2]
        ring = self.ring(ring[0], 1.0)
        perimeter = np.linalg.norm(np.diff(np.vstack([ring, ring[:1]]), axis=0), axis=1).sum()
        w, h = LABEL_PX * perimeter / (l1 - l0), LABEL_PX
        k = min(1.0, LABEL_MAX_PX / w)
        return (pts, faces), np.array(uv), (int(w * k) // 4 * 4, int(h * k))

    def cap(self):
        """Screw cap from the support ring to above the mouth: knurled skirt, tamper band with its bridge slit, rounded
        top edge; hollow over the finish (its underside closes the bore at the mouth). Returns visual (pts, faces) and
        a 24-sided convex collider."""
        g = self.g
        z0, z1, zb = self.z_ring1 + 0.0006, g["H"] + g["CAP_TOP"], self.z_ring1 + 0.0006 + g["BAND_H"]
        rc, ri = g["R_THREAD"] + g["CAP_T"], g["R_THREAD"] + 0.0002
        knurl = g["RIB_D"] * (1 - np.abs(np.cos(np.pi * g["CAP_RIBS"] * np.arange(N_THETA) / N_THETA)))

        def rd(r):
            return self.ring(r, 0.0)

        ribbed = rd(rc) * ((rc - knurl) / rc)[:, None]
        profile = [(0.0, g["H"])] + [(rd(ri), g["H"])] * 2 + [(rd(ri), z0)] * 2 + [(rd(rc - 0.0003), z0)] * 2
        profile += [(rd(rc - 0.0003), zb - 0.0004), (rd(rc - 0.0008), zb - 0.0002), (rd(rc - 0.0008), zb + 0.0002)]
        profile += [(rd(rc - 0.0003), zb + 0.0004), (ribbed, zb + 0.0008), (ribbed, z1 - 0.0012)]
        profile += [(rd(rc - 0.0005), z1 - 0.0003)] + [(rd(rc - 0.0012), z1)] * 2 + [(0.0, z1)]
        step = N_THETA // BOTTOM_SIDES
        hull = np.array([(*p, z) for z in (z0, z1) for p in rd(rc)[::step]])
        return pc.loft(profile), hull

    def collision_levels(self):
        """Greedy Z levels from the fillet top to the shoulder end: the collider follows the outer envelope within
        LEVEL_TOL between consecutive levels."""
        g = self.g
        zs = np.unique(np.r_[np.arange(g["FIL"], self.z_neck, 0.001), g["Z_SH"], self.z_neck])
        rings = [self.ring(*self.envelope(z)) for z in zs]
        levels, i = [0], 0
        while i < len(zs) - 1:
            j = i + 1
            while j + 1 < len(zs):
                k = np.arange(i + 1, j + 1)
                w = ((zs[k] - zs[i]) / (zs[j + 1] - zs[i]))[:, None, None]
                lerp = (1 - w) * rings[i] + w * rings[j + 1]
                if np.max(np.linalg.norm(np.stack([rings[m] for m in k]) - lerp, axis=2)) > LEVEL_TOL:
                    break
                j += 1
            levels.append(j)
            i = j
        return zs[levels]

    def hulls(self):
        g = self.g
        step = N_THETA // BOTTOM_SIDES
        slab = [(*p, 0.0) for p in self.ring(g["R"] - g["FIL"], 1.0)[::step]]
        slab += [(*p, g["FIL"]) for p in self.ring(self.body(g["FIL"]), 1.0)[::step]]
        hulls = [np.array(slab)]
        levels = []
        for z in self.collision_levels():
            r, b = self.envelope(z)
            levels.append((z, self.ring(r, b), self.ring(r - T_COLL, b)))
        hulls += wall_staves(levels)
        bore = self.ring(g["R_BORE"], 0.0)
        for z0, z1, r in (
            (self.z_neck, self.z_ring0, g["R_NECK"]),
            (self.z_ring0, self.z_ring1, g["R_RING"]),
            (self.z_ring1, g["H"], g["R_THREAD"]),
        ):
            hulls += wall_staves([(z0, self.ring(r, 0.0), bore), (z1, self.ring(r, 0.0), bore)])
        return hulls

    def cavity(self, z):
        """Collider inner wall at z."""
        if z <= self.z_neck:
            r, b = self.envelope(z)
            return self.ring(r - T_COLL, b)
        return self.ring(self.g["R_BORE"], 0.0)

    def brim_ml(self):
        """Visual inner volume from the bottom to the mouth (push-up ignored)."""
        g = self.g
        z = np.linspace(T_WALL, g["H"], 1200)
        a = []
        for v in z:
            if v > self.z_neck:
                ring = self.ring(g["R_BORE"], 0.0)
            elif v > g["Z_SH"]:
                r, b = self.envelope(v)
                ring = self.ring(r - T_WALL, b)
            else:
                ring = self.ring(self.visual_body(max(v, g["FIL"])) - T_WALL, 1.0)
            a.append(pc.section_area(ring))
        a = np.array(a)
        return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def wall_staves(levels):
    """pc.wall_staves with STAVE_TOL and at most MAX_COLS perimeter columns per stave."""
    hulls = []
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = [0]
        while cuts[-1] < N_THETA:
            a, b = cuts[-1], cuts[-1] + 1
            while b < N_THETA and b - a < MAX_COLS and max(pc.chord_sag(r, a, b + 1) for r in (i0, i1)) <= STAVE_TOL:
                b += 1
            cuts.append(b)
        for a, b in zip(cuts, cuts[1:]):
            idx = np.arange(a, b + 1) % N_THETA
            ring = [(*p, z) for r, z in ((o0, z0), (o1, z1)) for p in r[idx]]
            hulls.append(np.array(ring + [(*p, z) for r, z in ((i0, z0), (i1, z1)) for p in r[idx[[0, -1]]]]))
    return hulls


def tea_section(g):
    """Chamfered square of half-width 1 at the face centres (as measuring_cup_shape.rounded_octagon), N_THETA points
    CCW from the +X face centre, three times denser off the flat faces so the chamfer facets and their flutes are
    resolved. Also returns the flutes: per point, the inward displacement (m) at full depth."""
    a = g["R"]
    w, c, r = 1 - g["ROUND"] / a, g["CHAMFER"] / a, g["ROUND"] / a
    corner = np.array([(w, w - c), (w - c, w)])
    verts = np.concatenate([msc._rot(corner, k) for k in range(4)])
    phi = np.linspace(0, np.pi / 4, 64)
    curve = np.concatenate(
        [v + r * np.c_[np.cos(phi + k * np.pi / 4), np.sin(phi + k * np.pi / 4)] for k, v in enumerate(verts)]
    )
    closed = np.vstack([curve, curve[:1]])
    flat = np.max(np.abs(closed), axis=1) > 1 - 1e-9
    weight = np.where(flat[1:] & flat[:-1], 1.0, 3.0) * np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.r_[0, np.cumsum(weight)]
    u = (s[-1] - (w - c) + np.linspace(0, s[-1], N_THETA, endpoint=False)) % s[-1]  # (1, 0) is w - c before curve[0]
    sec = np.c_[np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])]
    n, depth, width, pitch = g["FLUTES"]
    normal = np.sign(sec) / np.sqrt(2)  # chamfer facet normals, per quadrant
    along = (sec[:, 1] * normal[:, 0] - sec[:, 0] * normal[:, 1]) * a  # position along the facet from its centre, m
    on_facet = np.all(np.abs(sec) < w - 1e-6, axis=1)  # straight chamfer part, not the rounded edges
    d = np.zeros(N_THETA)
    for k in range(n):
        x = along - (k - (n - 1) / 2) * pitch
        d += np.where(np.abs(x) < width / 2, depth * np.cos(np.pi * x / width) ** 2, 0.0)
    return sec, (d * on_facet)[:, None] * normal


def orient(pts, face, outward):
    """face with its vertex order flipped if its normal points against `outward`."""
    p = pts[face]
    n = np.sum(np.cross(p - p.mean(0), np.roll(p, -1, axis=0) - p.mean(0)), axis=0)
    return list(face) if np.dot(n, outward) > 0 else list(face)[::-1]


class TeaBottle(Bottle):
    """Square tea bottle (00006 family): chamfered-square body with fluted chamfer facets and a circular recess in
    the base, short shoulder into a long round neck, flat label panels on the front (+X) and back (-X) faces, a neck
    band over the front half of the neck, and a cap with a clear ribbed skirt and an inset top disc."""

    def __init__(self, g):
        g = dict(g, NECK_H=g["H"] - g["FINISH_H"] - g["RING_T"] - g["Z_NECK"])
        super().__init__(g)
        self.sec, self.flute = tea_section(g)
        z0, z1, hw, _ = g["PANEL"]
        assert hw <= g["R"] - g["ROUND"] - g["CHAMFER"] - 0.0003, "label panel wider than the flat face"
        assert g["FIL"] < z0 and z1 < g["Z_SH"] - 0.002, "label panel off the straight body"
        n0, n1, _ = g["NECK_LABEL"]
        assert self.z_neck < n0 and n1 < self.z_ring0, "neck label off the neck"

    def visual_body(self, z):
        return self.body(z)

    def body_samples(self):
        g = self.g
        z0, z1 = g["FIL"] + 0.002, g["Z_SH"] - 0.002
        z = np.r_[np.linspace(z0, z0 + 0.006, 5), np.linspace(z1 - 0.006, z1, 5), g["FIL"] + 0.001, g["Z_SH"]]
        return np.unique(np.round(z, 7))

    def body_ring(self, z, inset=0.0):
        """Visual body ring at z, flutes faded out towards the bottom edge and the shoulder."""
        g = self.g
        fade = np.clip((z - g["FIL"] - 0.002) / 0.006, 0, 1) * np.clip((g["Z_SH"] - 0.002 - z) / 0.006, 0, 1)
        return self.ring(self.body(z) - inset, 1.0) - fade * self.flute

    def loft(self, profile):
        return pc.loft(
            [
                (r, z) if isinstance(r, np.ndarray) else (0.0, z) if r == 0 else (self.ring(r, b), z)
                for r, z, b in profile
            ]
        )

    def finish(self):
        """Outer profile [(r, z)] from the neck top to the bore under the lip: support ring, tamper bead, threads."""
        g = self.g
        H = g["H"]
        z0, z1, rn, rr, rc, rt = self.z_ring0, self.z_ring1, g["R_NECK"], g["R_RING"], g["R_CORE"], g["R_THREAD"]
        finish = [(rn, z0)] * 2 + [(rr, z0)] * 2 + [(rr, z1)] * 2 + [(rc, z1)] * 2
        finish += [(rc, z1 + 0.002), (rt, z1 + 0.003), (rt, z1 + 0.004), (rc, z1 + 0.005)]  # tamper-band bead
        for d in g["THREADS"]:
            zt = H - d
            assert zt - 0.001 > z1 + 0.005, "thread overlaps the bead"
            finish += [(rc, zt - 0.001), (rt, zt - 0.0003), (rt, zt + 0.0003), (rc, zt + 0.001)]
        rb = g["R_BORE"]
        finish += [(rc, H - 0.001), (rc - 0.0004, H - 0.0001), ((rc + rb) / 2, H), (rb + 0.0004, H - 0.0001)]
        return finish + [(rb, H - 0.001)]

    def profiles(self):
        g = self.g
        R, FIL, FOOT, DOME = g["R"], g["FIL"], g["FOOT"], g["DOME"]
        dome = [(s * FOOT, DOME * (1 - s * s) ** 2) for s in (0.3, 0.55, 0.75, 0.9)]
        t = np.linspace(0, 1, 25)[1:]
        sh = [(*self.shoulder(v), g["Z_SH"] + v * (self.z_neck - g["Z_SH"])) for v in t]
        zs = self.body_samples()
        outer = [(0.0, DOME, 0.0)] + [(r, z, 0.0) for r, z in dome] + [(FOOT, 0.0, 0.0)]  # round recess
        outer += [(r, z, 1.0) for r, z in pc.fillet(R, FIL)]
        outer += [(self.body_ring(z), z, 1.0) for z in zs]
        outer += [(r, z, b) for r, b, z in sh]
        outer += [(r, z, 0.0) for r, z in self.finish()]
        rn, rb = g["R_NECK"], g["R_BORE"]
        inner = [(rb, self.z_ring0, 0.0)] * 2 + [(rn - T_WALL, self.z_ring0 - 0.002, 0.0)]  # thin neck wall
        inner += [(r - T_WALL, z, b) for r, b, z in sh[::-1]]
        inner += [(self.body_ring(z, T_WALL), z, 1.0) for z in zs[::-1]]
        inner += [(r, z, 1.0) for r, z in mp.arc(R - FIL, FIL, FIL - T_WALL, 0, -np.pi / 2, 6)]
        inner += [(FOOT, T_WALL, 0.0)] + [(r, z + T_WALL, 0.0) for r, z in dome[::-1]] + [(0.0, DOME + T_WALL, 0.0)]
        return outer, inner

    def labels(self):
        """Front / back panels and the neck band as thin closed shells, with per-face-vertex UVs: u 0-0.5 the
        printed side (image left to right as seen from outside, v up), u 0.5-1 the reverse (bottle_label.two_sided)."""
        g = self.g
        parts, uvs = {}, {}
        z0, z1, hw, k = g["PANEL"]
        poly = [(-hw + k, z0), (hw - k, z0), (hw, z0 + k), (hw, z1 - k), (hw - k, z1), (-hw + k, z1), (-hw, z1 - k),
                (-hw, z0 + k)]  # fmt: skip
        for name, sx in (("label_front", 1.0), ("label_back", -1.0)):  # seen from outside, image right is sx * +Y
            pts = np.array([(sx * x, sx * s, z) for x in (g["R"] + 0.00025, g["R"] + 0.00005) for s, z in poly])
            centre = pts.mean(0)
            faces = [orient(pts, range(8), (sx, 0, 0)), orient(pts, range(8, 16), (-sx, 0, 0))]
            for i in range(8):
                f = [i, (i + 1) % 8, (i + 1) % 8 + 8, i + 8]
                faces.append(orient(pts, f, pts[f].mean(0) - centre))
            uv = []
            for n, f in enumerate(faces):
                for i in f:
                    s, z = sx * pts[i, 1], pts[i, 2]
                    uv.append(((s + hw) / (2 * hw) * 0.5 + 0.5 * (n == 1), (z - z0) / (z1 - z0)))
            parts[name], uvs[name] = (pts, faces), np.array(uv)
        n0, n1, half = g["NECK_LABEL"]
        a = np.radians(np.linspace(-half, half, 97))
        m = len(a)
        rings = [(g["R_NECK"] + 0.00035, n0), (g["R_NECK"] + 0.00035, n1), (g["R_NECK"] + 0.0001, n0),
                 (g["R_NECK"] + 0.0001, n1)]  # fmt: skip
        pts = np.array([(r * np.cos(v), r * np.sin(v), z) for r, z in rings for v in a])
        ob, ot, ib, it = (np.arange(m) + q * m for q in range(4))
        faces, inside = [], []
        for j in range(m - 1):
            radial = np.array([np.cos((a[j] + a[j + 1]) / 2), np.sin((a[j] + a[j + 1]) / 2), 0.0])
            for f, out, rev in (
                ([ob[j], ob[j + 1], ot[j + 1], ot[j]], radial, False),
                ([ib[j], ib[j + 1], it[j + 1], it[j]], -radial, True),
                ([ot[j], ot[j + 1], it[j + 1], it[j]], (0, 0, 1), False),
                ([ob[j], ob[j + 1], ib[j + 1], ib[j]], (0, 0, -1), False),
            ):
                faces.append(orient(pts, f, out))
                inside.append(rev)
        for j, sign in ((0, -1), (m - 1, 1)):
            tangent = sign * np.array([-np.sin(a[j]), np.cos(a[j]), 0.0])
            faces.append(orient(pts, [ob[j], ot[j], it[j], ib[j]], tangent))
            inside.append(False)
        uv = [
            ((np.arctan2(pts[i, 1], pts[i, 0]) - a[0]) / (a[-1] - a[0]) * 0.5 + 0.5 * rev, (pts[i, 2] - n0) / (n1 - n0))
            for f, rev in zip(faces, inside)
            for i in f
        ]
        parts["label_neck"], uvs["label_neck"] = (pts, faces), np.array(uv)
        return parts, uvs

    def label_px(self):
        """Texture size (w, h) of the printed side of each label."""
        g = self.g
        z0, z1, hw, _ = g["PANEL"]
        n0, n1, half = g["NECK_LABEL"]
        panel = (round(2 * hw * TEA_PX), round((z1 - z0) * TEA_PX))
        neck = (round(np.radians(2 * half) * (g["R_NECK"] + 0.00035) * TEA_PX), round((n1 - n0) * TEA_PX))
        return {"label_front": panel, "label_back": panel, "label_neck": neck}

    def cap(self):
        """Screw cap as Bottle.cap, its top recessed for an inset disc (the white liner seen through the clear cap).
        Returns the cap shell, the disc and the cap collider."""
        g = self.g
        z0, z1, zb = self.z_ring1 + 0.0006, g["H"] + g["CAP_TOP"], self.z_ring1 + 0.0006 + g["BAND_H"]
        rc, ri = g["R_THREAD"] + g["CAP_T"], g["R_THREAD"] + 0.0002
        rw, zw = rc - 0.0013, z1 - 0.0006
        knurl = g["RIB_D"] * (1 - np.abs(np.cos(np.pi * g["CAP_RIBS"] * np.arange(N_THETA) / N_THETA)))
        circle = np.c_[
            np.cos(np.arange(N_THETA) * 2 * np.pi / N_THETA), np.sin(np.arange(N_THETA) * 2 * np.pi / N_THETA)
        ]

        def rd(r):
            return r * circle

        ribbed = rd(rc) * ((rc - knurl) / rc)[:, None]
        profile = [(0.0, g["H"])] + [(rd(ri), g["H"])] * 2 + [(rd(ri), z0)] * 2 + [(rd(rc - 0.0003), z0)] * 2
        profile += [(rd(rc - 0.0003), zb - 0.0004), (rd(rc - 0.0008), zb - 0.0002), (rd(rc - 0.0008), zb + 0.0002)]
        profile += [(rd(rc - 0.0003), zb + 0.0004), (ribbed, zb + 0.0008), (ribbed, z1 - 0.0012)]
        profile += [(rd(rc - 0.0005), z1 - 0.0003)] + [(rd(rc - 0.0012), z1)] * 2 + [(rd(rw), z1)] * 2
        profile += [(rd(rw), zw)] * 2 + [(0.0, zw)]
        disc = [(0.0, zw)] + [(rd(rw - 0.00005), zw)] * 2 + [(rd(rw - 0.00005), z1 - 0.00005)] * 2
        disc += [(0.0, z1 - 0.00005)]
        step = N_THETA // BOTTOM_SIDES
        hull = np.array([(*p, z) for z in (z0, z1) for p in rd(rc)[::step]])
        return pc.loft(profile), pc.loft(disc), hull

    def brim_ml(self):
        """Visual inner volume from the bottom to the mouth (push-up ignored)."""
        g = self.g
        z = np.linspace(T_WALL, g["H"], 1200)
        a = []
        for v in z:
            if v > self.z_ring0:
                ring = self.ring(g["R_BORE"], 0.0)
            elif v > g["Z_SH"]:
                r, b = self.envelope(min(v, self.z_neck))
                ring = self.ring(r - T_WALL, b)
            else:
                ring = self.ring(self.visual_body(max(v, g["FIL"])) - T_WALL, 1.0)
            a.append(pc.section_area(ring))
        a = np.array(a)
        return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def check_hulls(g, b, hulls):
    """Hull limits and the cavity check of build(); the cap collider closes the mouth on purpose and is not passed."""
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["FIL"] + 0.001, g["H"] - 0.001, 60)
            for f in np.linspace(0, 0.94, 6)
            for p in b.cavity(z)[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "slab points within PhysX plane tolerance of the bottom"


def build_tea(g):
    b = TeaBottle(g)
    outer, inner = b.profiles()
    cap, top, cap_hull = b.cap()
    labels, uvs = b.labels()
    parts = {"model": b.loft(outer + inner), **labels, "cap": cap, "cap_top": top}
    for name, part in parts.items():
        assert mp.signed_volume(*part) > 0, f"{name} has inward-facing winding"
    hulls = b.hulls()
    check_hulls(g, b, hulls)
    return dict(
        parts=parts, hulls=hulls, cap_hull=cap_hull, uvs=uvs, label_px=b.label_px(), ml=b.brim_ml(),
        contact_r=g["R"] - g["FIL"], bottle=b,
    )  # fmt: skip


def photo_labels(px):
    """00006 labels rectified from the registration photos, lighting evened out: {name: PIL image}."""
    front = Image.open(REFERENCE / PHOTO_LABELS["front"])
    back = Image.open(REFERENCE / PHOTO_LABELS["back"])
    out = {}
    for name, photo, quad, paper in (
        ("label_front", front, PHOTO_LABELS["front_quad"], (228, 228, 222)),  # light grey paper
        ("label_back", back, PHOTO_LABELS["back_quad"], (242, 242, 238)),  # white paper
    ):
        rgb = bottle_label.photo_panel(photo, quad, px[name])
        lum, chroma = rgb.mean(-1), np.ptp(rgb, axis=-1)
        out[name] = bottle_label.even_light(rgb, (chroma < 25) & (lum > np.percentile(lum, 40)), paper)
    n = PHOTO_LABELS["neck"]
    half = np.radians(DONGFANG["NECK_LABEL"][2])
    rgb = bottle_label.photo_band(
        front, n["cx"], n["rx"], n["top"], n["bottom"], (-half, half), np.radians(n["seen"]), px["label_neck"]
    )
    green = (rgb[..., 1] - rgb[..., 0] > 25) & (rgb.mean(-1) < np.percentile(rgb.mean(-1), 70))
    target = tuple(np.clip(np.median(rgb[green], axis=0) * 1.3, 0, 255))
    out["label_neck"] = bottle_label.even_light(rgb, green, target)
    return out


def tea_textures(g, c, idx):
    """{path inside the usdz: PNG file} of the three labels, printed side and reverse side by side."""
    px = c["label_px"]
    if g["ART"] == "photo":
        art = photo_labels(px)
    else:
        art = {
            "label_front": bottle_label.front_panel(g["ART"], px["label_front"], idx),
            "label_back": bottle_label.back_panel(g["ART"], px["label_back"], idx),
            "label_neck": bottle_label.neck_band(g["ART"], px["label_neck"], idx),
        }
    tmp = Path(tempfile.mkdtemp())
    out = {}
    for name, img in art.items():
        path = tmp / f"{name}.png"
        bottle_label.two_sided(img, (226, 228, 224), 0.15).save(path)
        out[f"textures/{name}.png"] = path
    return out


def tea_looks(g):
    """Materials of the TeaBottle parts that differ from the shared ones."""

    def textured(name):
        def material(st):
            mat = mp.add_material(
                st,
                f"/root/_materials/{name}",
                "OmniPBR",
                {"reflection_roughness_constant": 0.35, "metallic_constant": 0.0},
                (0.85, 0.85, 0.85),
                1.0,
                0.35,
            )
            sh = UsdShade.Shader.Get(st, f"/root/_materials/{name}/Shader")
            sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f"textures/{name}.png"))
            return mat

        return material

    def clear_cap(st):
        return mp.add_material(
            st,
            "/root/_materials/CapPlastic",
            "OmniGlass",
            {"glass_color": g["CAP_RGB"], "glass_ior": 1.49, "frosting_roughness": 0.05, "thin_walled": False},
            g["CAP_RGB"],
            0.3,
            0.1,
        )

    looks = {name: textured(name) for name in ("label_front", "label_back", "label_neck")}
    looks["cap"] = clear_cap if g["CAP_CLEAR"] else paint("Cap", g["CAP_RGB"])
    looks["cap_top"] = paint("CapTop", g["CAP_TOP_RGB"])
    return looks


def build(g):
    b = Bottle(g)
    outer, inner = b.profiles()
    label, label_uv, label_px = b.label()
    cap, cap_hull = b.cap()
    parts = {"model": b.loft(outer + inner), "label": label, "cap": cap}
    for name, part in parts.items():
        assert mp.signed_volume(*part) > 0, f"{name} has inward-facing winding"
    hulls = b.hulls()  # the bottle's; the cap collider closes the mouth on purpose, so the cavity check skips it
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["FIL"] + 0.001, g["H"] - 0.001, 60)
            for f in np.linspace(0, 0.94, 6)
            for p in b.cavity(z)[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "slab points within PhysX plane tolerance of the bottom"
    return dict(
        parts=parts, hulls=hulls, cap_hull=cap_hull, label_uv=label_uv, label_px=label_px,
        ml=b.brim_ml(), contact_r=g["R"] - g["FIL"], bottle=b,
    )  # fmt: skip


def glass(g):
    def material(st):
        if g["FROST"]:  # milky share, as in pc.clear_pp: OmniGlass frosting only blurs, it never whitens the wall
            return mp.add_material(
                st,
                "/root/_materials/Plastic",
                "OmniSurface",
                {
                    "diffuse_reflection_weight": 1.0,
                    "diffuse_reflection_color": g["TINT"],
                    "enable_specular_transmission": True,
                    "specular_transmission_weight": 1.0 - g["FROST"],
                    "specular_reflection_weight": 0.1,
                    "specular_reflection_roughness": 0.3,
                    "specular_reflection_ior": 1.57,
                    "thin_walled": False,
                },
                g["TINT"],
                0.5,
                0.3,
            )
        return mp.add_material(
            st,
            "/root/_materials/Plastic",
            "OmniGlass",
            {"glass_color": g["TINT"], "glass_ior": 1.57, "frosting_roughness": 0.0, "thin_walled": False},
            g["TINT"],
            0.25,
            0.05,
        )

    return material


def paint(name, rgb):
    def material(st):
        return mp.add_material(
            st,
            f"/root/_materials/{name}",
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": 0.45, "metallic_constant": 0.0},
            rgb,
            1.0,
            0.45,
        )

    return material


def label_look(g):
    def material(st):
        rgb = g["LABEL_RGB"][0]  # UsdPreviewSurface fallback colour
        mat = mp.add_material(
            st,
            "/root/_materials/Label",
            "OmniPBR",
            {"reflection_roughness_constant": 0.35, "metallic_constant": 0.0},
            rgb,
            1.0,
            0.35,
        )
        sh = UsdShade.Shader.Get(st, "/root/_materials/Label/Shader")
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(LABEL_TEX))
        return mat

    return material


def add_hulls(st, parent, hulls, physics_material=None):
    """Invisible convex-hull colliders <parent>/hull_NN (pc.add_collision under any parent)."""
    mp.add_xform(st, parent)
    for i, h in enumerate(hulls):
        hull = ConvexHull(h)
        tri = hull.simplices.copy()
        n = np.cross(h[tri[:, 1]] - h[tri[:, 0]], h[tri[:, 2]] - h[tri[:, 0]])
        flip = np.einsum("ij,ij->i", n, hull.equations[:, :3]) < 0
        tri[flip] = tri[flip][:, ::-1]
        used, tri = np.unique(tri, return_inverse=True)
        m = mp.add_mesh(st, f"{parent}/hull_{i:02d}", h[used], tri.reshape(-1, 3).tolist())
        prim = m.GetPrim()
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vis = UsdGeom.VisibilityAPI.Apply(prim)
        vis.CreateRenderVisibilityAttr(UsdGeom.Tokens.invisible)
        vis.CreateGuideVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        if physics_material is not None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                physics_material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )


def add_meshes(st, c, g, links):
    """Visual meshes: bottle shell and label on links['body'], cap on links['cap'] (model first: loaders read the
    first Mesh)."""
    looks = {"model": glass(g), "label": label_look(g), "cap": paint("Cap", g["CAP_RGB"])}
    if g.get("TEA"):
        looks.update(tea_looks(g))
    for name, (pts, faces) in c["parts"].items():
        link = links["cap" if name.startswith("cap") else "body"]
        if not st.GetPrimAtPath(f"{link}/visual"):
            mp.add_xform(st, f"{link}/visual")
        mesh = mp.add_mesh(st, f"{link}/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        if name.startswith("label"):
            uv = c["uvs"][name] if "uvs" in c else c["label_uv"]
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            ).Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(looks[name](st))


def add_cap_joints(st, c, g, mass):
    """Floating articulation body -> cap_twist (revolute about the bottle axis, 0 = closed, + = unscrewing, i.e.
    counter-clockwise seen from above) -> nut -> cap_lift (prismatic along the axis, mimics the twist: one PITCH per
    turn) -> cap. Both stop after OPEN_TURNS; the cap stays on the bottle (a loose cap cannot leave an articulation)."""
    root = st.GetPrimAtPath("/root/body")  # on the root link, as ArticulationObject.fix_root_link expects
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(16)
    pm = UsdShade.Material.Define(st, "/root/_materials/PhysicsPlastic")
    api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())  # ArticulationObject does not apply metadata friction
    api.CreateStaticFrictionAttr(STATIC_FRICTION)
    api.CreateDynamicFrictionAttr(DYNAMIC_FRICTION)
    api.CreateRestitutionAttr(0.0)
    zc = float(c["cap_hull"][:, 2].mean())
    for link, m, inertia in (
        ("/root/body", mass - g["CAP_MASS"] - NUT_MASS, None),  # inertia from the colliders
        ("/root/nut", NUT_MASS, (NUT_INERTIA,) * 3),  # massless-ish carrier between the two joints, no collider
        ("/root/cap", g["CAP_MASS"], None),
    ):
        prim = st.GetPrimAtPath(link)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(m)
        if inertia:
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, zc))
    add_hulls(st, "/root/body/collision", c["hulls"], pm)
    add_hulls(st, "/root/cap/collision", [c["cap_hull"]], pm)
    UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath("/root/cap")).CreateFilteredPairsRel().AddTarget("/root/body")
    mp.add_xform(st, "/root/joints")
    twist = UsdPhysics.RevoluteJoint.Define(st, "/root/joints/cap_twist")
    lift = UsdPhysics.PrismaticJoint.Define(st, "/root/joints/cap_lift")
    for j, b0, b1, upper in ((twist, "/root/body", "/root/nut", 360.0 * g["OPEN_TURNS"]),
                             (lift, "/root/nut", "/root/cap", g["PITCH"] * g["OPEN_TURNS"])):  # fmt: skip
        j.CreateBody0Rel().SetTargets([b0])
        j.CreateBody1Rel().SetTargets([b1])
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(0.0)
        j.CreateUpperLimitAttr(upper)
        for k in (0, 1):  # all link frames coincide with /root; the bottle axis is its Z axis
            getattr(j, f"CreateLocalPos{k}Attr")(Gf.Vec3f(0, 0, 0))
            getattr(j, f"CreateLocalRot{k}Attr")(Gf.Quatf(1, 0, 0, 0))
    drive = UsdPhysics.DriveAPI.Apply(twist.GetPrim(), "angular")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(CAP_DAMPING)
    PhysxSchema.PhysxJointAPI.Apply(twist.GetPrim()).CreateArmatureAttr(CAP_ARMATURE)
    mimic = PhysxSchema.PhysxMimicJointAPI.Apply(lift.GetPrim(), "rotZ")
    mimic.CreateReferenceJointRel().SetTargets([twist.GetPath()])
    mimic.CreateGearingAttr(-g["PITCH"] / 360.0)  # lift + gearing * twist = 0, metres per degree
    mimic.CreateOffsetAttr(0.0)


def write_usdz(out, c, g, label_png, articulated=False, mass=None):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    if articulated:
        for link in ("/root/body", "/root/nut", "/root/cap"):
            mp.add_xform(st, link)
        add_meshes(st, c, g, {"body": "/root/body", "cap": "/root/cap"})
        add_cap_joints(st, c, g, mass)
    else:
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        add_meshes(st, c, g, {"body": "/root", "cap": "/root"})
        add_hulls(st, "/root/collision", c["hulls"] + [c["cap_hull"]])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    for name, path in label_png.items() if isinstance(label_png, dict) else [(LABEL_TEX, label_png)]:
        zw.AddFile(str(path), name)
    zw.Save()


def description(uid, g, ml):
    w = g["WORDS"]
    caption = {
        "color": w["color"],
        "material": w["material"],
        "size": [f"holds about {ml:.0f} ml to the brim", f"about {g['H'] * 100:.0f} cm tall", "fits in one hand"],
        "name": w["name"],
        "shape": w["shape"],
        "function": w["function"],
    }
    desc = [
        w["look"],
        f"{w['name'][1]} closed with a screw cap, about {g['H'] * 100:.0f} cm tall",
        f"hollow {w['material'][1]} {w['name'][0]} that holds about {ml:.0f} ml",
        f"{w['name'][0]} with a {w['shape'][0]} and a {w['shape'][1]}",
        f"light {w['name'][0]} grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def measurements(c, parts, every, g):
    """measurements_mm of a registered bottle, named as its spec.json dimensions_mm."""
    model = parts["model"][0]
    mouth = model[model[:, 2] > c["bottle"].z_ring1 + c["shift"][2] + 1e-6]
    if g["MEASURE"] == "round":
        foot = model[model[:, 2] < model[:, 2].min() + 0.010]
        return {
            "开口直径": round(float(np.ptp(mouth[:, 0])) * 1000, 1),  # outer, thread crests
            "底部直径": round(float(np.ptp(foot[:, 0])) * 1000, 1),  # outer body just above the bottom fillet
        }
    lo = model[:, 2].min()
    body = model[(model[:, 2] > lo + g["FIL"]) & (model[:, 2] < lo + g["Z_SH"])]
    return {
        "瓶口外径": round(float(np.ptp(mouth[:, 0])) * 1000, 1),  # outer, thread crests
        "瓶身边长": round(float(np.ptp(body[:, 0])) * 1000, 1),  # across the flats
        "高度": round(float(np.ptp(every[:, 2])) * 1000, 1),  # bottom to the top of the cap
    }


def save(type_dir, idx, c, g, mass, articulated=False, base_idx=0):
    """Clutter/<category>/<idx>: one rigid body, cap fixed on. With articulated: the sibling
    Articulation/<category>/<idx> with the cap on cap_twist / cap_lift (add_cap_joints). base_idx: the registered
    bottle this one is (idx == base_idx) or varies."""
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]], cap_hull=c["cap_hull"] + shift)
    kind = "Articulation" if articulated else "Clutter"
    d = Path(type_dir).parent / kind / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/{kind.lower()}/{CATEGORY}/{idx}"))
    if g.get("TEA"):
        label_png = tea_textures(g, c, idx)
    else:
        label_png = Path(tempfile.mkdtemp()) / "label.png"
        art = bottle_label.draw_label(g["ART"], *g["LABEL_RGB"], g["LABEL"][2], c["label_px"], idx)
        ImageChops.offset(art, int(LABEL_FRONT_U * art.width), 0).save(label_png)
    write_usdz(d / "object.usdz", c, g, label_png, articulated, mass)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), mass, shift)
    meta["physics"].pop("friction", None)  # pc.metadata no longer writes it on every branch
    if g["FRICTION"]:
        meta["physics"].update(static_friction=g["FRICTION"][0], dynamic_friction=g["FRICTION"][1])
    meta["visual"] = {"color_parts": ["cap_top" if g.get("TEA") else "cap"]}  # rt_color: the bottle stays clear
    if articulated:  # joint positions as ArticulationObject reports them: radians, metres
        meta["articulation"] = {
            "cap_twist": {"closed": 0.0, "open": round(2 * np.pi * g["OPEN_TURNS"], 4), "unit": "rad"},
            "cap_lift": {"closed": 0.0, "open": round(g["PITCH"] * g["OPEN_TURNS"], 4), "unit": "m"},
        }
    if idx == base_idx:
        meta["measurements_mm"] = measurements(dict(c, shift=shift), parts, every + shift, g)
    else:
        meta["variant_of"] = f"{kind}/{CATEGORY}/{base_idx:05d}"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g, c["ml"]), indent=4))
    return d, meta


def main(type_dir):
    for first, registered, variants in FAMILIES:
        base = None
        for k, over in enumerate([{}] + variants):
            g = {**registered, **over}
            c = (build_tea if g.get("TEA") else build)(g)
            if k == 0:
                base = c["ml"]
                lo, hi = g["BRIM"]
                assert lo < base < hi, f"brim volume {base:.0f} ml outside {lo}-{hi} ml"
                mass = g["MASS"]
            else:
                mass = round(registered["MASS"] * c["ml"] / base, 4)  # estimated: the registered bottle by brim volume
            for articulated in (False, True):
                d, meta = save(type_dir, first + k, c, g, mass, articulated, first)
                print(
                    f"{d}: {c['ml']:.0f} ml brim, mass {mass} kg, size {meta['physics']['size']}, "
                    f"{len(c['hulls']) + 1} collision hulls, label {c['label_px']} px, "
                    f"measurements_mm {meta.get('measurements_mm', '-')}"
                )


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else TYPE_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
