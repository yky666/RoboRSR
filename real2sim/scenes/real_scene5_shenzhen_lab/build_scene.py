"""Build the ShenZhen lab scene (real_scene5) as modular USD layers: the desk and cabinet from the tape measurements in
registration.json and the photo fit (fit_photo.json, reference/IMG_4913-4918.jpeg), the room around them from the
room fit (fit_room.json, reference/IMG_4948-4955.jpg).

Scene frame: Z up, meters. Origin on the floor under the tabletop centre; +X along the long side; +Y towards the wall
behind the desk. The robot stands on the -Y side facing +Y (same convention as real_scene1-3).

The tabletop is fixed to the tape (1.20 x 0.60 m, 0.75 m high, 2 cm thick); the leg columns, feet and the cabinet's
position come from fit_photo, the cabinet's size from the registration (75 x 120 x 45 cm). The room is an open-plan
office floor: structural columns in the two back corners, a gypsum soffit along the back wall (flush with the columns)
and along the elevator side, a perforated metal ceiling with linear LED strips in between, a bench desk row with blue
screens along the east wall, a glass railing and a turnstile in front of the two elevators. Only the furniture asked
for is kept (desks, chairs, the shelf, the railing and turnstile); boxes, bags and loose items are not modelled.
Furniture sits under /Background/Furniture so the Room entry, scene_parts.py and render_scenes.py treat it as room.
"""

import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from shapely.geometry import box as sbox
import trimesh

HERE = Path(__file__).resolve().parent
PHOTOS = HERE / "reference"
OUT = HERE / "usd"
TEX = OUT / "textures"
ROOM_NAME = "real_scene5"
FIT = json.loads((HERE / "fit_photo.json").read_text())
FX, F = FIT["fixed"], FIT["dims"]
FR = json.loads((HERE / "fit_room.json").read_text())
RP, RM = FR["params"], FR["measure"]


def r3(v):
    return round(float(v), 3)


TABLE = dict(length=FX["length"], width=FX["width"], top_z=FX["top_z"], thick=FX["thick"], corner_r=0.05)
# Lift-desk T legs: two-stage column on a foot bar along Y, side rails under the top, a cross beam between the columns.
LEGS = dict(
    x=r3(F["col_x"]),
    y=r3(F["col_y"]),
    foot_front=r3(F["foot_front"]),
    foot_back=r3(F["foot_back"]),
    upper=(0.055, 0.045),  # column sections (x, y), from the photos' pixel widths
    lower=(0.065, 0.055),
    split_z=0.45,  # where the inner column leaves the outer one (IMG_4918, IMG_4913)
    foot=(0.06, 0.035),  # foot bar width, height
)
CABINET = dict(x0=r3(F["cab_x0"]), w=FX["cab_w"], h=FX["cab_h"], d=FX["cab_d"], y1=FX["wall_y"])
SKIRTING = 0.08  # black marble skirting, IMG_4916; the elevator wall's white panels have none (IMG_4952, IMG_4951)
# Room outline from fit_room.json (see PLAN.md for the photo each value comes from). Columns at the back corners;
# the elevator wall stops at x_sw, the floor goes on south to y_sw west of it.
ROOM = dict(
    x_w=r3(RP["x_W"]),
    x_e=r3(RP["x_E"]),
    y_n=FX["wall_y"],
    y_s=r3(RP["y_S"]),
    x_nw=r3(RP["x_nw"]),
    x_ne=r3(RP["x_jog"]),
    y_col=r3(FX["wall_y"] - RP["d_col"]),
    x_sw=r3(RM["s_wall_end"][0]),
    y_sw=r3(RM["sw_corner"][1]),
    z_soffit=r3(RP["z_s"]),  # gypsum soffit underside (back wall and elevator side)
    z_ceiling=r3(RP["z_p"]),  # perforated ceiling
    y_soffit_s=r3(RP["y_soffit_s"]),  # elevator-side soffit edge (IMG_4950)
)
# Linear LED strips along Y in the perforated ceiling (IMG_4955, IMG_4953, IMG_4954) and the dark slot diffuser
# (IMG_4955); they stop ~0.35 m short of the soffits (IMG_4955).
STRIP_X = [0.09, -3.31, -5.90]
SLOT_X = -1.32
STRIP_Y = (ROOM["y_col"] - 0.35, ROOM["y_soffit_s"] + 0.35)
# Downlights: in the back soffit as measured (IMG_4955, IMG_4954), in the elevator-side soffit by eye (IMG_4952)
DOWNLIGHTS_N = [(x, -0.08) for x in (-6.37, -4.57, -3.01, -0.64, 1.36)]
DOWNLIGHTS_S = [(x, -8.2) for x in (-7.0, -5.0, -2.8, 0.6, 2.3)]
# Elevator surrounds on the elevator wall, x ranges (KT3 west edge IMG_4952, KT4 east edge IMG_4950, symmetric about
# x = 0 with a 1.33 m wall between them, IMG_4951); door leaves 2.2 m high.
ELEVATORS = [(r3(RM["kt3_east"][0]), -0.66), (0.66, r3(RM["kt4_west"][0]))]
DESKS = dict(  # bench desk row on the east wall, IMG_4949 / IMG_4950
    x_front=r3(RP["x_E"] - RP["desk_dx"]),
    x_back=r3(RP["x_E"] - 0.02),
    y_north=r3(RP["desk_yn"]),
    module=RP["module"],
    n=RP["n_modules"],
    top_z=0.75,
    screen=(1.1, 0.35),  # blue fabric screen per module (width, height above the top), by eye
    pedestal=(0.40, 0.50, 0.60),  # mobile pedestal under the south side of each module (y, x, z)
)
LIFT_DESK = dict(x0=r3(ROOM["x_nw"] + 0.03), length=1.4, width=0.7, top_z=0.73)  # IMG_4954, against the back wall
SHELF = dict(  # blue steel shelf on the west wall, IMG_4953
    y0=r3(RM["shelf_front"][0][1]),
    y1=r3(RM["shelf_top"][1][1]),
    depth=0.5,
    height=r3(np.mean([p[2] for p in RM["shelf_top"]])),
    levels=[r3(p[2]) for p in reversed(RM["shelf_beam"])],
)
# Chairs (floor point under the seat, yaw of the seat front, style) from fit_room "chairs"; yaw by eye
CHAIRS = [
    ((2.25, -1.0), 270, "task"),  # measured (2.39, -0.84); 0.2 m further out so the turned back clears desk and column
    (RM["chairs"]["desk2"], -20, "ergo"),
    (RM["chairs"]["desk3"], 15, "ergo"),
    (RM["chairs"]["desk5"], 5, "ergo"),
    (RM["chairs"]["lift_desk"], 60, "task"),
]
# Glass railing (1.1 m posts) and turnstile in front of the elevators: east part along y = -7.85 from the east wall
# (IMG_4950 posts), the turnstile pillars x -0.78 / -1.62 (IMG_4952, IMG_4951), west part along y = -7.74 and its
# return to the elevator wall at x = -2.62 (IMG_4952 posts).
RAILING = dict(
    height=1.1,
    runs=[
        [(ROOM["x_e"] - 0.01, -7.85), (-0.68, -7.85)],
        [(-1.72, -7.74), (-2.62, -7.74), (-2.62, ROOM["y_s"] + 0.01)],
    ],
    turnstile=[(-0.78, -7.75), (-1.62, -7.75)],
)
# clean carpet patch in front of the cabinet in IMG_4913 (scene x0, x1, y0, y1); the photo pixels already carry the
# room light; gain set so the RTX floor matches IMG_4916 (0.67 as in real_scene3 rendered it 78 vs 98 sRGB, 0.9: 105)
CARPET = dict(patch=(0.85, 1.15, -1.5, -1.2), photo="IMG_4913.jpeg", gain=0.85)
PERFORATED_TILE = 0.6  # m, ceiling panel module

# Registered: ceiling strip lights. Modelled as one RectLight per LED strip plus a cone-shaped DiskLight per back-soffit
# downlight (the scallops on the back wall in IMG_4955); brightness not recorded.
LIGHTS = dict(
    strip_width=0.06,
    strip_intensity=100000.0,
    downlight_radius=0.05,
    downlight_intensity=1600000.0,
    downlight_cone_deg=55.0,
    # Isaac cameras have no white balance; the colours are in the materials (sRGB from the photos)
    color_temperature=6000.0,  # 6500 K rendered the cream wall neutral (R/B 1.01 vs 1.05)
    dome=150.0,
)


def lin(srgb):
    c = np.asarray(srgb, float) / 255
    return tuple(float(v) for v in np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4))


# name: (diffuse linear RGB, roughness, metallic, opacity, texture). sRGB sampled in the photos (IMG_4916 for the desk,
# wall, cabinet; IMG_4949-4955 for the room), scaled up like real_scene3's tabletop (photo pixels carry the room light).
MATERIALS = {
    "laminate": (lin((228, 228, 223)), 0.35, 0.0, 1.0, None),
    "frame_white": (lin((215, 216, 214)), 0.4, 0.2, 1.0, None),
    "cap_grey": (lin((150, 150, 148)), 0.6, 0.0, 1.0, None),
    "steel": (lin((120, 125, 130)), 0.35, 0.8, 1.0, None),
    "cabinet": (lin((190, 189, 184)), 0.45, 0.3, 1.0, None),
    "seam": (lin((40, 40, 40)), 0.6, 0.0, 1.0, None),
    "skirting": (lin((14, 15, 14)), 0.25, 0.0, 1.0, None),
    "wall": (lin((205, 201, 194)), 0.9, 0.0, 1.0, None),
    "wall_panel": (lin((212, 209, 201)), 0.35, 0.0, 1.0, None),  # elevator wall cladding, IMG_4952 (204, 200, 190)
    "ceiling": (lin((230, 230, 230)), 0.9, 0.0, 1.0, None),
    "perforated": ((1.0, 1.0, 1.0), 0.5, 0.3, 1.0, "perforated.png"),
    "led": ((1.0, 1.0, 1.0), 0.5, 0.0, 1.0, None),
    "slot": (lin((20, 20, 20)), 0.8, 0.0, 1.0, None),
    "carpet": ((1.0, 1.0, 1.0), 0.95, 0.0, 1.0, "carpet.png"),
    "wood_dark": (lin((62, 40, 24)), 0.4, 0.0, 1.0, None),  # elevator surround, IMG_4951 (54, 33, 17)
    "bronze": (lin((88, 60, 38)), 0.35, 0.6, 1.0, None),  # elevator doors, IMG_4951 (76, 50, 30)
    "screen_blue": (lin((90, 128, 185)), 0.9, 0.0, 1.0, None),  # desk screens, IMG_4949 (95, 132, 189)
    "lift_top": (lin((172, 158, 142)), 0.5, 0.0, 1.0, None),  # IMG_4954 (174, 158, 141)
    "frame_grey": (lin((140, 141, 138)), 0.4, 0.5, 1.0, None),
    "shelf_blue": (lin((40, 85, 180)), 0.5, 0.3, 1.0, None),  # IMG_4953 (49, 84, 174)
    "chair_black": (lin((24, 24, 26)), 0.7, 0.0, 1.0, None),
    "chair_grey": (lin((190, 192, 195)), 0.5, 0.0, 1.0, None),
    "chrome": (lin((170, 172, 175)), 0.25, 0.9, 1.0, None),
    "glass": (lin((225, 235, 235)), 0.05, 0.0, 0.18, None),
    "plastic_white": (lin((225, 228, 228)), 0.4, 0.0, 1.0, None),  # turnstile, IMG_4952 (221, 224, 224)
}
EMISSIVE = {"led": (8.0, 8.0, 7.6)}
WRAP = {"carpet.png": "mirror", "perforated.png": "repeat"}
PHYS = {"default": (0.5, 0.45, 0.0), "laminate": (0.5, 0.4, 0.0), "carpet": (0.9, 0.8, 0.0)}


# ----------------------------------------------------------------------------- geometry helpers
def box_xyz(x0, x1, y0, y1, z0, z1):
    m = np.eye(4)
    m[:3, 3] = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=m)


def box_c(x, y, sx, sy, z0, z1):
    return box_xyz(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z0, z1)


def quad(p0, p1, p2, p3):
    return trimesh.Trimesh(vertices=[p0, p1, p2, p3], faces=[[0, 1, 2], [0, 2, 3]], process=False)


def rounded_rect(cx, cy, sx, sy, r):
    return sbox(cx - sx / 2 + r, cy - sy / 2 + r, cx + sx / 2 - r, cy + sy / 2 - r).buffer(r, quad_segs=6)


def cylinder(x, y, r, z0, z1, sections=12):
    return trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sections).apply_translation(
        [x, y, (z0 + z1) / 2]
    )


def panel(width, height, thick, r):
    """Rounded panel in the YZ plane (width along Y, height along Z from 0), thickness along X centred on 0."""
    m = trimesh.creation.extrude_polygon(rounded_rect(0, height / 2, width, height, r), thick)
    t = np.array([[0, 0, 1, -thick / 2], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])  # (x, y, z) -> (z, x, y)
    return m.apply_transform(t)


def place(m, x, y, yaw_deg, z=0.0):
    t = trimesh.transformations.rotation_matrix(math.radians(yaw_deg), [0, 0, 1])
    t[:3, 3] = [x, y, z]
    return m.copy().apply_transform(t)


# ----------------------------------------------------------------------------- USD helpers
class Layer:
    def __init__(self, path, root):
        self.stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = UsdGeom.Xform.Define(self.stage, f"/{root}")
        self.stage.SetDefaultPrim(self.root.GetPrim())
        self.rootpath = f"/{root}"
        self.mats, self.pmats = {}, {}

    def material(self, key):
        if key in self.mats:
            return self.mats[key]
        color, rough, metal, opacity, tex = MATERIALS[key]
        mat = UsdShade.Material.Define(self.stage, f"{self.rootpath}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        if key in EMISSIVE:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*EMISSIVE[key]))
        if tex:
            st = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/stReader")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            tx = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{tex}")
            tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            for w in ("wrapS", "wrapT"):
                tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set(WRAP.get(tex, "clamp"))
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def phys_material(self, key):
        if key in self.pmats:
            return self.pmats[key]
        sf, df, rs = PHYS.get(key, PHYS["default"])
        pm = UsdShade.Material.Define(self.stage, f"{self.rootpath}/PhysicsMaterials/{key}")
        api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateStaticFrictionAttr(sf)
        api.CreateDynamicFrictionAttr(df)
        api.CreateRestitutionAttr(rs)
        self.pmats[key] = pm
        return pm

    def mesh(self, path, m, mat=None, uv=None, guide=False):
        um = UsdGeom.Mesh.Define(self.stage, f"{self.rootpath}/{path}")
        pts = m.vertices.astype(np.float32)
        um.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        um.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces)))
        um.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.reshape(-1).tolist()))
        um.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        um.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))]))
        um.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.repeat(m.face_normals, 3, axis=0).astype(np.float32)))
        um.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if uv is not None:
            fv = m.vertices[m.faces.reshape(-1)]
            pv = UsdGeom.PrimvarsAPI(um).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            )
            pv.Set(Vt.Vec2fArray.FromNumpy(uv(fv).astype(np.float32)))
        if guide:
            um.CreatePurposeAttr(UsdGeom.Tokens.guide)
        elif mat:
            UsdShade.MaterialBindingAPI.Apply(um.GetPrim()).Bind(self.material(mat))
        return um

    def collider(self, path, m, pkey="default"):
        """Static convex-hull collider (guide purpose). All colliders here are boxes: 8 vertices, 4-vertex faces."""
        prim = self.mesh(path, m, guide=True).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            self.phys_material(pkey), UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    def part(self, path, m, mat, pkey="default"):
        self.mesh(path, m, mat)
        self.collider(f"Colliders/{path.replace('/', '_')}", m, pkey)

    def save(self):
        self.stage.GetRootLayer().Save()


# ----------------------------------------------------------------------------- textures
def load_photo(name):
    img = cv2.imread(str(PHOTOS / name))  # EXIF orientation applied
    k = 1024 / max(img.shape[:2])
    return cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)


def photo_px(name, pts):
    c, f = FIT[name], FIT["focal_px"]
    w, h = c["size"]
    k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
    px, _ = cv2.projectPoints(np.array(pts, np.float64), np.array(c["rvec"]), np.array(c["tvec"]), k, None)
    return px.reshape(-1, 2)


def flatten_light(img, sigma):
    img = img.astype(np.float32)
    g = cv2.GaussianBlur(img, (0, 0), sigma)
    return np.clip(img / np.maximum(g, 1) * img.reshape(-1, 3).mean(0), 0, 255)


def carpet_texture(size=(512, 512)):
    x0, x1, y0, y1 = CARPET["patch"]
    corners = [(x0, y1, 0), (x1, y1, 0), (x1, y0, 0), (x0, y0, 0)]
    w, h = size
    m = cv2.getPerspectiveTransform(
        np.float32(photo_px(CARPET["photo"], corners)), np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    )
    tex = cv2.warpPerspective(
        load_photo(CARPET["photo"]), m, size, flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE
    )
    return np.clip(flatten_light(tex, 80) * CARPET["gain"], 0, 255).astype(np.uint8)


def perforated_texture(n=512, pitch=8):
    """One 0.6 m aluminium ceiling panel: light grey with a square grid of round holes and a dark joint on the edge.
    The holes are drawn coarser than the real ones (~3 mm) so they read as the grey sheen of IMG_4953, not moire."""
    img = np.full((n, n, 3), 205, np.uint8)
    for i in range(pitch // 2, n, pitch):
        for j in range(pitch // 2, n, pitch):
            cv2.circle(img, (i, j), 2, (120, 120, 118), -1, cv2.LINE_AA)
    img[:3], img[-3:], img[:, :3], img[:, -3:] = 90, 90, 90, 90
    return img


# ----------------------------------------------------------------------------- the desk and the cabinet
def add_table(L):
    t, g = TABLE, LEGS
    hl, hw, top = t["length"] / 2, t["width"] / 2, t["top_z"]
    z_under = top - t["thick"]
    outline = rounded_rect(0, 0, t["length"], t["width"], t["corner_r"])
    slab = trimesh.creation.extrude_polygon(outline, t["thick"]).apply_translation([0, 0, z_under])
    L.mesh("Top/slab", slab, "laminate")
    L.collider("Colliders/top", box_xyz(-hl, hl, -hw, hw, z_under, top), pkey="laminate")

    (ux, uy), (lx, ly), (fw, fh) = g["upper"], g["lower"], g["foot"]
    rail_z0 = z_under - 0.04
    for sx, side in ((1, "pos"), (-1, "neg")):
        x = sx * g["x"]
        L.part(f"Legs/{side}/rail", box_c(x, 0, 0.05, t["width"] - 0.1, rail_z0, z_under), "frame_white")
        L.part(f"Legs/{side}/upper", box_c(x, g["y"], ux, uy, g["split_z"], rail_z0), "frame_white")
        L.part(f"Legs/{side}/lower", box_c(x, g["y"], lx, ly, fh, g["split_z"]), "frame_white")
        L.part(
            f"Legs/{side}/foot", box_xyz(x - fw / 2, x + fw / 2, g["foot_front"], g["foot_back"], 0, fh), "frame_white"
        )
        for end, (y0, y1) in (
            ("front", (g["foot_front"], g["foot_front"] + 0.04)),
            ("back", (g["foot_back"] - 0.04, g["foot_back"])),
        ):
            L.mesh(
                f"Legs/{side}/cap_{end}",
                box_xyz(x - fw / 2 - 0.002, x + fw / 2 + 0.002, y0, y1, 0, fh + 0.002),
                "cap_grey",
            )
    # steel cross beam between the column tops (visible in IMG_4918, IMG_4914)
    L.part("Frame/beam", box_c(0, g["y"] + 0.05, 2 * g["x"], 0.03, rail_z0 - 0.03, rail_z0), "steel")


def add_cabinet(L):
    c = CABINET
    x0, x1, y0, y1 = c["x0"], c["x0"] + c["w"], c["y1"] - c["d"], c["y1"]
    L.part("Cabinet/body", box_xyz(x0, x1, y0, y1, 0.0, c["h"]), "cabinet")
    # two by two locker doors, separated by dark seams on the front (-Y) face
    xm, zm, gap = (x0 + x1) / 2, c["h"] * 0.55, 0.004
    for i, (dx0, dx1) in enumerate(((x0 + 0.02, xm - gap), (xm + gap, x1 - 0.02))):
        for j, (dz0, dz1) in enumerate(((0.06, zm - gap), (zm + gap, c["h"] - 0.04))):
            L.mesh(f"Cabinet/door_{i}{j}", box_xyz(dx0, dx1, y0 - 0.004, y0, dz0, dz1), "cabinet")
            L.mesh(f"Cabinet/seam_{i}{j}", box_xyz(dx0 - gap, dx1 + gap, y0 - 0.002, y0, dz0 - gap, dz1 + gap), "seam")
            L.mesh(
                f"Cabinet/lock_{i}{j}",
                box_xyz(dx0 + 0.03, dx0 + 0.08, y0 - 0.012, y0 - 0.004, dz1 - 0.12, dz1 - 0.10),
                "steel",
            )


# ----------------------------------------------------------------------------- the room
def outline():
    """Room outline, clockwise seen from above (the room on the right of each wall): (name, start, end)."""
    r = ROOM
    pts = [
        ("back", (r["x_nw"], r["y_n"])),
        ("column_ne_side", (r["x_ne"], r["y_n"])),
        ("column_ne_front", (r["x_ne"], r["y_col"])),
        ("east", (r["x_e"], r["y_col"])),
        ("elevator", (r["x_e"], r["y_s"])),
        ("alcove_side", (r["x_sw"], r["y_s"])),
        ("alcove_back", (r["x_sw"], r["y_sw"])),
        ("west", (r["x_w"], r["y_sw"])),
        ("column_nw_front", (r["x_w"], r["y_col"])),
        ("column_nw_side", (r["x_nw"], r["y_col"])),
    ]
    return [(n, a, pts[(i + 1) % len(pts)][1]) for i, (n, a) in enumerate(pts)]


def add_background(L):
    r, top = ROOM, ROOM["z_ceiling"]
    x0, x1, y0, y1 = CARPET["patch"]
    L.mesh(
        "Floor",
        quad([r["x_w"], r["y_sw"], 0], [r["x_e"], r["y_sw"], 0], [r["x_e"], r["y_n"], 0], [r["x_w"], r["y_n"], 0]),
        "carpet",
        uv=lambda fv: (fv[:, :2] - [x0, y0]) / [x1 - x0, y1 - y0],
    )
    L.collider(
        "Colliders/floor", box_xyz(r["x_w"] - 1, r["x_e"] + 1, r["y_sw"] - 1, r["y_n"] + 1, -0.10, 0.0), "carpet"
    )
    for name, a, b in outline():
        d = np.subtract(b, a)
        length = float(np.linalg.norm(d))
        n_in = np.array([d[1], -d[0]]) / length  # into the room
        mat = "wall_panel" if name == "elevator" else "wall"

        def along(fv, a=a, d=d, length=length):
            return np.c_[(fv[:, :2] - a) @ (d / length), fv[:, 2]]  # metres along the wall, height

        L.mesh(f"Walls/{name}", quad((*a, 0), (*b, 0), (*b, top), (*a, top)), mat, uv=along)
        out = [np.add(a, -0.1 * n_in), np.add(b, -0.1 * n_in)]
        xs, ys = sorted([a[0], b[0], out[0][0], out[1][0]]), sorted([a[1], b[1], out[0][1], out[1][1]])
        L.collider(f"Colliders/wall_{name}", box_xyz(xs[0], xs[-1], ys[0], ys[-1], 0.0, top))
        if name != "elevator":  # skirting, 1 cm proud of the wall
            inn = [np.add(a, 0.01 * n_in), np.add(b, 0.01 * n_in)]
            sx, sy = sorted([a[0], b[0], inn[0][0], inn[1][0]]), sorted([a[1], b[1], inn[0][1], inn[1][1]])
            L.mesh(f"Walls/skirting_{name}", box_xyz(sx[0], sx[-1], sy[0], sy[-1], 0.0, SKIRTING), "skirting")
    add_ceiling(L)
    add_elevators(L)


def add_ceiling(L):
    r = ROOM
    zc, zs, t = r["z_ceiling"], r["z_soffit"], PERFORATED_TILE
    # perforated panels between the two soffits; facing down (closes the room: without a ceiling RoboDojo's default
    # brown studio HDR floods the scene from above)
    L.mesh(
        "Ceiling",
        quad(
            [r["x_w"], r["y_soffit_s"], zc],
            [r["x_w"], r["y_col"], zc],
            [r["x_e"], r["y_col"], zc],
            [r["x_e"], r["y_soffit_s"], zc],
        ),  # fmt: skip
        "perforated",
        uv=lambda fv: fv[:, :2] / t,
    )
    L.mesh("Soffit/back", box_xyz(r["x_nw"], r["x_ne"], r["y_col"], r["y_n"], zs, zc), "ceiling")
    L.mesh("Soffit/elevator_side", box_xyz(r["x_w"], r["x_e"], r["y_sw"], r["y_soffit_s"], zs, zc), "ceiling")
    for i, x in enumerate(STRIP_X):
        L.mesh(f"CeilingLights/strip_{i}", box_c(x, sum(STRIP_Y) / 2, LIGHTS["strip_width"], STRIP_Y[0] - STRIP_Y[1],
                                                  zc - 0.012, zc - 0.002), "led")  # fmt: skip
    L.mesh("CeilingLights/slot", box_c(SLOT_X, sum(STRIP_Y) / 2, 0.04, STRIP_Y[0] - STRIP_Y[1], zc - 0.004, zc), "slot")
    for i, (x, y) in enumerate(DOWNLIGHTS_N + DOWNLIGHTS_S):
        L.mesh(f"CeilingLights/downlight_{i}", cylinder(x, y, 0.06, zs - 0.004, zs, sections=16), "led")


def add_elevators(L):
    y, zs = ROOM["y_s"], ROOM["z_soffit"]
    for i, (x0, x1) in enumerate(ELEVATORS):
        L.mesh(f"Elevators/kt{4 - i}/surround", box_xyz(x0, x1, y, y + 0.02, 0, zs), "wood_dark")
        d0, d1 = x0 + 0.125, x1 - 0.125
        L.mesh(f"Elevators/kt{4 - i}/doors", box_xyz(d0, d1, y + 0.02, y + 0.035, 0, 2.2), "bronze")
        L.mesh(f"Elevators/kt{4 - i}/door_gap", box_c((d0 + d1) / 2, y + 0.036, 0.006, 0.002, 0, 2.2), "seam")


# ----------------------------------------------------------------------------- furniture
def add_desk_row(L):
    d = DESKS
    xf, xb, top, th = d["x_front"], d["x_back"], d["top_z"], 0.025
    (sw, sh), (py, px, pz) = d["screen"], d["pedestal"]
    base = "Furniture/DeskRow"
    for k in range(d["n"] + 1):  # loop legs at every module joint: front and back posts, foot and top rails
        y = d["y_north"] - k * d["module"] + (0.03 if k == d["n"] else -0.03 if k == 0 else 0)
        for j, x in enumerate((xf + 0.06, xb - 0.06)):
            L.part(f"{base}/leg_{k}/post_{j}", box_c(x, y, 0.05, 0.05, 0.0, top - th), "frame_white")
        L.part(f"{base}/leg_{k}/foot", box_xyz(xf + 0.03, xb - 0.03, y - 0.025, y + 0.025, 0.0, 0.04), "frame_white")
        L.mesh(f"{base}/leg_{k}/rail", box_xyz(xf + 0.03, xb - 0.03, y - 0.025, y + 0.025, top - th - 0.05, top - th),
               "frame_white")  # fmt: skip
    for k in range(d["n"]):
        yn = d["y_north"] - k * d["module"]
        ys, yc = yn - d["module"], yn - d["module"] / 2
        L.part(f"{base}/module_{k}/top", box_xyz(xf, xb, ys + 0.001, yn - 0.001, top - th, top), "laminate", "laminate")
        scr = place(panel(sw, sh, 0.025, 0.03), xb - 0.03, yc, 0, top + 0.02)
        L.part(f"{base}/module_{k}/screen", scr, "screen_blue")
        p0 = ys + 0.06
        L.part(f"{base}/module_{k}/pedestal", box_xyz(xf + 0.03, xf + 0.03 + px, p0, p0 + py, 0.03, 0.03 + pz),
               "frame_white")  # fmt: skip
        for j, z in enumerate((0.25, 0.43)):  # drawer seams on the front (-X) face
            seam = box_xyz(xf + 0.028, xf + 0.031, p0 + 0.01, p0 + py - 0.01, z, z + 0.004)
            L.mesh(f"{base}/module_{k}/drawer_{j}", seam, "seam")


def add_lift_desk(L):
    d = LIFT_DESK
    x0, x1 = d["x0"], d["x0"] + d["length"]
    y1 = ROOM["y_n"]
    y0, xc, yc, top = y1 - d["width"], (x0 + x1) / 2, y1 - d["width"] / 2, d["top_z"]
    base = "Furniture/LiftDesk"
    slab = trimesh.creation.extrude_polygon(rounded_rect(xc, yc, d["length"], d["width"], 0.03), 0.025)
    L.part(f"{base}/top", slab.apply_translation([0, 0, top - 0.025]), "lift_top", "laminate")
    for side, x in (("pos", x1 - 0.12), ("neg", x0 + 0.12)):
        L.part(f"{base}/{side}/column", box_c(x, yc, 0.07, 0.05, 0.035, top - 0.025), "frame_grey")
        L.part(f"{base}/{side}/foot", box_xyz(x - 0.035, x + 0.035, y0 + 0.03, y1 - 0.03, 0, 0.035), "frame_grey")
        L.mesh(f"{base}/{side}/rail", box_c(x, yc, 0.05, d["width"] - 0.1, top - 0.065, top - 0.025), "frame_grey")
    L.mesh(f"{base}/beam", box_c(xc, yc, d["length"] - 0.24, 0.04, top - 0.095, top - 0.065), "frame_grey")


def chair_parts(style):
    """An office chair in its own frame: floor point under the seat at the origin, the seat facing +X. Returns
    [(name, mesh, material)] and the collider boxes. "task": black mesh back; "ergo": the back in a light grey frame."""
    parts = []
    for i in range(5):  # five-star base with casters
        a = math.radians(72 * i + 18)
        leg = box_xyz(0.0, 0.31, -0.02, 0.02, 0.07, 0.10).apply_transform(
            trimesh.transformations.rotation_matrix(a, [0, 0, 1])
        )
        parts.append((f"leg_{i}", leg, "chrome"))
        parts.append(
            (f"caster_{i}", cylinder(0.31 * math.cos(a), 0.31 * math.sin(a), 0.025, 0.0, 0.06, 8), "chair_black")
        )
    parts.append(("column", cylinder(0, 0, 0.025, 0.10, 0.42), "chrome"))
    parts.append(("sleeve", cylinder(0, 0, 0.035, 0.08, 0.26), "chair_black"))
    seat = trimesh.creation.extrude_polygon(rounded_rect(0.02, 0, 0.50, 0.50, 0.07), 0.08).apply_translation(
        [0, 0, 0.42]
    )
    parts.append(("seat", seat, "chair_black"))
    tilt = trimesh.transformations.rotation_matrix(math.radians(-12), [0, 1, 0], [-0.22, 0, 0.55])
    back = panel(0.46, 0.58, 0.04, 0.12).apply_translation([-0.22, 0, 0.56]).apply_transform(tilt)
    parts.append(("back", back, "chair_black"))
    if style == "ergo":
        frame = panel(0.52, 0.64, 0.03, 0.14).apply_translation([-0.255, 0, 0.53]).apply_transform(tilt)
        parts.append(("back_frame", frame, "chair_grey"))
    head = panel(0.30, 0.14, 0.05, 0.05).apply_translation([-0.26, 0, 1.17]).apply_transform(tilt)
    parts.append(("headrest", head, "chair_black"))
    for s in (-1, 1):
        parts.append((f"arm_post_{s + 1}", box_c(-0.02, 0.27 * s, 0.04, 0.03, 0.45, 0.66), "chair_black"))
        parts.append((f"arm_pad_{s + 1}", box_c(0.02, 0.27 * s, 0.25, 0.07, 0.66, 0.69), "chair_black"))
    colliders = [box_c(0, 0, 0.62, 0.62, 0.0, 0.12), box_c(0.02, 0, 0.5, 0.5, 0.42, 0.50),
                 box_c(-0.26, 0, 0.08, 0.5, 0.52, 1.32).apply_transform(tilt)]  # fmt: skip
    return parts, colliders


def chair_xy(pos):
    """Measured floor point (+-0.1 m), kept 0.36 m clear of the desk row front and 0.42 m clear of the column faces."""
    x, y = pos[0], pos[1]
    if x > DESKS["x_front"] - 1.0:
        x = min(x, DESKS["x_front"] - 0.36)
    return r3(x), r3(min(y, ROOM["y_col"] - 0.42))


def add_chairs(L):
    for i, (pos, yaw, style) in enumerate(CHAIRS):
        pos = chair_xy(pos)
        parts, colliders = chair_parts(style)
        for name, m, mat in parts:
            L.mesh(f"Furniture/Chairs/chair_{i}/{name}", place(m, pos[0], pos[1], yaw), mat)
        for j, m in enumerate(colliders):
            L.collider(f"Colliders/chair_{i}_{j}", place(m, pos[0], pos[1], yaw))


def add_shelf(L):
    s = SHELF
    x0, x1 = ROOM["x_w"] + 0.01, ROOM["x_w"] + 0.01 + s["depth"]
    base = "Furniture/Shelf"
    for i, (x, y) in enumerate([(x0 + 0.02, s["y0"] + 0.02), (x0 + 0.02, s["y1"] - 0.02), (x1 - 0.02, s["y0"] + 0.02),
                                (x1 - 0.02, s["y1"] - 0.02)]):  # fmt: skip
        L.part(f"{base}/post_{i}", box_c(x, y, 0.04, 0.04, 0.0, s["height"]), "shelf_blue")
    for i, z in enumerate(s["levels"]):
        L.part(f"{base}/board_{i}", box_xyz(x0 + 0.04, x1 - 0.04, s["y0"] + 0.04, s["y1"] - 0.04, z - 0.03, z),
               "shelf_blue")  # fmt: skip


def add_railing(L):
    rl = RAILING
    h, base, k = rl["height"], "Furniture/Railing", 0
    for run in rl["runs"]:
        for a, b in zip(run[:-1], run[1:]):
            length = float(np.hypot(b[0] - a[0], b[1] - a[1]))
            n = max(1, math.ceil(length / 1.0))
            pts = [np.add(a, np.subtract(b, a) * t / n) for t in range(n + 1)]
            for p, q in zip(pts[:-1], pts[1:]):
                L.mesh(f"{base}/post_{k}", cylinder(p[0], p[1], 0.025, 0.0, h), "chrome")
                lo, hi = np.minimum(p, q), np.maximum(p, q)
                pad = np.where(hi - lo < 1e-6, 0.006, -0.03)  # glass 12 mm thick, 3 cm clear of each post
                L.part(f"{base}/glass_{k}", box_xyz(lo[0] - pad[0], hi[0] + pad[0], lo[1] - pad[1], hi[1] + pad[1],
                                                    0.06, h - 0.05), "glass")  # fmt: skip
                k += 1
            L.mesh(f"{base}/post_{k}", cylinder(pts[-1][0], pts[-1][1], 0.025, 0.0, h), "chrome")
            k += 1
    for i, (x, y) in enumerate(rl["turnstile"]):  # east pillar first; each flap reaches 0.26 m into the passage
        L.part(f"{base}/turnstile_{i}/pillar", box_c(x, y, 0.16, 0.5, 0.0, 1.0), "plastic_white")
        fx = (x - 0.34, x - 0.08) if i == 0 else (x + 0.08, x + 0.34)
        L.mesh(f"{base}/turnstile_{i}/flap", box_xyz(*fx, y - 0.006, y + 0.006, 0.35, 0.95), "glass")


# ----------------------------------------------------------------------------- lights
def add_lights(L):
    lt, zc, zs = LIGHTS, ROOM["z_ceiling"], ROOM["z_soffit"]
    for i, x in enumerate(STRIP_X):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/CeilingStrip_{i}")
        rl.CreateWidthAttr(lt["strip_width"])
        rl.CreateHeightAttr(STRIP_Y[0] - STRIP_Y[1])
        rl.CreateIntensityAttr(lt["strip_intensity"])
        rl.CreateEnableColorTemperatureAttr(True)
        rl.CreateColorTemperatureAttr(lt["color_temperature"])
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(x, sum(STRIP_Y) / 2, zc - 0.015))
    for i, (x, y) in enumerate(DOWNLIGHTS_N):
        dl = UsdLux.DiskLight.Define(L.stage, f"{L.rootpath}/Downlight_{i}")
        dl.CreateRadiusAttr(lt["downlight_radius"])
        dl.CreateIntensityAttr(lt["downlight_intensity"])
        dl.CreateEnableColorTemperatureAttr(True)
        dl.CreateColorTemperatureAttr(lt["color_temperature"])
        shaping = UsdLux.ShapingAPI.Apply(dl.GetPrim())
        shaping.CreateShapingConeAngleAttr(lt["downlight_cone_deg"])
        shaping.CreateShapingConeSoftnessAttr(0.4)
        UsdGeom.Xformable(dl).AddTranslateOp().Set(Gf.Vec3d(x, y, zs - 0.006))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.rootpath}/AmbientDome")
    dome.CreateIntensityAttr(lt["dome"])
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))


def add_furniture(L):
    add_desk_row(L)
    add_lift_desk(L)
    add_chairs(L)
    add_shelf(L)
    add_railing(L)


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    TEX.mkdir(parents=True)
    cv2.imwrite(str(TEX / "carpet.png"), carpet_texture())
    cv2.imwrite(str(TEX / "perforated.png"), perforated_texture())
    layers = [
        ("background", lambda L: (add_background(L), add_furniture(L))),
        ("table", add_table),
        ("cabinet", add_cabinet),
        ("lights", add_lights),
    ]
    for name, fn in layers:
        layer = Layer(OUT / f"{name}.usda", name.capitalize())
        fn(layer)
        layer.save()
    names = [n.capitalize() for n, _ in layers]

    st = Usd.Stage.CreateNew(str(OUT / "scene.usda"))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(st, "/World/PhysicsScene").CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    for nm in names:
        st.DefinePrim(f"/World/{nm}").GetReferences().AddReference(f"./{nm.lower()}.usda")
    st.GetRootLayer().Save()
    # RoboDojo Room entry (layout_manager.select_room loads the only .usd in Assets/Room/<name>/): no PhysicsScene,
    # the env owns it. Installed by scripts/g1d/build_asset.py.
    room = Usd.Stage.CreateNew(str(HERE / f"{ROOM_NAME}.usd"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    scene = UsdGeom.Xform.Define(room, "/Scene")
    room.SetDefaultPrim(scene.GetPrim())
    for nm in names:
        room.DefinePrim(f"/Scene/{nm}").GetReferences().AddReference(f"./usd/{nm.lower()}.usda")
    room.GetRootLayer().Save()

    t = TABLE
    params = dict(
        frame="Z-up, meters; origin on the floor under the tabletop centre; +X along the table; +Y to the wall",
        table=dict(top_z=t["top_z"], length=t["length"], width=t["width"], front_y=-t["width"] / 2, thick=t["thick"]),
        legs=LEGS,
        cabinet=CABINET,
        room=ROOM,
        outline=[[n, list(a), list(b)] for n, a, b in outline()],
        ceiling=dict(
            strip_x=STRIP_X, slot_x=SLOT_X, strip_y=list(STRIP_Y), downlights_n=DOWNLIGHTS_N, downlights_s=DOWNLIGHTS_S
        ),  # fmt: skip
        elevators=ELEVATORS,
        desks=DESKS,
        lift_desk=LIFT_DESK,
        shelf=SHELF,
        chairs=[dict(pos=list(chair_xy(p)), yaw=y, style=s) for p, y, s in CHAIRS],
        railing=RAILING,
        lights=LIGHTS,
        layers=dict(scene="scene.usda", **{n: f"{n}.usda" for n, _ in layers}),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    for f in OUT.glob("*.usda"):  # USD ends text layers with a blank line, which `git diff --check` rejects
        f.write_text(f.read_text().rstrip("\n") + "\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
