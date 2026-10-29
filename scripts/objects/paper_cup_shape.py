"""Paper cup cross-section, numpy only: shared by make_paper_cup.py (rigid Clutter/paper_cup) and anything else that
needs the same cup (the deformable twin, task-side checks).

Clutter/paper_cup/00000 is a single-wall paper cup, black outside, white inside and on the rolled rim. Measured on the
Real2Sim platform (spec.json updated 2026-10-29 13:46:11 by 徐鹤鸣): mouth outer diameter over the rim bead 87.9 mm,
bottom diameter 60.5 mm, height 136.0 mm, 22 g. The rest of CUP is estimated from the three reference photos.

Half-plane (r, z) in metres, z up from the lowest point of the folded bottom edge, which the cup stands on:
- straight conical wall r_outer(z) from the bottom fold up into the rolled rim bead (a torus whose top is z = H);
- the floor disc is recessed: its top is FLOOR_Z above the bottom edge; below it the wall goes on as a skirt whose edge
  is folded inwards (SKIRT_T thick); a small crease ring where the floor meets the wall (the step ring seen in pc_2);
- SLEEVE > 0 adds a second outer wall (double-wall cups) from the bottom up to SLEEVE_TOP below the rim.
"""

import numpy as np

CUP = dict(
    MOUTH_D=0.0879,  # measured: outer diameter over the rolled rim bead
    BOTTOM_D=0.0605,  # measured: outer diameter at the bottom edge (widest point of the fold)
    H=0.136,  # measured: bottom edge to top of the rim bead
    BEAD=0.0034,  # estimated (photos: white rolled rim 3-4 mm thick)
    T_WALL=0.0007,  # estimated: visual wall; real paper is ~0.4 mm, thickened to render and collide robustly
    T_OUTER=0.0004,  # estimated: outer-coloured share of the wall, the rest shows the inner colour
    FLOOR_Z=0.0100,  # estimated (photos): top of the recessed floor above the bottom edge
    T_FLOOR=0.0008,  # estimated
    SKIRT_T=0.0015,  # estimated: skirt below the floor, the wall folded in around the floor flange
    GROOVE_W=0.0008,  # estimated: crease ring where the floor meets the wall
    GROOVE_D=0.0005,  # estimated
    SLEEVE=0.0,  # double-wall variants: gap of the outer sleeve (0: single wall)
    SLEEVE_TOP=0.008,  # double-wall variants: sleeve top below the rim top
)


def arc(cx, cz, rad, a0, a1, n):
    a = np.linspace(a0, a1, n)
    return [(float(cx + rad * np.cos(t)), float(cz + rad * np.sin(t))) for t in a]


def paper_cup_profile(params=None):
    """Cross-section of the cup for `params` (overrides of CUP).

    Returns a dict: scalars H, z_top (wall top, inside the bead at its centre height), bead_rc / bead_r (bead centre
    radius, tube radius), fold (bottom fold radius, also its centre height), floor_z / floor_bottom (recessed floor
    top / underside), sleeve_z (sleeve top centre, None for single wall), the inputs as `params`; functions of z:
    r_outer (outer surface of the inner wall), r_inner (cavity wall), r_shell (outermost surface incl. sleeve),
    r_skirt (inside of the skirt below the floor).
    """
    p = {**CUP, **(params or {})}
    H, rb, S = p["H"], p["BEAD"] / 2, p["SLEEVE"]
    rc = p["MOUTH_D"] / 2 - rb
    z_top = H - rb
    r_top = rc - rb + p["T_WALL"]  # the cavity wall meets the bead's inner equator
    fold = (p["SKIRT_T"] + S) / 2
    r_bot = p["BOTTOM_D"] / 2 - S  # BOTTOM_D is over the sleeve when there is one
    k = (r_top - r_bot) / (z_top - fold)
    sleeve_z = H - p["SLEEVE_TOP"] if S > 0 else None

    def r_outer(z):
        return r_bot + k * (np.asarray(z, dtype=float) - fold)

    def r_inner(z):
        return r_outer(z) - p["T_WALL"]

    def r_shell(z):
        z = np.asarray(z, dtype=float)
        return r_outer(z) + (S * (z <= sleeve_z) if S > 0 else 0.0)

    def r_skirt(z):
        return r_outer(z) - p["SKIRT_T"]

    return dict(
        params=p, H=H, z_top=z_top, bead_rc=rc, bead_r=rb, fold=fold, floor_z=p["FLOOR_Z"],
        floor_bottom=p["FLOOR_Z"] - p["T_FLOOR"], sleeve_z=sleeve_z, slope=k,
        r_outer=r_outer, r_inner=r_inner, r_shell=r_shell, r_skirt=r_skirt,
    )  # fmt: skip


def section_loops(prof, n_arc=10, n_bead=41):
    """Closed (r, z) outlines of the cup parts, counter-clockwise (outward normals when revolved). A repeated point is a
    sharp edge. Returns {name: (points, welded)}; welded loops end on their (smooth) start point.

    model: outer wall + sleeve + skirt + folded bottom edge (outer colour, outer T_OUTER of the wall); inner: cavity
    wall (inner colour); rim: rolled bead; floor: recessed floor disc (starts/ends on the axis); crease: bevel ring
    where the floor meets the wall.
    """
    p, ro, ri, rk = prof["params"], prof["r_outer"], prof["r_inner"], prof["r_skirt"]
    zt, zb, zf, f = prof["z_top"], prof["floor_bottom"], prof["floor_z"], prof["fold"]
    S, TO = p["SLEEVE"], p["T_OUTER"]
    cx = float(rk(f)) + f
    model = arc(cx, f, f, np.pi, 2 * np.pi, n_arc)
    if S > 0:
        zs = prof["sleeve_z"]
        model += arc(float(ro(zs)) + S / 2, zs, S / 2, 0.0, np.pi, n_arc)
        model.append(model[-1])
    top = (float(ro(zt)), zt)
    model += [top, top]
    model += [(top[0] - TO, zt)] * 2
    model += [(float(ro(zb)) - TO, zb)] * 2
    model += [(float(rk(zb)), zb)] * 2
    model.append(model[0])

    a, b, c, d = (float(ro(zb)) - TO, zb), (float(ro(zt)) - TO, zt), (float(ri(zt)), zt), (float(ri(zb)), zb)
    inner = [a, b, b, c, c, d, d, a]

    rim = arc(prof["bead_rc"], zt, prof["bead_r"], np.pi, 3 * np.pi, n_bead)
    rim[-1] = rim[0]

    z0 = zb + 0.0001  # just above the skirt's ceiling, so the two faces do not overlap
    rf, gw, gd = float(ri(z0)), p["GROOVE_W"], p["GROOVE_D"]
    a, b, c = (rf - gw, zf - gd), (rf, zf - gd), (rf - gw, zf)
    floor = [(0.0, z0), (rf, z0), (rf, z0), b, b, a, a, c, c, (0.0, zf)]
    crease = [a, b, b, c, c, a]
    return {
        "model": (model, True),
        "inner": (inner, False),
        "rim": (rim, True),
        "floor": (floor, False),
        "crease": (crease, False),
    }


def volume_ml(prof, n=400):
    """Brim-full capacity: floor top to the wall top."""
    z = np.linspace(prof["floor_z"], prof["z_top"], n)
    a = np.pi * prof["r_inner"](z) ** 2
    return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def paper_area(prof):
    """Paper blank area (m^2): side wall incl. the rolled rim and the folded skirt, floor disc, sleeve."""
    p, ro = prof["params"], prof["r_outer"]

    def cone(z0, z1):
        r0, r1 = float(ro(z0)), float(ro(z1))
        return np.pi * (r0 + r1) * np.hypot(r1 - r0, z1 - z0)

    area = cone(0.0, prof["z_top"]) + 2 * np.pi * prof["bead_rc"] * np.pi * prof["bead_r"] * 2
    area += 2 * np.pi * float(ro(0.0)) * prof["floor_bottom"]  # folded-in skirt
    area += np.pi * float(prof["r_inner"](prof["floor_bottom"]) + 0.004) ** 2  # floor disc with its flange
    if p["SLEEVE"] > 0:
        area += cone(0.0, prof["sleeve_z"])
    return float(area)
