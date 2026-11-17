#!/usr/bin/env python3
# Ballistic Dynamics — independent reference trajectories.
# SPDX-License-Identifier: MIT
#
# Runs py-ballisticcalc (https://github.com/o-murphy/py-ballisticcalc, LGPL,
# used only as an outside program: none of its code is in this repository), a
# point-mass solver in the JBM tradition, on the Milestone 2 cartridges and
# writes tests/reference/point_mass.json. tests/test_point_mass.cpp compares
# the engine against it. This is the "JBM-style solution" check of the brief;
# published manufacturer figures are checked separately (docs/validation.md).
#
#   python3 -m venv /tmp/pbc && /tmp/pbc/bin/pip install py-ballisticcalc==3.1.1
#   /tmp/pbc/bin/python tools/reference/make_reference.py
#
# Conditions: ICAO standard atmosphere at sea level (15 C, 1013.25 hPa, dry),
# a flat range, line of sight horizontal, no spin drift (twist 0), no Coriolis,
# a 10 mph full-value crosswind from 3 o'clock.
import json
import os
import sys
import warnings

import py_ballisticcalc as pbc
from py_ballisticcalc import (Ammo, Angular, Atmo, Calculator, DragModel, Shot, TableG1, TableG7, Unit, Weapon,
                              Wind)

warnings.filterwarnings("ignore", category=DeprecationWarning)

CASES = [
    # id, model, bc (lb/in^2), mass gr, diameter in, mv fps, zero yd, sight in, ranges yd
    ("308_175_g1", "G1", 0.505, 175, 0.308, 2600, 100, 1.5, list(range(0, 1001, 100))),
    ("308_175_g7", "G7", 0.243, 175, 0.308, 2600, 100, 1.5, list(range(0, 1001, 100))),
    ("65cm_140_g1", "G1", 0.646, 140, 0.264, 2710, 100, 1.5, list(range(0, 1001, 100))),
    ("65cm_140_g7", "G7", 0.326, 140, 0.264, 2710, 100, 1.5, list(range(0, 1001, 100))),
    ("338lm_250_g1", "G1", 0.648, 250, 0.338, 2920, 100, 1.5, list(range(0, 1501, 100))),
    ("338lm_250_g7", "G7", 0.320, 250, 0.338, 2920, 100, 1.5, list(range(0, 1501, 100))),
    ("50bmg_750_g1", "G1", 1.050, 750, 0.510, 2820, 100, 1.5, list(range(0, 2001, 100))),
    ("m855_62_g1", "G1", 0.304, 62, 0.224, 3020, 100, 1.5, list(range(0, 801, 100))),
    ("9mm_124_g1", "G1", 0.134, 124, 0.355, 1150, 25, 0.5, list(range(0, 101, 25))),
]


def run(case):
    cid, model, bc, gr, dia, mv, zero, sight, ranges = case
    table = TableG1 if model == "G1" else TableG7
    weapon = Weapon(sight_height=Unit.Inch(sight), twist=Unit.Inch(0))
    ammo = Ammo(DragModel(bc, table, Unit.Grain(gr), Unit.Inch(dia), Unit.Inch(1.0)), mv=Unit.FPS(mv))
    atmo = Atmo.icao()
    calc = Calculator()
    calc.set_weapon_zero(Shot(weapon=weapon, ammo=ammo, atmo=atmo), Unit.Yard(zero))
    shot = Shot(weapon=weapon, ammo=ammo, atmo=atmo, winds=[Wind(Unit.MPH(10), Angular.OClock(3))])
    step = ranges[1] - ranges[0]
    res = calc.fire(shot, Unit.Yard(ranges[-1]), trajectory_step=Unit.Yard(step), raise_range_error=False)
    rows = []
    for r in res.records:
        yd = r.distance >> Unit.Yard
        if min(abs(yd - x) for x in ranges) > 1e-6:
            continue
        rows.append({
            "range_yd": round(yd, 6),
            "drop_in": round(r.height >> Unit.Inch, 4),
            # py-ballisticcalc reports drift away from a 3 o'clock wind as
            # positive; the engine's windage is positive right, so it is negative.
            "windage_in": round(-(r.windage >> Unit.Inch), 4),
            "velocity_fps": round(r.velocity >> Unit.FPS, 3),
            "time_s": round(r.time, 6),
            "mach": round(r.mach, 6),
        })
    return {"id": cid, "model": model, "bc": bc, "mass_gr": gr, "diameter_in": dia, "muzzle_velocity_fps": mv,
            "zero_yd": zero, "sight_height_in": sight, "crosswind_mph": 10, "rows": rows}


def main():
    out = {
        "comment": "Reference point-mass trajectories from py-ballisticcalc " + pbc.__version__ +
                   " (tools/reference/make_reference.py). ICAO sea level, dry; line of sight horizontal; "
                   "no spin drift or Coriolis; 10 mph crosswind from 3 o'clock. Drop is relative to the line of "
                   "sight; windage is positive right.",
        "cases": [run(c) for c in CASES],
    }
    path = os.path.join(os.path.dirname(__file__), "..", "..", "tests", "reference", "point_mass.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print("wrote", os.path.normpath(path), file=sys.stderr)


if __name__ == "__main__":
    main()
