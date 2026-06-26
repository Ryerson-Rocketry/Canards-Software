#!/usr/bin/env python3
"""
mag_calibrate.py - hard-iron calibration for the MMC5983MA from a tumble log.

Reads a CSV in this firmware's LOG.CSV format, fits the best sphere to the
magnetometer samples, and prints the hard-iron offset as a ready-to-paste C
constant for Core/Src/Drivers/mmc5983ma.c (MAG_HARD_IRON).

WHY: a hard-iron offset shifts the magnetometer's sphere off-center, so a real
rotation only sweeps a small heading arc (yours read ~21 deg for a true 90).
Subtracting the offset re-centers the sphere so heading tracks rotation 1:1.

USAGE:
    python tools/mag_calibrate.py LOG.CSV

No third-party packages required (pure standard library).

The log must be a TUMBLE: rotate the assembled, powered rocket slowly through
ALL orientations so the samples cover the whole sphere. A single-axis spin is
NOT enough - it only traces one circle and the fit will be poor (the script
warns you if the coverage looks too flat). Collect it with MAG_HARD_IRON still
{0,0,0} so the fit sees raw data.
"""

import csv
import math
import sys

MAG_SCALE = 100.0  # the log stores mag * 100 ("scaled x100")


def load_mag(path):
    """Pull mag_x/mag_y/mag_z out of the CSV by header name, unscale to Gauss."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {}
        for i, name in enumerate(header):
            n = name.strip().lower()
            if n.startswith("mag_x"):
                idx["x"] = i
            elif n.startswith("mag_y"):
                idx["y"] = i
            elif n.startswith("mag_z"):
                idx["z"] = i
        if len(idx) != 3:
            sys.exit("Could not find mag_x/mag_y/mag_z columns in the CSV header.")

        pts, last = [], max(idx.values())
        for row in reader:
            if len(row) <= last:
                continue
            try:
                pts.append(tuple(float(row[idx[a]]) / MAG_SCALE for a in ("x", "y", "z")))
            except ValueError:
                continue  # skip blank/garbage rows
    return pts


def solve_linear(A, b):
    """Solve A x = b (square) via Gaussian elimination with partial pivoting."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            sys.exit("Fit failed: data is degenerate (tumble more thoroughly).")
        M[col], M[piv] = M[piv], M[col]
        for r in range(n):
            if r != col:
                f = M[r][col] / M[col][col]
                for c in range(col, n + 1):
                    M[r][c] -= f * M[col][c]
    return [M[i][n] / M[i][i] for i in range(n)]


def sphere_fit(pts):
    """Least-squares sphere: x^2+y^2+z^2 = a x + b y + c z + d -> center=(a,b,c)/2."""
    Sxx = Sxy = Sxz = Sx = Syy = Syz = Sy = Szz = Sz = 0.0
    Trx = Try = Trz = Tr = 0.0
    for x, y, z in pts:
        r2 = x * x + y * y + z * z
        Sxx += x * x; Sxy += x * y; Sxz += x * z; Sx += x
        Syy += y * y; Syz += y * z; Sy += y
        Szz += z * z; Sz += z
        Trx += x * r2; Try += y * r2; Trz += z * r2; Tr += r2
    n = float(len(pts))
    A = [[Sxx, Sxy, Sxz, Sx],
         [Sxy, Syy, Syz, Sy],
         [Sxz, Syz, Szz, Sz],
         [Sx,  Sy,  Sz,  n]]
    sol = solve_linear(A, [Trx, Try, Trz, Tr])
    cx, cy, cz = sol[0] / 2.0, sol[1] / 2.0, sol[2] / 2.0
    radius = math.sqrt(max(sol[3] + cx * cx + cy * cy + cz * cz, 0.0))
    return (cx, cy, cz), radius


def spread(pts, center):
    """mean/std of |m - center|; a good fit makes std small relative to mean."""
    cx, cy, cz = center
    norms = [math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2) for x, y, z in pts]
    m = sum(norms) / len(norms)
    var = sum((v - m) ** 2 for v in norms) / len(norms)
    return m, math.sqrt(var)


def _sym3_eigvals(a11, a22, a33, a12, a13, a23):
    """Eigenvalues of a symmetric 3x3 matrix (analytic)."""
    p1 = a12 * a12 + a13 * a13 + a23 * a23
    if p1 == 0.0:
        return sorted([a11, a22, a33])
    q = (a11 + a22 + a33) / 3.0
    p2 = (a11 - q) ** 2 + (a22 - q) ** 2 + (a33 - q) ** 2 + 2 * p1
    p = math.sqrt(p2 / 6.0)
    b11, b22, b33 = (a11 - q) / p, (a22 - q) / p, (a33 - q) / p
    b12, b13, b23 = a12 / p, a13 / p, a23 / p
    detB = (b11 * (b22 * b33 - b23 * b23)
            - b12 * (b12 * b33 - b23 * b13)
            + b13 * (b12 * b23 - b22 * b13))
    r = max(-1.0, min(1.0, detB / 2.0))
    phi = math.acos(r) / 3.0
    e1 = q + 2 * p * math.cos(phi)
    e3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    return sorted([e1, 3 * q - e1 - e3, e3])


def coverage(pts):
    """0..1: ratio of smallest to largest principal spread. ~0 = flat (1 axis)."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    mz = sum(p[2] for p in pts) / n
    cxx = cxy = cxz = cyy = cyz = czz = 0.0
    for x, y, z in pts:
        dx, dy, dz = x - mx, y - my, z - mz
        cxx += dx * dx; cxy += dx * dy; cxz += dx * dz
        cyy += dy * dy; cyz += dy * dz; czz += dz * dz
    eigs = [max(e, 0.0) for e in _sym3_eigvals(cxx, cyy, czz, cxy, cxz, cyz)]
    lo, hi = min(eigs), max(eigs)
    return math.sqrt(lo / hi) if hi > 0 else 0.0


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: python tools/mag_calibrate.py LOG.CSV")
    pts = load_mag(sys.argv[1])
    if len(pts) < 50:
        sys.exit(f"Only {len(pts)} samples - not enough to fit. Tumble and log longer.")

    print(f"# samples:             {len(pts)}")
    if len(pts) < 200:
        print("# WARNING: few samples - tumble longer for a better fit.")

    cov = coverage(pts)
    print(f"# coverage (0..1):     {cov:.3f}   "
          + ("(good 3D tumble)" if cov > 0.15 else
             "(TOO FLAT - looks like a single-axis spin; rotate about ALL axes!)"))

    rmean, rstd = spread(pts, (0.0, 0.0, 0.0))
    print(f"# raw    |m|: mean={rmean:.3f} std={rstd:.3f}  ({100 * rstd / rmean:.1f}% of mean)")

    center, radius = sphere_fit(pts)
    cmean, cstd = spread(pts, center)
    print(f"# fit radius:          {radius:.3f}")
    print(f"# after  |m|: mean={cmean:.3f} std={cstd:.3f}  ({100 * cstd / cmean:.1f}% of mean)")
    print("# (good calibration -> 'after' std a few % of mean; lower is better)")
    print()
    print("// ---- paste into Core/Src/Drivers/mmc5983ma.c (replace MAG_HARD_IRON) ----")
    print("static const float MAG_HARD_IRON[3] = "
          f"{{{center[0]:.6f}f, {center[1]:.6f}f, {center[2]:.6f}f}};")


if __name__ == "__main__":
    main()
