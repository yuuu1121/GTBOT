#!/usr/bin/env python3
"""Pure-numpy Python port of the ouster_cluster BEV plate-detection pipeline
(elevation cut -> preprocess crop -> BEV density -> BEV line extraction ->
plate classification), for offline replay/debugging against captured point
clouds. No creative changes: logic is carried over 1:1 from the C++ source.

Ported from (C++ source, ROS2 package ouster_cluster):
  - elevation_filter.hpp  (filterElevation)              -> elevation_cut()
  - plane_detect.hpp      (preprocessCloud, range/z crop  -> preprocess()
                            portion only)
  - bev_density.hpp       (computeBevDensity)             -> compute_bev_density()
  - bev_lines.hpp         (extractBevLineSegments,         -> extract_bev_line_segments()
                            isPlateSegment)                -> is_plate_segment()

Deliberately NOT ported (all OFF in the sim operating params in
gtbot_formation/launch/perception.launch.py, so out of scope for this replay
tool):
  - refineBevSegment / refineBevSegments  (bev_refine_enable: False)
  - passesReflectivityGate                (bev_refl_enable: False)
  - BevSegmentFilter (temporal confirm/hold hysteresis) — this tool reports
    PER-FRAME stage survival, which is exactly what temporal confirmation
    would filter out; not meaningful for single-frame replay.
  - voxel-grid downsample inside preprocessCloud           (preprocess_voxel_leaf: 0.0
                                                              in the sim params, i.e. disabled)

Floating-point note: the C++ code does point/cell math in `float` (32-bit);
this port uses numpy float64 throughout. A point sitting exactly on a cell
boundary could in rare cases land in a different cell than the C++ code due
to float32 vs float64 rounding. Not expected to matter at the cell sizes used
here (>= 0.02 m), noted per the porting brief.
"""
from __future__ import annotations

import argparse
import math
from collections import Counter

import numpy as np

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

# C++ struct default member values, keyed by the ROS parameter name each maps
# to in ouster_cluster_node.cpp's declare_parameter() calls (so --set /
# --sim-params keys match the launch-file names 1:1).
DEFAULT_PARAMS: dict = {
    # ElevationFilterParams
    "elev_filter_enable": False,
    "elev_min_deg": -30.0,
    "elev_max_deg": 15.0,
    # PreprocessParams (range/z crop only; voxel_leaf kept for reference, unused)
    "preprocess_max_range": 3.0,
    "preprocess_z_min": -1.0,
    "preprocess_z_max": 1.0,
    "preprocess_voxel_leaf": 0.02,
    # BevDensityParams
    "bev_density_cell_size": 0.05,
    "bev_density_min": 0.0,
    "bev_density_max": 0.0,
    "bev_density_z_bin": 0.05,
    "bev_density_min_pts_per_bin": 2,
    "bev_density_min_z_bins": 0,
    "bev_density_dual_grid": False,
    "bev_density_drop_lonely": False,
    # BevLineParams (cell_size wired from bev_density_cell_size, like the node does)
    "bev_line_min_cells": 3,
    "bev_line_max_width": 0.12,
    "bev_line_min_length": 0.05,
    "bev_line_max_length": 0.0,
    "bev_line_z_gap": 0.1,
    "bev_line_z_med_gap": 0.0,
    "bev_line_diag_flank_gate": False,
    "bev_line_touch_connect": False,
    "bev_line_touch_gap": 0.03,
    "bev_line_touch_xy_gap": 0.0,
    "bev_line_touch_min_matched": 2,
    "bev_line_touch_min_ratio": 0.5,
    "bev_line_top_tol": 0.06,
    "bev_line_top_step": 0.07,
    "bev_line_max_segments": 10,
    # BevPlateParams
    "bev_plate_width": 0.14,
    "bev_plate_height": 0.10,
    "bev_plate_tol_width": 0.05,
    "bev_plate_tol_height": 0.02,
    "bev_plate_thickness_max": 0.05,
}

# Sim operating params (gtbot_formation/launch/perception.launch.py,
# ouster_cluster node block) — overrides applied on top of DEFAULT_PARAMS via
# --sim-params.
SIM_PARAMS: dict = {
    "preprocess_max_range": 5.0,
    "preprocess_z_min": -0.06,
    "preprocess_z_max": 0.10,
    "preprocess_voxel_leaf": 0.0,
    "elev_filter_enable": True,
    "elev_min_deg": -30.0,
    "elev_max_deg": 15.0,
    "bev_density_cell_size": 0.02,
    "bev_density_min": 3.0,
    "bev_density_max": 0.0,
    "bev_density_z_bin": 0.05,
    "bev_density_min_pts_per_bin": 1,
    "bev_density_min_z_bins": 2,
    "bev_density_dual_grid": True,
    "bev_density_drop_lonely": True,
    "bev_line_min_cells": 5,
    "bev_line_max_width": 0.09,
    "bev_line_min_length": 0.05,
    "bev_line_max_length": 0.25,
    "bev_line_z_gap": 0.10,
    "bev_line_diag_flank_gate": True,
    "bev_line_touch_connect": False,
    "bev_line_touch_gap": 0.015,
    "bev_line_touch_xy_gap": 0.03,
    "bev_line_touch_min_matched": 4,
    "bev_line_touch_min_ratio": 0.6,
    "bev_line_z_med_gap": 0.04,
    "bev_line_top_tol": 0.10,
    "bev_line_top_step": 0.07,
    "bev_plate_width": 0.14,
    "bev_plate_height": 0.10,
    "bev_plate_tol_width": 0.05,
    "bev_plate_tol_height": 0.04,
    "bev_plate_thickness_max": 0.0,
    "bev_line_max_segments": 50,
}


def _bev_density_params(params: dict) -> dict:
    return {
        "cell_size": params["bev_density_cell_size"],
        "min_density": params["bev_density_min"],
        "max_density": params["bev_density_max"],
        "z_bin": params["bev_density_z_bin"],
        "min_pts_per_bin": params["bev_density_min_pts_per_bin"],
        "min_z_bins": params["bev_density_min_z_bins"],
        "dual_grid": params["bev_density_dual_grid"],
        "drop_lonely": params["bev_density_drop_lonely"],
    }


def _bev_line_params(params: dict) -> dict:
    return {
        # the node sets line_prm_.cell_size = bev_.cell_size before extraction
        "cell_size": params["bev_density_cell_size"],
        "min_cells": params["bev_line_min_cells"],
        "max_width": params["bev_line_max_width"],
        "min_length": params["bev_line_min_length"],
        "max_length": params["bev_line_max_length"],
        "z_gap": params["bev_line_z_gap"],
        "z_med_gap": params["bev_line_z_med_gap"],
        "diag_flank_gate": params["bev_line_diag_flank_gate"],
        "touch_connect": params["bev_line_touch_connect"],
        "touch_gap": params["bev_line_touch_gap"],
        "touch_xy_gap": params["bev_line_touch_xy_gap"],
        "touch_min_matched": params["bev_line_touch_min_matched"],
        "touch_min_ratio": params["bev_line_touch_min_ratio"],
        "top_tol": params["bev_line_top_tol"],
        "top_step": params["bev_line_top_step"],
        "max_segments": params["bev_line_max_segments"],
    }


def _bev_plate_params(params: dict) -> dict:
    return {
        "width": params["bev_plate_width"],
        "height": params["bev_plate_height"],
        "tol_width": params["bev_plate_tol_width"],
        "tol_height": params["bev_plate_tol_height"],
        "thickness_max": params["bev_plate_thickness_max"],
    }


# ---------------------------------------------------------------------------
# elevation_filter.hpp :: filterElevation
# ---------------------------------------------------------------------------

def elevation_cut(points: np.ndarray, elev_min_deg: float, elev_max_deg: float) -> np.ndarray:
    """Keep points whose ray elevation angle lies in [elev_min_deg, elev_max_deg].
    Compares z against r_xy * tan(bound), same as the C++ version (no atan2 per
    point). Caller decides whether to invoke this (mirrors ElevationFilterParams.enable)."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] == 0:
        return pts
    tan_min = math.tan(math.radians(elev_min_deg))
    tan_max = math.tan(math.radians(elev_max_deg))
    r_xy = np.hypot(pts[:, 0], pts[:, 1])
    mask = (pts[:, 2] >= r_xy * tan_min) & (pts[:, 2] <= r_xy * tan_max)
    return pts[mask]


# ---------------------------------------------------------------------------
# plane_detect.hpp :: preprocessCloud (range/z crop portion only)
# ---------------------------------------------------------------------------

def preprocess(points: np.ndarray, max_range: float, z_min: float, z_max: float) -> np.ndarray:
    """Range/height-crop `points`: drop points outside [z_min, z_max] or farther
    than max_range from the origin (range crop skipped when max_range <= 0).
    Voxel-grid downsample step from the C++ version is intentionally NOT ported
    here: the sim operating params run preprocess_voxel_leaf=0.0 (disabled)."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] == 0:
        return pts
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    if max_range > 0.0:
        r2 = max_range * max_range
        mask &= (pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2) <= r2
    return pts[mask]


# ---------------------------------------------------------------------------
# bev_density.hpp :: computeBevDensity
# ---------------------------------------------------------------------------

def compute_bev_density(points: np.ndarray, p: dict) -> np.ndarray:
    """Bin points onto an XY grid of p['cell_size']; each cell's density is the
    sum of r^2 (=x^2+y^2) over its points (count x squared range, distance-
    invariant, noise-robust). Returns surviving points as an (N,4) array of
    (x, y, z, intensity) where intensity = the cell's density; cells failing
    the density band, or (if min_z_bins>0) the z-bin verticality gate, are
    dropped. dual_grid runs a second grid shifted half a cell and OR-combines
    survivors; drop_lonely removes cells with no surviving 8-neighbour."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] == 0 or p["cell_size"] <= 0.0:
        return np.empty((0, 4), dtype=np.float64)

    inv = 1.0 / p["cell_size"]
    zinv = 1.0 / p["z_bin"] if p["z_bin"] > 0.0 else 0.0
    grids = 2 if p["dual_grid"] else 1
    off = (0.0, 0.5 * p["cell_size"])

    density = [dict(), dict()]    # (cx,cy) -> sum r^2
    zbin_pts = [dict(), dict()]   # (cx,cy,cz) -> point count
    occupied = [dict(), dict()]   # (cx,cy) -> count of occupied z-bins

    for g in range(grids):
        o = off[g]
        for x, y, z in pts[:, :3]:
            cx = int(math.floor((x + o) * inv))
            cy = int(math.floor((y + o) * inv))
            key = (cx, cy)
            density[g][key] = density[g].get(key, 0.0) + x * x + y * y
            if p["min_z_bins"] > 0:
                cz = int(math.floor(z * zinv))
                zkey = (cx, cy, cz)
                zbin_pts[g][zkey] = zbin_pts[g].get(zkey, 0) + 1
        if p["min_z_bins"] > 0:
            for (cx, cy, _cz), cnt in zbin_pts[g].items():
                if cnt >= p["min_pts_per_bin"]:
                    xykey = (cx, cy)
                    occupied[g][xykey] = occupied[g].get(xykey, 0) + 1

    keep = [set(), set()]
    for g in range(grids):
        for key, dens in density[g].items():
            if dens < p["min_density"]:
                continue
            if p["max_density"] > 0.0 and dens > p["max_density"]:
                continue
            if p["min_z_bins"] > 0 and occupied[g].get(key, 0) < p["min_z_bins"]:
                continue
            keep[g].add(key)
        if p["drop_lonely"]:
            connected = set()
            for (cx, cy) in keep[g]:
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        if (cx + dx, cy + dy) in keep[g]:
                            connected.add((cx, cy))
                            break
                    else:
                        continue
                    break
            keep[g] = connected

    out = []
    for x, y, z in pts[:, :3]:
        for g in range(grids):
            o = off[g]
            cx = int(math.floor((x + o) * inv))
            cy = int(math.floor((y + o) * inv))
            key = (cx, cy)
            if key not in keep[g]:
                continue
            out.append((x, y, z, density[g][key]))
            break
    return np.asarray(out, dtype=np.float64).reshape(-1, 4)


# ---------------------------------------------------------------------------
# bev_lines.hpp :: extractBevLineSegments / isPlateSegment
# ---------------------------------------------------------------------------

def _pca_axis(centres: np.ndarray):
    """2x2 covariance principal axis (analytic, matches the C++ 0.5*atan2 form)."""
    mx = float(centres[:, 0].mean())
    my = float(centres[:, 1].mean())
    dx = centres[:, 0] - mx
    dy = centres[:, 1] - my
    sxx = float(np.sum(dx * dx))
    sxy = float(np.sum(dx * dy))
    syy = float(np.sum(dy * dy))
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    return mx, my, math.cos(theta), math.sin(theta)


def extract_bev_line_segments(points: np.ndarray, p: dict, *, collect_stats: bool = False):
    """Collapse the (already band-passed) BEV density points to unique XY cells,
    group into 8-neighbour connected components (touch_connect majority-touch >
    z_med_gap > z_gap connectivity rule, diag_flank_gate corner guard), then
    keep components shaped like a line (PCA length/width + top-edge uniformity,
    split at top-height steps). Returns a list of segment dicts
    {x0,y0,x1,y1,cells,z_min,z_max,width}. With collect_stats=True also returns
    a stats dict: {"n_components": int, "reject": Counter(gate -> count)}."""
    pts = np.asarray(points, dtype=np.float64)
    segs: list = []
    stats = {"n_components": 0, "reject": Counter()}
    if pts.shape[0] == 0 or p["cell_size"] <= 0.0:
        return (segs, stats) if collect_stats else segs

    inv = 1.0 / p["cell_size"]
    need_pts = p["z_med_gap"] > 0.0 or p["touch_connect"]

    cell_idx: dict = {}          # (cx,cy) -> index
    cx_list: list = []
    cy_list: list = []
    zmin_list: list = []
    zmax_list: list = []
    cell_pts: list = []          # per-cell [(x,y,z), ...], z-sorted ascending (only if need_pts)

    for x, y, z in pts[:, :3]:
        cx = int(math.floor(x * inv))
        cy = int(math.floor(y * inv))
        key = (cx, cy)
        idx = cell_idx.get(key)
        if idx is None:
            idx = len(cx_list)
            cell_idx[key] = idx
            cx_list.append(cx)
            cy_list.append(cy)
            zmin_list.append(z)
            zmax_list.append(z)
            if need_pts:
                cell_pts.append([(x, y, z)])
        else:
            if z < zmin_list[idx]:
                zmin_list[idx] = z
            if z > zmax_list[idx]:
                zmax_list[idx] = z
            if need_pts:
                cell_pts[idx].append((x, y, z))

    n_cells = len(cx_list)
    if need_pts:
        for i in range(n_cells):
            cell_pts[i].sort(key=lambda t: t[2])
        zmed_list = [cp[len(cp) // 2][2] for cp in cell_pts]
    else:
        zmed_list = [0.0] * n_cells

    def majority_touch(ia: int, ib: int) -> bool:
        A = cell_pts[ia]
        B = cell_pts[ib]  # both z-sorted ascending
        gap = p["touch_gap"]
        xy_gap = p["touch_xy_gap"]
        check_xy = xy_gap > 0.0
        matched = 0
        lo = 0
        for ax, ay, az in A:
            while lo < len(B) and B[lo][2] < az - gap:
                lo += 1
            found = False
            j = lo
            while j < len(B) and B[j][2] <= az + gap:
                bx, by, _bz = B[j]
                if not check_xy or math.hypot(bx - ax, by - ay) <= xy_gap:
                    found = True
                    break
                j += 1
            if found:
                matched += 1
        return matched > p["touch_min_ratio"] * len(A) and matched >= max(p["touch_min_matched"], 0)

    def z_connected(ia: int, ib: int) -> bool:
        if p["touch_connect"]:
            # one passing direction suffices: an occluded (few-point) section
            # still connects to its fully visible continuation
            return majority_touch(ia, ib) or majority_touch(ib, ia)
        if p["z_med_gap"] > 0.0:
            return abs(zmed_list[ia] - zmed_list[ib]) <= p["z_med_gap"]
        gap = max(zmin_list[ia], zmin_list[ib]) - min(zmax_list[ia], zmax_list[ib])
        return gap <= p["z_gap"]

    # 8-neighbour connected components
    comp = [-1] * n_cells
    n_comp = 0
    for s in range(n_cells):
        if comp[s] >= 0:
            continue
        comp[s] = n_comp
        stack = [s]
        while stack:
            i = stack.pop()
            cxi, cyi = cx_list[i], cy_list[i]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    if dx != 0 and dy != 0 and p["diag_flank_gate"]:
                        # corner-touch guard: both flanking orthogonal cells
                        # must exist and pass the z check
                        f1 = cell_idx.get((cxi + dx, cyi))
                        f2 = cell_idx.get((cxi, cyi + dy))
                        if (f1 is None or not z_connected(i, f1) or
                                f2 is None or not z_connected(i, f2)):
                            continue
                    nb = cell_idx.get((cxi + dx, cyi + dy))
                    if nb is not None and comp[nb] < 0 and z_connected(i, nb):
                        comp[nb] = n_comp
                        stack.append(nb)
        n_comp += 1
    stats["n_components"] = n_comp

    groups: list = [[] for _ in range(n_comp)]
    for i in range(n_cells):
        groups[comp[i]].append(i)

    def centre(i: int):
        return (cx_list[i] + 0.5) * p["cell_size"], (cy_list[i] + 0.5) * p["cell_size"]

    def emit_group(g: list):
        if len(g) < p["min_cells"]:
            if collect_stats:
                stats["reject"]["min_cells"] += 1
            return
        if len(segs) >= p["max_segments"]:
            if collect_stats:
                stats["reject"]["max_segments"] += 1
            return
        centres = np.array([centre(i) for i in g])
        mx, my, ux, uy = _pca_axis(centres)
        dx = centres[:, 0] - mx
        dy = centres[:, 1] - my
        t = ux * dx + uy * dy   # along the line axis
        w = -uy * dx + ux * dy  # across it
        tmin, tmax = float(t.min()), float(t.max())
        wmin, wmax = float(w.min()), float(w.max())

        # robust z-band: median of the per-cell extremes (nth_element-equivalent)
        zlo = sorted(zmin_list[i] for i in g)
        zhi = sorted(zmax_list[i] for i in g)
        zmin = zlo[len(zlo) // 2]
        zmax = zhi[len(zhi) // 2]
        length = tmax - tmin
        width = wmax - wmin
        if width > p["max_width"]:
            if collect_stats:
                stats["reject"]["max_width"] += 1
            return
        if length < p["min_length"]:
            if collect_stats:
                stats["reject"]["min_length"] += 1
            return
        if p["max_length"] > 0.0 and length > p["max_length"]:
            if collect_stats:
                stats["reject"]["max_length"] += 1
            return
        if p["top_tol"] > 0.0:
            tops = sorted(zmax_list[i] for i in g)
            trim = 1 if len(tops) >= 5 else 0
            spread = tops[-1 - trim] - tops[trim]
            if spread > p["top_tol"]:
                if collect_stats:
                    stats["reject"]["top_tol"] += 1
                return

        segs.append({
            "x0": mx + ux * tmin, "y0": my + uy * tmin,
            "x1": mx + ux * tmax, "y1": my + uy * tmax,
            "cells": len(g), "z_min": zmin, "z_max": zmax, "width": width,
        })

    # per component: split at persistent steps in the (3-tap median smoothed)
    # top-height profile along the principal axis, then emit each piece
    for g in groups:
        if len(g) < p["min_cells"]:
            if collect_stats:
                stats["reject"]["min_cells"] += 1
            continue
        if p["top_step"] <= 0.0:
            emit_group(g)
            continue

        centres = np.array([centre(i) for i in g])
        mx, my, ux, uy = _pca_axis(centres)
        proj = ux * (centres[:, 0] - mx) + uy * (centres[:, 1] - my)
        order = sorted(zip(proj.tolist(), g))  # cells ordered along the axis
        n = len(order)
        tops = [zmax_list[idx] for _, idx in order]
        smooth = []
        for j in range(n):
            a = tops[j - 1] if j > 0 else tops[j]
            b = tops[j]
            c = tops[j + 1] if j + 1 < n else tops[j]
            smooth.append(max(min(a, b), min(max(a, b), c)))  # median of 3
        run: list = []
        for j in range(n):
            if j > 0 and abs(smooth[j] - smooth[j - 1]) > p["top_step"]:
                emit_group(run)
                run = []
            run.append(order[j][1])
        emit_group(run)

    return (segs, stats) if collect_stats else segs


def is_plate_segment(seg: dict, p: dict) -> bool:
    """True if a segment's length and z-band height match the target plate size
    (within their own tolerances), and its perpendicular width is thin enough
    (thickness_max <= 0 disables that check)."""
    if p["thickness_max"] > 0.0 and seg["width"] > p["thickness_max"]:
        return False
    length = math.hypot(seg["x1"] - seg["x0"], seg["y1"] - seg["y0"])
    height = seg["z_max"] - seg["z_min"]
    return (abs(length - p["width"]) <= p["tol_width"] and
            abs(height - p["height"]) <= p["tol_height"])


# ---------------------------------------------------------------------------
# Pipeline runner + CLI
# ---------------------------------------------------------------------------

def run_pipeline(points: np.ndarray, params: dict, *, collect_stats: bool = False) -> dict:
    """Run elevation_cut -> preprocess -> compute_bev_density ->
    extract_bev_line_segments -> is_plate_segment, in the order onCloud() wires
    them (minus the disabled refine/reflectivity/temporal stages)."""
    pts = np.asarray(points, dtype=np.float64)
    n_raw = pts.shape[0]

    elev = pts
    if params["elev_filter_enable"]:
        elev = elevation_cut(pts, params["elev_min_deg"], params["elev_max_deg"])
    n_elev = elev.shape[0]

    prep = preprocess(elev, params["preprocess_max_range"], params["preprocess_z_min"],
                       params["preprocess_z_max"])
    n_prep = prep.shape[0]

    density = compute_bev_density(prep, _bev_density_params(params))
    # n_cells here matches the C++ node's own timing-log field name ("cells="),
    # which is actually density->size() — the density SURVIVOR POINT count, not
    # a deduped unique-cell count (dedup to unique cells happens inside
    # extract_bev_line_segments, feeding n_components below).
    n_cells = density.shape[0]

    segs, stats = extract_bev_line_segments(density, _bev_line_params(params), collect_stats=True)
    n_components = stats["n_components"]

    plate_p = _bev_plate_params(params)
    plates = [s for s in segs if is_plate_segment(s, plate_p)]

    result = {
        "n_raw": n_raw, "n_elev": n_elev, "n_prep": n_prep,
        "n_cells": n_cells, "n_components": n_components,
        "n_segments": len(segs), "n_plates": len(plates),
        "segments": segs, "plates": plates,
    }
    if collect_stats:
        result["reject"] = stats["reject"]
    return result


def load_frames(npz_path: str) -> list:
    data = np.load(npz_path, allow_pickle=True)
    if "clouds" in data.files:
        return [np.asarray(c, dtype=np.float64) for c in data["clouds"]]
    frames = []
    i = 0
    while f"cloud_{i}" in data.files:
        frames.append(np.asarray(data[f"cloud_{i}"], dtype=np.float64))
        i += 1
    if not frames:
        raise ValueError(f"{npz_path}: no 'clouds' or 'cloud_N' keys found")
    return frames


def apply_overrides(params: dict, overrides: list) -> dict:
    for item in overrides:
        if "=" not in item:
            raise SystemExit(f"--set expects key=value, got: {item!r}")
        key, val = item.split("=", 1)
        if key not in params:
            raise SystemExit(f"unknown parameter: {key}")
        default = params[key]
        if isinstance(default, bool):
            params[key] = val.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(default, int):
            params[key] = int(val)
        elif isinstance(default, float):
            params[key] = float(val)
        else:
            params[key] = val
    return params


def print_stage_report(frames: list, params: dict) -> None:
    print(f"{'frame':>5} {'n_raw':>7} {'n_elev':>7} {'n_prep':>7} {'n_cells':>8} "
          f"{'n_comp':>7} {'n_seg':>6} {'n_plate':>8}")
    for fi, pts in enumerate(frames):
        r = run_pipeline(pts, params, collect_stats=True)
        print(f"{fi:>5} {r['n_raw']:>7} {r['n_elev']:>7} {r['n_prep']:>7} {r['n_cells']:>8} "
              f"{r['n_components']:>7} {r['n_segments']:>6} {r['n_plates']:>8}")
        for s in r["plates"]:
            length = math.hypot(s["x1"] - s["x0"], s["y1"] - s["y0"])
            cx = 0.5 * (s["x0"] + s["x1"])
            cy = 0.5 * (s["y0"] + s["y1"])
            print(f"    plate: center=({cx:.3f},{cy:.3f}) length={length:.3f} "
                  f"zband=[{s['z_min']:.3f},{s['z_max']:.3f}]")
        if r["reject"]:
            reasons = ", ".join(f"{k}={v}" for k, v in sorted(r["reject"].items()))
            print(f"    dropped components: {reasons}")


# ---------------------------------------------------------------------------
# Self-test: synthetic plate must be detected, a low hull-blob must not be
# ---------------------------------------------------------------------------

def _synth_plate(center=(1.5, 0.0), width=0.14, height=0.10, z_center=0.02,
                  h_step_deg=0.35, v_step_deg=0.71, noise=0.01, seed=0) -> np.ndarray:
    """Vertical plate face-on to the sensor, sampled on a horizontal/vertical
    angular grid (approximating ring-lidar sampling density) plus XYZ noise."""
    rng = np.random.default_rng(seed)
    cx, cy = center
    range_ = math.hypot(cx, cy)
    az_span = 2.0 * math.degrees(math.atan2(width / 2.0, range_))
    n_h = max(int(az_span / h_step_deg), 2)
    n_v = max(int(math.degrees(height / range_) / v_step_deg), 2)
    base_yaw = math.atan2(cy, cx)
    dux, duy = -math.sin(base_yaw), math.cos(base_yaw)  # in-plate axis, perp. to range dir
    us = np.linspace(-width / 2.0, width / 2.0, n_h)
    zs = np.linspace(z_center - height / 2.0, z_center + height / 2.0, n_v)
    uu, zz = np.meshgrid(us, zs)
    uu = uu.ravel()
    zz = zz.ravel()
    xs = cx + uu * dux
    ys = cy + uu * duy
    pts = np.stack([xs, ys, zz], axis=1)
    pts += rng.normal(0.0, noise, size=pts.shape)
    return pts


def _synth_hull_blob(center=(1.0, 0.0), extent=0.6, z=-0.02, n=400, noise=0.01, seed=1) -> np.ndarray:
    """Low, flat, wide scatter (hull-like) — should fail the density map's
    verticality gate (min_z_bins) rather than being mistaken for a plate."""
    rng = np.random.default_rng(seed)
    cx, cy = center
    xs = cx + rng.uniform(-extent / 2, extent / 2, n)
    ys = cy + rng.uniform(-extent / 2, extent / 2, n)
    zs = np.full(n, z) + rng.normal(0.0, noise, n)
    return np.stack([xs, ys, zs], axis=1)


def selftest() -> None:
    params = dict(DEFAULT_PARAMS)
    params.update(SIM_PARAMS)

    plate_pts = _synth_plate()
    r_plate = run_pipeline(plate_pts, params)
    assert r_plate["n_plates"] == 1, (
        f"expected 1 plate, got {r_plate['n_plates']} "
        f"(n_segments={r_plate['n_segments']}, n_cells={r_plate['n_cells']}, "
        f"n_components={r_plate['n_components']})"
    )

    hull_pts = _synth_hull_blob()
    r_hull = run_pipeline(hull_pts, params)
    assert r_hull["n_plates"] == 0, f"expected 0 plates for hull blob, got {r_hull['n_plates']}"

    print(f"selftest OK: plate frame -> n_plates={r_plate['n_plates']} "
          f"(n_segments={r_plate['n_segments']}, n_cells={r_plate['n_cells']}) | "
          f"hull-blob frame -> n_plates={r_hull['n_plates']} (n_cells={r_hull['n_cells']})")


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Replay the ouster_cluster BEV plate-detection pipeline over a captured npz cloud sequence.")
    ap.add_argument("capture", nargs="?", help="npz file with 'clouds' or 'cloud_0'..'cloud_N' keys (Nx3 arrays)")
    ap.add_argument("--stage-report", action="store_true", help="print per-frame stage survival table")
    ap.add_argument("--sim-params", action="store_true",
                     help="apply the sim operating parameter set (gtbot_formation perception.launch.py) on top of the C++ defaults")
    ap.add_argument("--set", action="append", default=[], metavar="key=value",
                     help="override one parameter (repeatable)")
    ap.add_argument("--selftest", action="store_true", help="run the synthetic self-check and exit")
    return ap


def main(argv=None) -> int:
    ap = build_argparser()
    args = ap.parse_args(argv)

    if args.selftest:
        selftest()
        return 0

    if not args.capture:
        ap.error("capture npz path required unless --selftest")

    params = dict(DEFAULT_PARAMS)
    if args.sim_params:
        params.update(SIM_PARAMS)
    apply_overrides(params, args.set)

    frames = load_frames(args.capture)
    if args.stage_report:
        print_stage_report(frames, params)
    else:
        for fi, pts in enumerate(frames):
            r = run_pipeline(pts, params)
            print(f"frame {fi}: n_segments={r['n_segments']} n_plates={r['n_plates']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
