#!/usr/bin/env python3
"""
generate_offset_waypoints_new.py

회전된 workpiece(local frame 기준) 내부에 가운데 십자가(cross)가 있고,
사용자가 원하는 영역이 "십자가 바깥의 4개 코너 영역"일 때
각 코너 영역별 waypoint를 생성한다.

코너 영역 정의 (local frame 기준)
--------------------------------
십자가 중심을 (u_center, v_center), 폭을 (cross_width_u, cross_width_v)라 하면

  u_left   = u_center - cross_width_u / 2
  u_right  = u_center + cross_width_u / 2
  v_bottom = v_center - cross_width_v / 2
  v_top    = v_center + cross_width_v / 2

그러면 코너 4영역은:

  TOP_LEFT:
      u < u_left,   v > v_top

  TOP_RIGHT:
      u > u_right,  v > v_top

  BOTTOM_LEFT:
      u < u_left,   v < v_bottom

  BOTTOM_RIGHT:
      u > u_right,  v < v_bottom

내부와 십자가 영역은 제외하고
바깥 4개 코너에서만 waypoint를 생성한다.

핵심 가정
---------
1) workpiece는 local frame에서 잘 정렬된 회전 형상이다.
2) 십자가 중심은 기본적으로 local bounding box 중심과 일치한다.
3) 필요하면 center offset으로 미세 조정 가능하다.
"""

import argparse
import math
from pathlib import Path

import numpy as np
import trimesh


def axis_from_str(tool_axis: str) -> np.ndarray:
    tool_axis = tool_axis.strip().upper()
    sign = -1.0 if tool_axis.startswith("-") else 1.0

    if tool_axis.endswith("X"):
        v = np.array([1.0, 0.0, 0.0], dtype=float)
    elif tool_axis.endswith("Y"):
        v = np.array([0.0, 1.0, 0.0], dtype=float)
    elif tool_axis.endswith("Z"):
        v = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        raise ValueError(f"Invalid tool_axis: {tool_axis}")

    return sign * v


def make_uv_basis(axis: np.ndarray):
    axis = axis / np.linalg.norm(axis)

    tmp = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(np.dot(tmp, axis)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0], dtype=float)

    u = np.cross(axis, tmp)
    u = u / np.linalg.norm(u)

    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)
    return u, v


def uv_basis_from_xy_angle(tool_axis: np.ndarray, angle_deg: float):
    axis = tool_axis / np.linalg.norm(tool_axis)
    if abs(abs(axis[2]) - 1.0) > 1e-6:
        raise ValueError("manual_xy_angle mode currently assumes tool_axis = +Z or -Z")

    th = math.radians(angle_deg)
    u = np.array([math.cos(th), math.sin(th), 0.0], dtype=float)
    u = u - axis * np.dot(u, axis)
    u = u / np.linalg.norm(u)

    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)

    u = np.cross(v, axis)
    u = u / np.linalg.norm(u)
    return u, v


def uv_basis_from_mesh_pca(mesh: trimesh.Trimesh, tool_axis: np.ndarray):
    tool_axis = tool_axis / np.linalg.norm(tool_axis)
    V = mesh.vertices - mesh.vertices.mean(axis=0)
    C = V.T @ V
    eigvals, eigvecs = np.linalg.eigh(C)

    order = np.argsort(eigvals)
    a1 = eigvecs[:, order[-1]]
    a2 = eigvecs[:, order[-2]]

    def proj_orth(vec: np.ndarray):
        vec = vec - tool_axis * np.dot(vec, tool_axis)
        n = np.linalg.norm(vec)
        if n < 1e-9:
            return None
        return vec / n

    u = proj_orth(a1)
    if u is None:
        u = proj_orth(a2)
        if u is None:
            raise RuntimeError("Cannot build PCA UV basis.")

    v = np.cross(tool_axis, u)
    v = v / np.linalg.norm(v)

    u = np.cross(v, tool_axis)
    u = u / np.linalg.norm(u)
    return u, v


def project_to_uv(points_xyz: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    uu = points_xyz @ u
    vv = points_xyz @ v
    return np.stack([uu, vv], axis=1)


def uv_to_xyz(uv: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    return uv[:, 0:1] * u[None, :] + uv[:, 1:2] * v[None, :]


def batch_raycast_first_hit(mesh: trimesh.Trimesh, origins: np.ndarray, directions: np.ndarray):
    try:
        intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh)
    except Exception:
        intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)

    locations, index_ray, index_tri = intersector.intersects_location(
        origins, directions, multiple_hits=False
    )

    hit_mask = np.zeros(len(origins), dtype=bool)
    hit_points = np.zeros((len(origins), 3), dtype=float)
    hit_normals = np.zeros((len(origins), 3), dtype=float)

    hit_mask[index_ray] = True
    hit_points[index_ray] = locations
    hit_normals[index_ray] = mesh.face_normals[index_tri]
    return hit_mask, hit_points, hit_normals


def erosion_mask_by_radius(mask: np.ndarray, cell: float, radius: float) -> np.ndarray:
    try:
        from scipy.ndimage import distance_transform_edt
        dist_cells = distance_transform_edt(mask)
        dist_m = dist_cells * cell
        return dist_m >= radius
    except Exception:
        r_cells = int(math.ceil(radius / cell))
        yy, xx = np.mgrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
        disk = (xx ** 2 + yy ** 2) <= (radius / cell) ** 2

        eroded = mask.copy()
        H, W = mask.shape

        for i in range(H):
            i0 = max(0, i - r_cells)
            i1 = min(H, i + r_cells + 1)
            di0 = i0 - (i - r_cells)
            di1 = di0 + (i1 - i0)

            for j in range(W):
                j0 = max(0, j - r_cells)
                j1 = min(W, j + r_cells + 1)
                dj0 = j0 - (j - r_cells)
                dj1 = dj0 + (j1 - j0)

                k = disk[di0:di1, dj0:dj1]
                region = mask[i0:i1, j0:j1]
                if not np.all(region[k]):
                    eroded[i, j] = False
        return eroded


def centered_axis_values(minv: float, maxv: float, step: float) -> np.ndarray:
    length = maxv - minv
    if length < 0:
        return np.zeros((0,), dtype=float)

    n = int(np.floor(length / step)) + 1
    if n <= 0:
        return np.zeros((0,), dtype=float)

    remainder = length - (n - 1) * step
    start = minv + 0.5 * remainder
    return start + np.arange(n) * step


def serpentine_order(sample_uv: np.ndarray, step: float, scan_dir: str = "u", start_corner: str = "minmin") -> np.ndarray:
    if len(sample_uv) == 0:
        return np.array([], dtype=int)

    scan_dir = scan_dir.lower()
    if scan_dir not in ("u", "v"):
        raise ValueError("scan_dir must be 'u' or 'v'")

    if scan_dir == "u":
        col_axis = sample_uv[:, 0]
        row_axis = sample_uv[:, 1]
    else:
        col_axis = sample_uv[:, 1]
        row_axis = sample_uv[:, 0]

    col_id = np.round((col_axis - col_axis.min()) / step).astype(int)

    cols = {}
    for i, c in enumerate(col_id):
        cols.setdefault(c, []).append(i)

    unique_cols = sorted(cols.keys())
    scan_minmax = start_corner[:3]
    cross_minmax = start_corner[3:]

    if scan_minmax == "max":
        unique_cols = list(reversed(unique_cols))

    ordered = []
    for k, c in enumerate(unique_cols):
        idxs = cols[c]
        base_reverse = (cross_minmax == "max")
        reverse = base_reverse ^ (k % 2 == 1)
        idxs_sorted = sorted(idxs, key=lambda i: row_axis[i], reverse=reverse)
        ordered.extend(idxs_sorted)

    return np.array(ordered, dtype=int)


def path_cost_distance_and_normal(points: np.ndarray, normals: np.ndarray, w_dist: float = 1.0, w_nrm: float = 0.05) -> float:
    if len(points) < 2:
        return 0.0

    dp = points[1:] - points[:-1]
    dist = np.linalg.norm(dp, axis=1).sum()

    n0 = normals[:-1]
    n1 = normals[1:]
    dot = np.sum(n0 * n1, axis=1)
    dot = np.clip(dot, -1.0, 1.0)
    ang = np.arccos(dot).sum()

    return w_dist * dist + w_nrm * ang


def make_cross_corner_regions(uv_min, uv_max, u_left, u_right, v_bottom, v_top, split_margin, names):
    umin, vmin = uv_min
    umax, vmax = uv_max

    return [
        {
            "key": "TOP_LEFT",
            "name": names["TOP_LEFT"],
            "u_min": umin,
            "u_max": u_left - split_margin,
            "v_min": v_top + split_margin,
            "v_max": vmax,
        },
        {
            "key": "TOP_RIGHT",
            "name": names["TOP_RIGHT"],
            "u_min": u_right + split_margin,
            "u_max": umax,
            "v_min": v_top + split_margin,
            "v_max": vmax,
        },
        {
            "key": "BOTTOM_LEFT",
            "name": names["BOTTOM_LEFT"],
            "u_min": umin,
            "u_max": u_left - split_margin,
            "v_min": vmin,
            "v_max": v_bottom - split_margin,
        },
        {
            "key": "BOTTOM_RIGHT",
            "name": names["BOTTOM_RIGHT"],
            "u_min": u_right + split_margin,
            "u_max": umax,
            "v_min": vmin,
            "v_max": v_bottom - split_margin,
        },
    ]


def generate_waypoints_for_region(*, region, uv_min, uv_max, safe_mask, cell, step, tool_axis, u, v, mesh, origin_offset,
                                  standoff, scan_dir, start_corner, auto_w_dist, auto_w_nrm):
    W = safe_mask.shape[1]
    H = safe_mask.shape[0]

    ru_min = max(region["u_min"], uv_min[0])
    ru_max = min(region["u_max"], uv_max[0])
    rv_min = max(region["v_min"], uv_min[1])
    rv_max = min(region["v_max"], uv_max[1])

    sx = centered_axis_values(ru_min, ru_max, step)
    sy = centered_axis_values(rv_min, rv_max, step)

    if len(sx) == 0 or len(sy) == 0:
        return np.zeros((0, 2)), np.zeros((0, 3)), np.zeros((0, 3))

    sample_uv = []
    for v0 in sy:
        for u0 in sx:
            ix = int(round((u0 - uv_min[0]) / cell))
            iy = int(round((v0 - uv_min[1]) / cell))
            if 0 <= ix < W and 0 <= iy < H and safe_mask[iy, ix]:
                sample_uv.append([u0, v0])

    sample_uv = np.array(sample_uv, dtype=float)
    if len(sample_uv) == 0:
        return np.zeros((0, 2)), np.zeros((0, 3)), np.zeros((0, 3))

    origins = uv_to_xyz(sample_uv, u, v) + origin_offset[None, :]
    directions = np.tile(-tool_axis[None, :], (len(origins), 1))

    hit, pts_hit, nrm = batch_raycast_first_hit(mesh, origins, directions)

    sample_uv = sample_uv[hit]
    pts_hit = pts_hit[hit]
    nrm = nrm[hit]

    if len(pts_hit) == 0:
        return np.zeros((0, 2)), np.zeros((0, 3)), np.zeros((0, 3))

    nrm_unit = nrm / np.linalg.norm(nrm, axis=1, keepdims=True)

    scan_dir = scan_dir.lower()
    if scan_dir not in ("u", "v", "auto"):
        raise ValueError("--scan_dir must be one of: u, v, auto")

    if scan_dir == "auto":
        ord_u = serpentine_order(sample_uv, step=step, scan_dir="u", start_corner=start_corner)
        ord_v = serpentine_order(sample_uv, step=step, scan_dir="v", start_corner=start_corner)

        cost_u = path_cost_distance_and_normal(pts_hit[ord_u], nrm_unit[ord_u], auto_w_dist, auto_w_nrm)
        cost_v = path_cost_distance_and_normal(pts_hit[ord_v], nrm_unit[ord_v], auto_w_dist, auto_w_nrm)

        order = ord_u if cost_u <= cost_v else ord_v
        chosen = "u" if cost_u <= cost_v else "v"
        print(f"[{region['name']}] auto chose scan_dir={chosen} (cost_u={cost_u:.4f}, cost_v={cost_v:.4f})")
    else:
        order = serpentine_order(sample_uv, step=step, scan_dir=scan_dir, start_corner=start_corner)

    sample_uv = sample_uv[order]
    pts_hit = pts_hit[order]
    nrm_unit = nrm_unit[order]
    pts_standoff = pts_hit + nrm_unit * standoff

    return sample_uv, pts_hit, pts_standoff


def save_xyz(path, xyz, header_text):
    np.savetxt(path, xyz, header=header_text, comments="", fmt="%.6f")


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--mesh", required=True)
    ap.add_argument("--out_dir", type=str, default="cross_corner_waypoints")

    ap.add_argument("--pad_diameter", type=float, default=0.025)
    ap.add_argument("--overlap", type=float, default=0.15)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--tool_axis", type=str, default="+Z")
    ap.add_argument("--standoff", type=float, default=0.01)

    ap.add_argument("--scan_dir", type=str, default="auto")
    ap.add_argument("--start_corner", type=str, default="minmin")
    ap.add_argument("--grid_cell", type=float, default=0.01)
    ap.add_argument("--auto_w_dist", type=float, default=1.0)
    ap.add_argument("--auto_w_nrm", type=float, default=0.05)

    ap.add_argument("--frame_mode", type=str, default="manual_xy_angle", choices=["manual_xy_angle", "pca", "default"])
    ap.add_argument("--frame_angle_deg", type=float, default=45.0)
    ap.add_argument("--flip_u", action="store_true")
    ap.add_argument("--flip_v", action="store_true")

    ap.add_argument("--cross_width_u", type=float, default=0.05)
    ap.add_argument("--cross_width_v", type=float, default=0.05)
    ap.add_argument("--cross_center_offset_u", type=float, default=0.01)
    ap.add_argument("--cross_center_offset_v", type=float, default=0.01)
    ap.add_argument("--split_margin", type=float, default=0.0)

    ap.add_argument("--name_tl", type=str, default="1")
    ap.add_argument("--name_tr", type=str, default="2")
    ap.add_argument("--name_bl", type=str, default="3")
    ap.add_argument("--name_br", type=str, default="4")

    ap.add_argument("--save_combined", action="store_true")
    ap.add_argument("--save_hits_no_standoff", action="store_true")

    args = ap.parse_args()

    pad_radius = 0.5 * args.pad_diameter
    step = args.step if args.step is not None else args.pad_diameter * (1.0 - args.overlap)

    if step <= 0:
        raise ValueError("step must be > 0")
    if pad_radius <= 0:
        raise ValueError("pad_diameter must be > 0")
    if args.cross_width_u <= 0 or args.cross_width_v <= 0:
        raise ValueError("cross_width_u and cross_width_v must be > 0")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    tool_axis = axis_from_str(args.tool_axis)
    tool_axis = tool_axis / np.linalg.norm(tool_axis)

    mesh = trimesh.load_mesh(args.mesh, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError("Loaded object is not a Trimesh")

    if args.frame_mode == "manual_xy_angle":
        u, v = uv_basis_from_xy_angle(tool_axis, args.frame_angle_deg)
    elif args.frame_mode == "pca":
        u, v = uv_basis_from_mesh_pca(mesh, tool_axis)
    else:
        u, v = make_uv_basis(tool_axis)

    if args.flip_u:
        u = -u
    if args.flip_v:
        v = -v

    uv_verts = project_to_uv(mesh.vertices, u, v)
    uv_min = uv_verts.min(axis=0)
    uv_max = uv_verts.max(axis=0)

    cell = args.grid_cell
    xs = np.arange(uv_min[0], uv_max[0] + 1e-12, cell)
    ys = np.arange(uv_min[1], uv_max[1] + 1e-12, cell)

    W = len(xs)
    H = len(ys)

    xx, yy = np.meshgrid(xs, ys)
    uv_grid = np.stack([xx.reshape(-1), yy.reshape(-1)], axis=1)

    bbox = mesh.bounds
    diag = np.linalg.norm(bbox[1] - bbox[0])
    origin_offset = tool_axis * (diag * 2.0)

    origins = uv_to_xyz(uv_grid, u, v) + origin_offset[None, :]
    directions = np.tile(-tool_axis[None, :], (len(origins), 1))

    hit_mask, _, _ = batch_raycast_first_hit(mesh, origins, directions)
    valid_mask = hit_mask.reshape(H, W)
    safe_mask = erosion_mask_by_radius(valid_mask, cell=cell, radius=pad_radius)

    u_center = 0.5 * (uv_min[0] + uv_max[0]) + args.cross_center_offset_u
    v_center = 0.5 * (uv_min[1] + uv_max[1]) + args.cross_center_offset_v

    half_cross_u = 0.5 * args.cross_width_u
    half_cross_v = 0.5 * args.cross_width_v

    u_left = u_center - half_cross_u
    u_right = u_center + half_cross_u
    v_bottom = v_center - half_cross_v
    v_top = v_center + half_cross_v

    names = {
        "TOP_LEFT": args.name_tl,
        "TOP_RIGHT": args.name_tr,
        "BOTTOM_LEFT": args.name_bl,
        "BOTTOM_RIGHT": args.name_br,
    }

    regions = make_cross_corner_regions(
        uv_min=uv_min,
        uv_max=uv_max,
        u_left=u_left,
        u_right=u_right,
        v_bottom=v_bottom,
        v_top=v_top,
        split_margin=args.split_margin,
        names=names,
    )

    info_path = out_dir / "local_frame_info.txt"
    with open(info_path, "w", encoding="utf-8") as f:
        f.write(f"mesh = {args.mesh}\n")
        f.write(f"frame_mode = {args.frame_mode}\n")
        f.write(f"frame_angle_deg = {args.frame_angle_deg:.6f}\n")
        f.write(f"flip_u = {args.flip_u}\n")
        f.write(f"flip_v = {args.flip_v}\n")
        f.write(f"u = {u.tolist()}\n")
        f.write(f"v = {v.tolist()}\n")
        f.write(f"uv_min = {uv_min.tolist()}\n")
        f.write(f"uv_max = {uv_max.tolist()}\n")
        f.write(f"u_center = {u_center:.6f}\n")
        f.write(f"v_center = {v_center:.6f}\n")
        f.write(f"cross_width_u = {args.cross_width_u:.6f}\n")
        f.write(f"cross_width_v = {args.cross_width_v:.6f}\n")
        f.write(f"cross_center_offset_u = {args.cross_center_offset_u:.6f}\n")
        f.write(f"cross_center_offset_v = {args.cross_center_offset_v:.6f}\n")
        f.write(f"u_left = {u_left:.6f}\n")
        f.write(f"u_right = {u_right:.6f}\n")
        f.write(f"v_bottom = {v_bottom:.6f}\n")
        f.write(f"v_top = {v_top:.6f}\n")
        f.write(f"split_margin = {args.split_margin:.6f}\n")
        f.write(f"pad_radius = {pad_radius:.6f}\n")
        f.write(f"step = {step:.6f}\n")

    all_xyz = []
    report_lines = ["region_key,region_name,num_waypoints,output_file"]

    for region in regions:
        _, pts_hit, pts_standoff = generate_waypoints_for_region(
            region=region,
            uv_min=uv_min,
            uv_max=uv_max,
            safe_mask=safe_mask,
            cell=cell,
            step=step,
            tool_axis=tool_axis,
            u=u,
            v=v,
            mesh=mesh,
            origin_offset=origin_offset,
            standoff=args.standoff,
            scan_dir=args.scan_dir,
            start_corner=args.start_corner,
            auto_w_dist=args.auto_w_dist,
            auto_w_nrm=args.auto_w_nrm,
        )

        out_path = out_dir / f"{region['name']}.txt"
        header = (
            f"# local cross-corner waypoint xyz only\n"
            f"# region_key={region['key']}\n"
            f"# region_name={region['name']}\n"
            f"# mesh={args.mesh}\n"
            f"# frame_mode={args.frame_mode}\n"
            f"# frame_angle_deg={args.frame_angle_deg:.6f}\n"
            f"# region_u_min={region['u_min']:.6f}, region_u_max={region['u_max']:.6f}\n"
            f"# region_v_min={region['v_min']:.6f}, region_v_max={region['v_max']:.6f}\n"
            f"# columns: x y z\n"
        )
        save_xyz(out_path, pts_standoff, header)

        if args.save_hits_no_standoff:
            raw_path = out_dir / f"{region['name']}_hit.txt"
            raw_header = header.replace("# columns: x y z\n", "# columns: x y z (surface hit, no standoff)\n")
            save_xyz(raw_path, pts_hit, raw_header)

        print(f"[OK] Region '{region['name']}' -> {len(pts_standoff)} waypoints saved to {out_path}")
        report_lines.append(f"{region['key']},{region['name']},{len(pts_standoff)},{out_path}")

        if len(pts_standoff) > 0:
            all_xyz.append(pts_standoff)

    if args.save_combined and len(all_xyz) > 0:
        combined = np.vstack(all_xyz)
        combined_path = out_dir / "all_regions_combined.txt"
        header = (
            f"# combined local cross-corner waypoints\n"
            f"# mesh={args.mesh}\n"
            f"# frame_mode={args.frame_mode}\n"
            f"# frame_angle_deg={args.frame_angle_deg:.6f}\n"
            f"# columns: x y z\n"
        )
        save_xyz(combined_path, combined, header)
        print(f"[OK] Combined file saved to {combined_path} ({len(combined)} points)")

    report_path = out_dir / "region_report.csv"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report_lines) + "\n")

    print(f"[OK] Local frame info saved to {info_path}")
    print(f"[OK] Report saved to {report_path}")


if __name__ == "__main__":
    main()
