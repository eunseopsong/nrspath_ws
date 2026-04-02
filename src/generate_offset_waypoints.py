#!/usr/bin/env python3
"""
generate_offset_waypoints.py (patched + PCA uv align)

목표:
- STL mesh 표면에서 waypoints 생성
- mesh 경계로부터 pad_radius (= pad_diameter/2) 만큼 "항상" 안쪽 offset을 보장
- offset이 상하좌우 균일하도록 "centered grid" 사용
- 점 연결(출력 순서)이 시각적으로/물리적으로 합리적이도록 "serpentine raster" 정렬 강제
- 평면/곡면 모두 사용 가능 (ray casting 기반)
- 출력: x y z 텍스트


추가 기능:
- --uv_align pca 를 사용하면, 메쉬가 월드에서 회전돼 있어도
  메쉬 로컬 축(PCA) 기준으로 u,v를 잡아서 래스터가 "메쉬 축 정렬"되도록 함.
"""

import argparse
import math
import numpy as np
import trimesh


# ---------------------------
# 기본 유틸
# ---------------------------
def axis_from_str(tool_axis: str) -> np.ndarray:
    """'+Z', '-Z', '+X', ... 문자열을 단위벡터로 변환"""
    tool_axis = tool_axis.strip().upper()
    sign = 1.0
    if tool_axis.startswith('-'):
        sign = -1.0
    if tool_axis.endswith('X'):
        v = np.array([1.0, 0.0, 0.0])
    elif tool_axis.endswith('Y'):
        v = np.array([0.0, 1.0, 0.0])
    elif tool_axis.endswith('Z'):
        v = np.array([0.0, 0.0, 1.0])
    else:
        raise ValueError(f"Invalid tool_axis: {tool_axis}")
    return sign * v


def make_uv_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    axis에 수직인 (u,v) 직교기저 생성.
    axis: 3D unit vector
    """
    axis = axis / np.linalg.norm(axis)

    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, axis)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])

    u = np.cross(axis, tmp)
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)
    return u, v


def uv_basis_from_mesh_pca(mesh: trimesh.Trimesh, tool_axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    메쉬 정점들의 PCA(주성분)로 u,v를 잡되, tool_axis에 직교하도록 투영/정규화.
    - flat/준평면 메쉬에서 '메쉬 로컬 축 정렬' 효과가 큼.
    """
    tool_axis = tool_axis / np.linalg.norm(tool_axis)

    V = mesh.vertices - mesh.vertices.mean(axis=0)
    C = V.T @ V
    eigvals, eigvecs = np.linalg.eigh(C)  # ascending

    order = np.argsort(eigvals)
    a1 = eigvecs[:, order[-1]]  # 1st principal
    a2 = eigvecs[:, order[-2]]  # 2nd principal

    def proj_orth(vec):
        vec = vec - tool_axis * np.dot(vec, tool_axis)
        n = np.linalg.norm(vec)
        if n < 1e-9:
            return None
        return vec / n

    u = proj_orth(a1)
    if u is None:
        u = proj_orth(a2)
        if u is None:
            raise RuntimeError("Cannot build PCA UV basis (PCA axes parallel to tool_axis).")

    v = np.cross(tool_axis, u)
    v = v / np.linalg.norm(v)

    # 수치 오차 보정: u를 다시 직교화
    u = np.cross(v, tool_axis)
    u = u / np.linalg.norm(u)

    return u, v


def project_to_uv(points_xyz: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """3D 점들을 (u,v) 좌표로 투영 (원점 기준)"""
    uu = points_xyz @ u
    vv = points_xyz @ v
    return np.stack([uu, vv], axis=1)


def uv_to_xyz(uv: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """(u,v) 좌표를 3D 공간의 u,v 선형결합으로 올림 (원점 기준)"""
    return uv[:, 0:1] * u[None, :] + uv[:, 1:2] * v[None, :]


# ---------------------------
# Ray casting
# ---------------------------
def batch_raycast_first_hit(mesh: trimesh.Trimesh,
                            origins: np.ndarray,
                            directions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    다수 ray에 대해 첫 hit point / normal / hit mask 반환.
    """
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
    hit_normals[index_ray] = mesh.face_normals[index_tri]  # face normal

    return hit_mask, hit_points, hit_normals


# ---------------------------
# Erosion (안전영역)
# ---------------------------
def erosion_mask_by_radius(mask: np.ndarray, cell: float, radius: float) -> np.ndarray:
    """
    mask(True=유효) 영역을 radius 만큼 안쪽으로 erosion한 결과를 반환.
    - scipy가 있으면 distance transform으로 빠르게 처리
    - 없으면 원반 커널 기반 fallback(느림)
    """
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


# ---------------------------
# Centered grid (offset 균일)
# ---------------------------
def centered_axis_values(minv: float, maxv: float, step: float) -> np.ndarray:
    """양쪽 여백이 동일하도록 중앙 정렬된 샘플 좌표 배열 반환"""
    length = maxv - minv
    if length < 0:
        return np.zeros((0,), dtype=float)
    n = int(np.floor(length / step)) + 1
    if n <= 0:
        return np.zeros((0,), dtype=float)
    remainder = length - (n - 1) * step
    start = minv + 0.5 * remainder
    return start + np.arange(n) * step


# ---------------------------
# Serpentine ordering (합리적 연결)
# ---------------------------
def serpentine_order(sample_uv: np.ndarray,
                     step: float,
                     scan_dir: str = "u",
                     start_corner: str = "minmin") -> np.ndarray:
    """
    sample_uv: (N,2) in (u,v)
    step: grid spacing
    scan_dir:
      - 'u': u를 column id로 묶고 column 내에서 v를 왕복
      - 'v': v를 column id로 묶고 column 내에서 u를 왕복
    start_corner: minmin|minmax|maxmin|maxmax (scan,cross)
    """
    if len(sample_uv) == 0:
        return np.array([], dtype=int)

    scan_dir = scan_dir.lower()
    assert scan_dir in ("u", "v")

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


def path_cost_distance_and_normal(points: np.ndarray,
                                  normals: np.ndarray,
                                  w_dist: float = 1.0,
                                  w_nrm: float = 0.05) -> float:
    """scan_dir auto 선택용 비용 함수"""
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


# ---------------------------
# Main
# ---------------------------
def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--mesh", required=True, help="Path to STL/mesh file")
    ap.add_argument("--pad_diameter", type=float, default=0.10, help="Pad diameter [m] (default 0.10m = 10cm)")
    ap.add_argument("--overlap", type=float, default=0.15, help="Overlap ratio [0..1], step = D*(1-overlap)")
    ap.add_argument("--step", type=float, default=None, help="Waypoint spacing [m]. Overrides D*(1-overlap)")
    ap.add_argument("--tool_axis", type=str, default="+Z", help="Tool axis (+Z/-Z/+X/-X/+Y/-Y)")
    ap.add_argument("--standoff", type=float, default=0.01, help="Offset along surface normal [m]")

    ap.add_argument("--scan_dir", type=str, default="auto",
                    help="Raster scan direction: u | v | auto")
    ap.add_argument("--start_corner", type=str, default="minmin",
                    help="Start corner in (scan,cross): minmin|minmax|maxmin|maxmax")

    ap.add_argument("--grid_cell", type=float, default=0.01,
                    help="Cell size [m] for building valid-domain mask (smaller=more accurate, slower).")

    ap.add_argument("--auto_w_dist", type=float, default=1.0, help="auto scan: weight for distance")
    ap.add_argument("--auto_w_nrm", type=float, default=0.05, help="auto scan: weight for normal-change")

    ap.add_argument("--uv_align", type=str, default="world",
                    help="UV frame: world | pca  (pca aligns raster to mesh axes)")

    ap.add_argument("--out", type=str, default="waypoints.txt", help="Output txt path")
    args = ap.parse_args()

    pad_radius = 0.5 * args.pad_diameter
    step = args.step if args.step is not None else args.pad_diameter * (1.0 - args.overlap)

    if step <= 0:
        raise ValueError("step must be > 0")
    if pad_radius <= 0:
        raise ValueError("pad_diameter must be > 0")

    tool_axis = axis_from_str(args.tool_axis)
    tool_axis = tool_axis / np.linalg.norm(tool_axis)

    # mesh 로드
    mesh = trimesh.load_mesh(args.mesh, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError("Loaded object is not a Trimesh")


    # UV basis 선택
    uv_align = args.uv_align.lower()
    if uv_align == "pca":
        u, v = uv_basis_from_mesh_pca(mesh, tool_axis)
    else:
        u, v = make_uv_basis(tool_axis)

    # mesh vertices를 UV로 투영 -> UV bounding box
    uv_verts = project_to_uv(mesh.vertices, u, v)
    uv_min = uv_verts.min(axis=0)
    uv_max = uv_verts.max(axis=0)

    # 유효영역 마스크를 만들기 위한 coarse grid (cell)
    cell = args.grid_cell
    xs = np.arange(uv_min[0], uv_max[0] + 1e-12, cell)
    ys = np.arange(uv_min[1], uv_max[1] + 1e-12, cell)
    W = len(xs)
    H = len(ys)

    xx, yy = np.meshgrid(xs, ys)
    uv_grid = np.stack([xx.reshape(-1), yy.reshape(-1)], axis=1)

    # 레이캐스팅 origin 설정: tool_axis 방향으로 충분히 위로
    bbox = mesh.bounds
    diag = np.linalg.norm(bbox[1] - bbox[0])
    origin_offset = tool_axis * (diag * 2.0)

    origins = uv_to_xyz(uv_grid, u, v) + origin_offset[None, :]
    directions = np.tile(-tool_axis[None, :], (len(origins), 1))

    hit_mask, _, _ = batch_raycast_first_hit(mesh, origins, directions)
    valid_mask = hit_mask.reshape(H, W)

    # pad_radius만큼 erosion해서 "안전 영역"
    safe_mask = erosion_mask_by_radius(valid_mask, cell=cell, radius=pad_radius)

    # -----------------------------------------
    # Centered grid: offset 상하좌우 균일
    # -----------------------------------------
    uv_safe_min = uv_min + pad_radius
    uv_safe_max = uv_max - pad_radius

    sx = centered_axis_values(uv_safe_min[0], uv_safe_max[0], step)
    sy = centered_axis_values(uv_safe_min[1], uv_safe_max[1], step)

    if len(sx) == 0 or len(sy) == 0:
        raise RuntimeError(
            "Safe domain too small for the given pad_radius/step.\n"
            "Try smaller pad_diameter, smaller step, or check mesh size."
        )

    sample_uv = []
    for y0 in sy:
        for x0 in sx:
            ix = int(round((x0 - uv_min[0]) / cell))
            iy = int(round((y0 - uv_min[1]) / cell))
            if 0 <= ix < W and 0 <= iy < H and safe_mask[iy, ix]:
                sample_uv.append([x0, y0])

    sample_uv = np.array(sample_uv, dtype=float)
    if len(sample_uv) == 0:
        raise RuntimeError(
            "No valid samples after erosion.\n"
            "- pad_radius too large\n"
            "- grid_cell too coarse\n"
            "- ray casting can't hit the surface (tool_axis wrong)\n"
        )

    # 샘플을 다시 정확히 raycast하여 3D hit point/normal 획득
    origins2 = uv_to_xyz(sample_uv, u, v) + origin_offset[None, :]
    directions2 = np.tile(-tool_axis[None, :], (len(origins2), 1))
    hit2, pts2, nrm2 = batch_raycast_first_hit(mesh, origins2, directions2)

    sample_uv = sample_uv[hit2]
    pts2 = pts2[hit2]
    nrm2 = nrm2[hit2]
    if len(pts2) == 0:
        raise RuntimeError("All sample rays missed the surface. Check tool_axis or mesh orientation.")

    nrm2_unit = nrm2 / np.linalg.norm(nrm2, axis=1, keepdims=True)

    # -----------------------------------------
    # 합리적 연결: serpentine raster ordering
    # -----------------------------------------
    scan_dir = args.scan_dir.lower()
    if scan_dir not in ("u", "v", "auto"):
        raise ValueError("--scan_dir must be one of: u, v, auto")

    if scan_dir == "auto":
        ord_u = serpentine_order(sample_uv, step=step, scan_dir="u", start_corner=args.start_corner)
        ord_v = serpentine_order(sample_uv, step=step, scan_dir="v", start_corner=args.start_corner)

        cost_u = path_cost_distance_and_normal(
            pts2[ord_u], nrm2_unit[ord_u],
            w_dist=args.auto_w_dist, w_nrm=args.auto_w_nrm
        )
        cost_v = path_cost_distance_and_normal(
            pts2[ord_v], nrm2_unit[ord_v],
            w_dist=args.auto_w_dist, w_nrm=args.auto_w_nrm
        )

        order = ord_u if cost_u <= cost_v else ord_v
        chosen = "u" if cost_u <= cost_v else "v"
        print(f"[auto] chose scan_dir={chosen} (cost_u={cost_u:.4f}, cost_v={cost_v:.4f})")
    else:
        order = serpentine_order(sample_uv, step=step, scan_dir=scan_dir, start_corner=args.start_corner)

    sample_uv = sample_uv[order]
    pts2 = pts2[order]
    nrm2_unit = nrm2_unit[order]

    # -----------------------------------------
    # standoff 적용 (XYZ만 저장)
    # -----------------------------------------
    pts_standoff = pts2 + nrm2_unit * args.standoff
    out_xyz = pts_standoff.astype(float)  # (N,3)

    header = (
        f"# waypoints xyz only (centered-grid + serpentine)\n"
        f"# mesh: {args.mesh}\n"
        f"# pad_diameter={args.pad_diameter:.6f} m, pad_radius={pad_radius:.6f} m\n"
        f"# step={step:.6f} m (overlap={args.overlap:.3f}, override_step={args.step is not None})\n"
        f"# tool_axis={args.tool_axis}, standoff={args.standoff:.6f} m\n"
        f"# scan_dir={args.scan_dir}, start_corner={args.start_corner}\n"
        f"# grid_cell={args.grid_cell:.6f} m\n"
        f"# columns: x y z\n"
    )

    np.savetxt(args.out, out_xyz, header=header, comments="", fmt="%.6f")
    print(f"[OK] Saved {len(out_xyz)} waypoints (xyz) to: {args.out}")


if __name__ == "__main__":
    main()
