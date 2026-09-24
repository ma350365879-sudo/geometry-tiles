# -*- coding: utf-8 -*-
"""
几何拼接片 V3 · P0 基准版

V3 第一版：
  1. 生成用户定义的 2D 图形 P0（直线边 / 凹弧 / 凸弧）。
  2. P1 = offset(P0, -3mm)，P2 = offset(P0, -6mm)，仅作为 2D 参考线。
  3. 3D 主体 = P0 拉伸 3mm 的实心板。
  4. 每条边在 P0 边中点放 1 个卡扣，平整面贴 P0，方向取 P0 边外法线。
  5. 卡扣与主体做最小 0.01mm 重叠的布尔并，输出预览与 STL。
  6. P1 顶点：内角 <180° 做切线圆角（半径 P1_CORNER_R，含直线/凸弧/凹弧边），
     内角 <15° 或 >345° 跳过并红字提示，>=180° 不做处理。

不做：P1/P2 实体化、挖孔、包边、倒角、旋转楔形、装配分析。
"""
from __future__ import annotations

import io
import json
import math
import struct
import sys
import tempfile
import threading
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Tuple

import build123d as bd

BASE_DIR = Path(__file__).resolve().parent
STEP_FILE = BASE_DIR / "卡扣元件.step"
HTML_FILE = BASE_DIR / "tile_generator_v3.html"
VENDOR_DIR = BASE_DIR / "vendor"

THICKNESS = 3.0
INSET1 = 3.0
INSET2 = 6.0
CLIP_OVERLAP = 0.01
CORNER_R = 0.03
# P1 顶点圆角：内角 <180° 做切线圆角（半径 P1_CORNER_R）；<15° 或 >345° 跳过并提示用户。
P1_CORNER_R = 0.05
P1_CORNER_MIN_ANGLE = 15.0
P1_CORNER_MAX_ANGLE = 345.0
# 包边：卡扣端面扫掠截面与主体的重叠量。
SWEEP_CLIP_OVERLAP = 0.01
# 包边段两端与卡扣端面的留空量：只需断开共面接触即可。
# 0.02mm 落在 _plane_crossings 的容差带（0.02）内，凹弧楔形侧近共面时
# glue=False 并集 invalid、回退 glue=True 吞掉薄包边件；0.03mm 脱离容差带，
# 且仍在打印可见缝隙上限（0.05mm）之内。
WRAP_END_GAP = 0.03
# 凹角段A 在顶点前的提前停止量：断开与段B 端帽的共边/共面接触。
# （历史"延长段"已删除 2026-08-30：延长段与段B 同路径重叠导致逐件融合吞件，
# 凹角连续性由段A/段B 顶点外侧的空间交叉 + Compound 一次布尔保证。）
REFLEX_WRAP_OVERLAP = 2.0  # 已弃用：延长段方案随凹角拆段重构移除，保留备查
# 凹角段A 在顶点前的提前停止量：断开与延长段/段B 端帽的共边接触。
WRAP_VERTEX_GAP = 0.05
HOLE_CHAMFER = 0.75
HOLE_FILLET_R = 1.5
MESH_TOLERANCE = 0.02

MIN_SIDE_MM = 12.0
MAX_SIDE_MM = 160.0
MIN_ANGLE_DEG = 15.0
MAX_ANGLE_DEG = 300.0
EARLY_CLOSE_MM = 0.5

_clip_cache: Dict[str, object] = {}

# ------------------------------------------------------------
# 构建子进程：OCC/build123d 的布尔内核非线程安全，且重型构建可能长时间
# 占用 GIL（3D 构建期间服务器整体无响应）。把 2D/3D 构建放进独立常驻
# 子进程：主进程永远能响应页面请求；子进程卡死时由看门狗杀掉重启。
# ------------------------------------------------------------
BUILD_TIMEOUT_S = 60.0

_worker_state: Dict[str, dict] = {}
_rid_counter = [0]
_pending = {}          # rid -> {"event": Event, "result": None}
_pending_lock = threading.Lock()


def _worker_2d(q_in, q_out):
    """2D 构建子进程主循环（spawn 后重新导入本模块）。"""
    while True:
        rid, params = q_in.get()
        if rid is None:
            break
        try:
            result2d = get_p0_2d(params)
            q_out.put((rid, "ok", result2d))
        except Exception as exc:
            q_out.put((rid, "err", str(exc)))


def _stl_name_from_params(params, meta=None):
    """STL 文件名：前端翻译层传 stlName（中文完整参数式）优先；否则 ASCII 兜底。"""
    if isinstance(params, dict):
        raw = params.get("stlName")
        if isinstance(raw, str) and raw.strip():
            name = raw.strip().replace("/", "_").replace("\\", "_") \
                .replace('"', "'").replace("\n", " ").replace("\r", " ")
            if not name.lower().endswith(".stl"):
                name += ".stl"
            return name
    n = (meta or {}).get("sides", "?")
    c = (meta or {}).get("clipCount", "?")
    return f"tile_v3_p0_n{n}_clips{c}.stl"


def _worker_3d(q_in, q_out):
    """3D 构建子进程主循环。"""
    while True:
        rid, kind, params = q_in.get()
        if rid is None:
            break
        try:
            shape, result = get_p0_result(params)
            meta = result["meta"]
            name = _stl_name_from_params(params, meta)
            if kind == "stl":
                data, _stats = shape_to_stl_bytes(shape)
                q_out.put((rid, "stl", {"meta": meta, "data": data, "name": name}))
            else:
                result["name"] = name
                q_out.put((rid, "ok", result))
        except Exception as exc:
            q_out.put((rid, "err", str(exc)))


def _dispatcher(q_out):
    """把子进程响应路由给等待中的请求线程。"""
    while True:
        try:
            rid, status, payload = q_out.get()
        except Exception:
            continue
        with _pending_lock:
            rec = _pending.get(rid)
        if rec is not None:
            rec["result"] = (status, payload)
            rec["event"].set()


def _spawn_worker(kind):
    import multiprocessing as mp
    ctx = mp.get_context("spawn")
    st = _worker_state[kind]
    if kind == "2d":
        proc = ctx.Process(target=_worker_2d, args=(st["q_in"], st["q_out"]), daemon=True)
    else:
        proc = ctx.Process(target=_worker_3d, args=(st["q_in"], st["q_out"]), daemon=True)
    proc.start()
    st["proc"] = proc
    return proc


def _restart_worker(kind):
    """杀掉卡死的构建子进程并重启（由看门狗路径调用）。"""
    st = _worker_state[kind]
    with st["lock"]:
        proc = st["proc"]
        try:
            if proc is not None and proc.is_alive():
                proc.kill()
                proc.join(timeout=3)
        except Exception:
            pass
        _spawn_worker(kind)
        st["gen"] += 1
def _start_build_workers():
    import multiprocessing as mp
    if _worker_state:
        return
    for kind, worker in (("2d", _worker_2d), ("3d", _worker_3d)):
        st = {
            "q_in": mp.Queue(),
            "q_out": mp.Queue(),
            "proc": None,
            "gen": 0,
            "lock": threading.Lock(),
        }
        _worker_state[kind] = st
        _spawn_worker(kind)
        threading.Thread(target=_dispatcher, args=(st["q_out"],), daemon=True).start()


def _submit_build(kind, payload):
    """向构建子进程提交任务并等待结果；超时则重启子进程并报错。"""
    _start_build_workers()
    st = _worker_state[kind]
    with _pending_lock:
        _rid_counter[0] += 1
        rid = _rid_counter[0]
        rec = {"event": threading.Event(), "result": None}
        _pending[rid] = rec
    gen = st["gen"]
    st["q_in"].put((rid,) + payload)
    if not rec["event"].wait(BUILD_TIMEOUT_S):
        with _pending_lock:
            _pending.pop(rid, None)
        with st["lock"]:
            if st["gen"] == gen:
                try:
                    if st["proc"] is not None and st["proc"].is_alive():
                        st["proc"].kill()
                        st["proc"].join(timeout=3)
                except Exception:
                    pass
                _spawn_worker(kind)
                st["gen"] += 1
        raise TimeoutError(f"{kind} 构建超时（> {BUILD_TIMEOUT_S:g}s），已重启生成进程，请重试")
    with _pending_lock:
        _pending.pop(rid, None)
    status, payload2 = rec["result"]
    if status == "err":
        raise RuntimeError(payload2)
    return status, payload2


# ------------------------------------------------------------
# 卡扣元件：平整面 + 两个扫掠端面形成的局部坐标系
# ------------------------------------------------------------
def load_clip():
    """读取 STEP，标准化到卡扣安装坐标系。

    +x = 平整面外法线（外伸方向）
    y  = 两个扫掠端面之间的长轴方向
    z  = 3mm 厚度方向
    平整面位于 x=0，y/z 中心位于 0。
    """
    if "clip" in _clip_cache:
        return _clip_cache["clip"]
    clip = bd.import_step(str(STEP_FILE))
    solid = clip.solids()[0]

    flat_faces = [f for f in solid.faces() if f.geom_type == bd.GeomType.PLANE]
    if not flat_faces:
        raise RuntimeError("卡扣 STEP 中找不到平面")
    flat = max(flat_faces, key=lambda f: f.area)
    flat_center = flat.center()

    # 面积最大的平面 = 平整面。
    # 局部 +x 取“从平整面指向实体内部”的方向，这样变换后实体位于 x>=0，
    # 平整面位于 x=0，与旧版卡扣母版坐标一致。
    outward = flat.normal_at(flat_center)
    if outward.length < 1e-12:
        raise RuntimeError("无法确定卡扣平整面法线")
    x_dir = -outward

    # 平整面的最长边是长轴 y，与之垂直的是厚度 z。
    flat_edges = sorted(flat.edges(), key=lambda e: e.length, reverse=True)
    y_dir = (flat_edges[0].end_point() - flat_edges[0].start_point()).normalized()
    z_dir = x_dir.cross(y_dir).normalized()

    plane = bd.Plane(flat_center, x_dir=x_dir, z_dir=z_dir)
    local = solid.moved(plane.location.inverse())
    bb = local.bounding_box()
    flat_after = max([f for f in local.faces() if f.geom_type == bd.GeomType.PLANE],
                     key=lambda f: f.area)
    local = local.translate((
        -flat_after.center().X,
        -(bb.min.Y + bb.max.Y) / 2.0,
        -(bb.min.Z + bb.max.Z) / 2.0,
    ))
    _clip_cache["clip"] = local
    return local


def load_sweep_section(overlap: float = 0.0):
    """提取卡扣端面轮廓作为旋转楔形截面。

    卡扣端面位于 canonical 坐标 y=±5；把截面平移到 y=0。
    """
    key = f"section:{overlap:g}"
    if key in _clip_cache:
        return _clip_cache[key]
    clip = load_clip()
    sec_compound = clip.intersect(bd.Plane.XZ.offset(5))
    faces = list(sec_compound.faces())
    if not faces:
        raise RuntimeError("无法从卡扣 STEP 提取端面扫掠截面")
    sec = faces[0]
    sec = sec.translate((0, 5, 0)).translate((-overlap, 0, 0))
    _clip_cache[key] = sec
    return sec


def _clip_with_rotated_end_faces(pe, direction: str):
    """给弧边卡扣的两个扫掠端面做旋转楔形，使端面与 P0 弧垂直。

    - 凸弧绕 e_in（截面 x=0）旋转；
    - 凹弧绕 e_out（截面外棱 x=W）旋转；
    - 卡扣安装位置/锚点不移动，仍由调用方放在 P0 弧中点。
    """
    clip = load_clip()
    sweep_face = load_sweep_section(0.0)
    W = round(float(sweep_face.bounding_box().max.X), 6)
    axis_x = 0.0 if direction == "out" else W
    L = float(pe.length)
    half = min(5.0, max(0.5, L / 2.0 - 1e-6))
    pmid = pe.position_at(L / 2.0, bd.PositionMode.LENGTH)
    tmid = pe.tangent_at(L / 2.0, bd.PositionMode.LENGTH)
    outward = bd.Vector(tmid.Y, -tmid.X, 0)

    def _rotated_wedge(phi: float):
        phi_deg = math.degrees(phi)
        z_dir = (0, 0, 1) if phi >= 0 else (0, 0, -1)
        axis = bd.Axis((axis_x, 0.0, 0.0), z_dir)
        wedge = bd.revolve([sweep_face], axis=axis, revolution_arc=abs(phi_deg))
        if not wedge.is_valid():
            raise ValueError("扫掠面旋转实体生成失败")
        return wedge

    shape = clip
    for sgn in (1.0, -1.0):
        arc_d = L / 2.0 + half * sgn
        tan = pe.tangent_at(arc_d, bd.PositionMode.LENGTH)
        tx = tan.X * outward.X + tan.Y * outward.Y
        ty = tan.X * tmid.X + tan.Y * tmid.Y
        phi = math.atan2(-tx, ty)
        wedge = _rotated_wedge(phi)
        if (wedge.center().Y >= 0) != (sgn > 0):
            wedge = wedge.mirror(bd.Plane.XZ)
        if sgn > 0:
            wedge = wedge.translate((0, half, 0))
        else:
            wedge = wedge.translate((0, -half, 0))
        shape = shape.fuse(wedge)
    return shape


# ------------------------------------------------------------
# 2D 多边形求解（V3 独立实现，不依赖旧版）
# ------------------------------------------------------------
def _wrap_rad(x: float) -> float:
    x = x % (2.0 * math.pi)
    if x > math.pi:
        x -= 2.0 * math.pi
    if x < -math.pi:
        x += 2.0 * math.pi
    return x


def _cross2(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, p, eps: float = 1e-9) -> bool:
    return (
        min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
        and abs(_cross2(a, b, p)) <= eps
    )


def _segments_intersect(a, b, c, d) -> bool:
    o1 = _cross2(a, b, c)
    o2 = _cross2(a, b, d)
    o3 = _cross2(c, d, a)
    o4 = _cross2(c, d, b)
    if ((o1 > 1e-9 and o2 < -1e-9) or (o1 < -1e-9 and o2 > 1e-9)) and \
       ((o3 > 1e-9 and o4 < -1e-9) or (o3 < -1e-9 and o4 > 1e-9)):
        return True
    if abs(o1) <= 1e-9 and _on_segment(a, b, c):
        return True
    if abs(o2) <= 1e-9 and _on_segment(a, b, d):
        return True
    if abs(o3) <= 1e-9 and _on_segment(c, d, a):
        return True
    if abs(o4) <= 1e-9 and _on_segment(c, d, b):
        return True
    return False


def polygon_is_simple(verts: List[bd.Vector]) -> bool:
    n = len(verts)
    if n < 3:
        return False
    for i in range(n):
        a = (verts[i].X, verts[i].Y)
        b = (verts[(i + 1) % n].X, verts[(i + 1) % n].Y)
        for j in range(i + 1, n):
            if j == i or j == (i + 1) % n or (j + 1) % n == i:
                continue
            c = (verts[j].X, verts[j].Y)
            d = (verts[(j + 1) % n].X, verts[(j + 1) % n].Y)
            if _segments_intersect(a, b, c, d):
                return False
    return True


def polygon_signed_area(verts: List[bd.Vector]) -> float:
    s = 0.0
    n = len(verts)
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        s += a.X * b.Y - b.X * a.Y
    return s / 2.0


def regular_polygon(sides: int, side_len: float) -> List[bd.Vector]:
    if sides < 3:
        raise ValueError("sides must be >= 3")
    radius = side_len / (2.0 * math.sin(math.pi / sides))
    return [
        bd.Vector(radius * math.cos(math.pi / 2.0 + 2.0 * math.pi * i / sides),
                  radius * math.sin(math.pi / 2.0 + 2.0 * math.pi * i / sides),
                  0.0)
        for i in range(sides)
    ]


def triangle_from_lengths(lengths: List[float]) -> List[bd.Vector]:
    a, b, c = float(lengths[0]), float(lengths[1]), float(lengths[2])
    if min(a, b, c) <= 0 or a >= b + c or b >= a + c or c >= a + b:
        raise ValueError("三边长无法构成三角形")
    x = (a * a + c * c - b * b) / (2.0 * a)
    y = math.sqrt(max(0.0, c * c - x * x))
    verts = [bd.Vector(0, 0, 0), bd.Vector(a, 0, 0), bd.Vector(x, y, 0)]
    centroid = sum(verts, bd.Vector(0, 0, 0)) * (1.0 / 3.0)
    return [v - centroid for v in verts]


def make_edges(verts: List[bd.Vector]):
    edges = []
    n = len(verts)
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        t = (b - a).normalized()
        outward = bd.Vector(t.Y, -t.X, 0)
        edges.append((a, b, t, outward))
    return edges


def solve_free_polygon_solutions(lengths: List[float], angles_deg: List[float]):
    n = len(lengths)
    if n < 3 or n > 12:
        raise ValueError("自由多边形边数需为 3-12")
    if len(angles_deg) != max(0, n - 3):
        raise ValueError(f"自由{n}边形需要 {max(0, n - 3)} 个可调内角")
    lengths = [float(v) for v in lengths]
    angles_deg = [float(v) for v in angles_deg]
    for v in lengths:
        if not math.isfinite(v) or v < MIN_SIDE_MM or v > MAX_SIDE_MM:
            raise ValueError(f"边长 {v:g} 超出 {MIN_SIDE_MM:g}–{MAX_SIDE_MM:g} mm")
    for v in angles_deg:
        if not math.isfinite(v) or v < MIN_ANGLE_DEG or v > MAX_ANGLE_DEG:
            raise ValueError(f"角度 {v:g}° 超出 {MIN_ANGLE_DEG:g}°–{MAX_ANGLE_DEG:g}°")

    k = n - 3
    dirs = [0.0]
    for a in angles_deg:
        dirs.append(dirs[-1] + math.radians(180.0 - a))

    pts = [bd.Vector(0, 0, 0)]
    x = y = 0.0
    for i in range(k + 1):
        x += lengths[i] * math.cos(dirs[i])
        y += lengths[i] * math.sin(dirs[i])
        pts.append(bd.Vector(x, y, 0))

    p0, pend = pts[0], pts[-1]
    d = (pend - p0).length
    la, lb = lengths[-2], lengths[-1]
    if d < 1e-9:
        raise ValueError("前 n-2 条边已回到起点，图形过早闭合")
    if d < abs(la - lb) - 1e-9 or d > la + lb + 1e-9:
        raise ValueError("最后两条边无法闭合")

    dx, dy = pend.X - p0.X, pend.Y - p0.Y
    base_dir = math.atan2(dy, dx)
    xc = (d * d + la * la - lb * lb) / (2.0 * d)
    h = math.sqrt(max(0.0, la * la - xc * xc))
    cxs = [pend.X - xc * math.cos(base_dir) + h * math.sin(base_dir),
           pend.X - xc * math.cos(base_dir) - h * math.sin(base_dir)]
    cys = [pend.Y - xc * math.sin(base_dir) - h * math.cos(base_dir),
           pend.Y - xc * math.sin(base_dir) + h * math.cos(base_dir)]

    def angles_for(verts):
        out = []
        for i in range(n):
            prev_v, cur_v, next_v = verts[(i - 1) % n], verts[i], verts[(i + 1) % n]
            turn = _wrap_rad(math.atan2(next_v.Y - cur_v.Y, next_v.X - cur_v.X)
                             - math.atan2(cur_v.Y - prev_v.Y, cur_v.X - prev_v.X))
            angle = 180.0 - math.degrees(turn)
            if angle > 360.0:
                angle -= 360.0
            if angle < 0.0:
                angle += 360.0
            out.append(angle)
        return out

    candidates = []
    for cx, cy in zip(cxs, cys):
        verts = list(pts) + [bd.Vector(cx, cy, 0)]
        signed_area = polygon_signed_area(verts)
        if signed_area < 0:
            verts = [bd.Vector(v.X, -v.Y, 0) for v in verts]
            signed_area = -signed_area
        if not polygon_is_simple(verts):
            continue
        angles = angles_for(verts)
        if any(a < MIN_ANGLE_DEG - 0.01 or a > MAX_ANGLE_DEG + 0.01 for a in angles):
            continue
        if any(abs(angles[i + 1] - a) > 0.05 for i, a in enumerate(angles_deg)):
            continue
        candidates.append((signed_area, verts, angles))
    if not candidates:
        raise ValueError("无法构造满足边长与角度的简单多边形")

    candidates.sort(key=lambda c: (max(c[2]), -c[0]))
    solutions = []
    for _area, verts, actual_angles in candidates:
        early = False
        for i in range(2, n - 1):
            if (verts[i] - verts[0]).length < EARLY_CLOSE_MM:
                early = True
                break
        if early:
            continue
        actual_lengths = [float((verts[(i + 1) % n] - verts[i]).length) for i in range(n)]
        if any(v < MIN_SIDE_MM - 0.01 or v > MAX_SIDE_MM + 0.01 for v in actual_lengths):
            continue
        centroid = sum(verts, bd.Vector(0, 0, 0)) * (1.0 / n)
        shifted = [v - centroid for v in verts]
        solutions.append((shifted, float(actual_angles[-1]), float(actual_lengths[-1]), actual_angles))
    if not solutions:
        raise ValueError("无法构造满足边长与角度的简单多边形")
    return solutions


def solve_free_polygon(lengths, angles_deg, solution=0):
    solutions = solve_free_polygon_solutions(lengths, angles_deg)
    if solution < 0 or solution >= len(solutions):
        raise ValueError(f"闭合解索引 {solution} 不可用")
    return solutions[solution][0], solutions[solution][1], solutions[solution][2]


def _arc_midpoint(a: bd.Vector, b: bd.Vector, radius: float, direction: str) -> bd.Vector:
    L = float((b - a).length)
    if radius < L / 2.0 - 1e-6:
        raise ValueError(f"弧半径 {radius:g} mm 小于弦长一半 {L/2:.2f} mm")
    h = math.sqrt(max(0.0, radius * radius - L * L / 4.0))
    sag = radius - h
    tx, ty = (b.X - a.X) / L, (b.Y - a.Y) / L
    nx, ny = ty, -tx
    sign = -1.0 if direction == "in" else 1.0
    return bd.Vector((a.X + b.X) / 2.0 + sign * nx * sag,
                     (a.Y + b.Y) / 2.0 + sign * ny * sag, 0.0)


# ------------------------------------------------------------
# P0 实心主体 + 每边 P0 中点卡扣
# ------------------------------------------------------------
def _edge_to_polyline(edge, arc_samples: int = 24):
    pts = []
    if edge.geom_type == bd.GeomType.LINE:
        pts = [edge.start_point(), edge.end_point()]
    else:
        L = float(edge.length)
        for k in range(arc_samples + 1):
            pts.append(edge.position_at(L * k / arc_samples, bd.PositionMode.LENGTH))
    return [[round(p.X, 4), round(p.Y, 4)] for p in pts]


def _sketch_to_loops(sketch, arc_samples: int = 24):
    loops = []
    faces = list(sketch.faces()) if hasattr(sketch, "faces") else [sketch]
    for face in faces:
        for wire in face.wires():
            loop = []
            for edge in wire.edges():
                poly = _edge_to_polyline(edge, arc_samples)
                if loop and len(poly) > 1:
                    poly = poly[1:]
                loop.extend(poly)
            if len(loop) >= 3:
                loops.append(loop)
    return loops


def _outward_at(edge_obj, dist: float, face_center: bd.Vector) -> bd.Vector:
    tan = edge_obj.tangent_at(dist, bd.PositionMode.LENGTH)
    outward = bd.Vector(tan.Y, -tan.X, 0)
    pt = edge_obj.position_at(dist, bd.PositionMode.LENGTH)
    to_center = bd.Vector(face_center.X - pt.X, face_center.Y - pt.Y, 0)
    if outward.X * to_center.X + outward.Y * to_center.Y > 0:
        outward = bd.Vector(-outward.X, -outward.Y, 0)
    return outward


def _offset_face(p0_face, amount: float):
    """向内 offset，若有多个区域则取面积最大的面。"""
    sketch = bd.offset(p0_face, amount=amount, kind=bd.Kind.INTERSECTION)
    faces = list(sketch.faces()) if hasattr(sketch, "faces") else [sketch]
    if not faces:
        raise RuntimeError("No offset generated")
    return max(faces, key=lambda f: f.area)


def _fillet_vertical_edges(shape, radius: float):
    """对 3mm 厚板的竖直侧棱做圆角；批量失败时逐条重试。"""
    edges = []
    for e in shape.edges():
        sp = e.start_point()
        ep = e.end_point()
        if (
            abs(sp.Z - ep.Z) > THICKNESS * 0.5
            and abs(sp.X - ep.X) < 1e-9
            and abs(sp.Y - ep.Y) < 1e-9
        ):
            edges.append(e)
    if not edges:
        return shape
    try:
        return shape.fillet(radius, edges)
    except Exception:
        pass
    for edge in edges:
        try:
            candidate = shape.fillet(radius, [edge])
            if candidate.is_valid():
                shape = candidate
        except Exception:
            continue
    return shape


def _apply_inner_hole_chamfer_fillet(shape, p2_wire):
    """镂空内孔：上下内棱 0.75 倒角，倒角后新棱 R1.5 圆角。

    返回 (shape, report)。失败步骤记录但不阻断生成。
    """
    report = {"chamferFailed": [], "filletFailed": []}

    def near_p2(point, tol=0.03):
        return min(e.distance_to(bd.Vector(point.X, point.Y, 0)) for e in p2_wire.edges()) < tol

    def edge_sample(e):
        try:
            return e @ 0.5
        except Exception:
            return (e.start_point() + e.end_point()) * 0.5

    top_edges = [
        e for e in shape.edges()
        if e.start_point().Z > THICKNESS - 0.001 and e.end_point().Z > THICKNESS - 0.001
        and near_p2(edge_sample(e))
    ]
    if top_edges:
        try:
            shape = shape.chamfer(HOLE_CHAMFER, None, top_edges)
        except Exception:
            report["chamferFailed"].append("top")

    top_chamfer_edges = [
        e for e in shape.edges()
        if abs(e.start_point().Z - (THICKNESS - HOLE_CHAMFER)) < 0.01
        and abs(e.end_point().Z - (THICKNESS - HOLE_CHAMFER)) < 0.01
        and near_p2(edge_sample(e), tol=0.08)
    ]
    if top_chamfer_edges:
        try:
            shape = shape.fillet(HOLE_FILLET_R, top_chamfer_edges)
        except Exception:
            report["filletFailed"].append("top")

    bottom_edges = [
        e for e in shape.edges()
        if e.start_point().Z < 0.001 and e.end_point().Z < 0.001
        and near_p2(edge_sample(e))
    ]
    if bottom_edges:
        try:
            shape = shape.chamfer(HOLE_CHAMFER, None, bottom_edges)
        except Exception:
            report["chamferFailed"].append("bottom")

    bottom_chamfer_edges = [
        e for e in shape.edges()
        if abs(e.start_point().Z - HOLE_CHAMFER) < 0.01
        and abs(e.end_point().Z - HOLE_CHAMFER) < 0.01
        and near_p2(edge_sample(e), tol=0.08)
    ]
    if bottom_chamfer_edges:
        try:
            shape = shape.fillet(HOLE_FILLET_R, bottom_chamfer_edges)
        except Exception:
            report["filletFailed"].append("bottom")

    return shape, report


def _ordered_wire_edges(wire):
    edges = list(wire.edges())
    if len(edges) <= 1:
        return edges
    ordered = [edges[0]]
    rest = edges[1:]
    tol = 1e-4
    while rest:
        end = ordered[-1].end_point()
        match_idx = None
        for i, edge in enumerate(rest):
            if (edge.start_point() - end).length < tol:
                match_idx = i
                break
        if match_idx is None:
            for i, edge in enumerate(rest):
                if (edge.end_point() - end).length < tol:
                    rest[i] = edge.reversed()
                    match_idx = i
                    break
        if match_idx is None:
            ordered.extend(rest)
            break
        ordered.append(rest.pop(match_idx))
    return ordered


def _wire_to_loop(wire, arc_samples: int = 24):
    loop = []
    for edge in _ordered_wire_edges(wire):
        poly = _edge_to_polyline(edge, arc_samples)
        if loop and len(poly) > 1:
            poly = poly[1:]
        loop.extend(poly)
    return loop


def _vertex_interior_angle(t_in: bd.Vector, t_out: bd.Vector) -> float:
    turn = math.atan2(t_in.X * t_out.Y - t_in.Y * t_out.X,
                      t_in.X * t_out.X + t_in.Y * t_out.Y)
    angle = math.pi - turn
    if angle < 0.0:
        angle += 2.0 * math.pi
    if angle >= 2.0 * math.pi:
        angle -= 2.0 * math.pi
    return math.degrees(angle)


def adaptive_corner_radius(angle_deg: float) -> float:
    d90 = 2.0 * (1.0 / math.sin(math.radians(45.0)) - 1.0)
    theta = math.radians(angle_deg)
    sin_half = math.sin(theta / 2.0)
    if sin_half >= 1.0:
        return 0.30
    radius = d90 * sin_half / (1.0 - sin_half)
    return max(0.30, radius)


def _trim_edge(edge, s0: float, s1: float):
    if s1 - s0 < 1e-6:
        return None
    a = edge.position_at(s0, bd.PositionMode.LENGTH)
    b = edge.position_at(s1, bd.PositionMode.LENGTH)
    if edge.geom_type == bd.GeomType.LINE:
        return bd.Edge.make_line(a, b)
    mid = edge.position_at((s0 + s1) / 2.0, bd.PositionMode.LENGTH)
    return bd.Edge.make_three_point_arc(a, mid, b)


def _fillet_wire_at_vertex(wire, vertex, radius: float):
    edges = _ordered_wire_edges(wire)
    idx = None
    vertex_pos = bd.Vector(vertex.X, vertex.Y, vertex.Z)
    for i, edge in enumerate(edges):
        if (edge.end_point() - vertex_pos).length < 1e-6:
            idx = i
            break
    if idx is None:
        raise RuntimeError("找不到圆角顶点")
    prev_edge = edges[idx]
    next_edge = edges[(idx + 1) % len(edges)]
    lp = float(prev_edge.length)
    ln = float(next_edge.length)
    t_in = prev_edge.tangent_at(lp, bd.PositionMode.LENGTH)
    t_out = next_edge.tangent_at(0.0, bd.PositionMode.LENGTH)
    turn = math.atan2(t_in.X * t_out.Y - t_in.Y * t_out.X,
                      t_in.X * t_out.X + t_in.Y * t_out.Y)
    if abs(turn) < math.radians(0.5):
        return wire
    d = radius * math.tan(abs(turn) / 2.0)
    if d <= 1e-6 or d >= lp - 1e-6 or d >= ln - 1e-6:
        raise RuntimeError("圆角半径相对邻边过大")
    pin = prev_edge.position_at(lp - d, bd.PositionMode.LENGTH)
    pout = next_edge.position_at(d, bd.PositionMode.LENGTH)
    arc = bd.Edge.make_tangent_arc(
        bd.Vector(pin.X, pin.Y, 0),
        bd.Vector(t_in.X, t_in.Y, 0),
        bd.Vector(pout.X, pout.Y, 0),
    )
    left = _trim_edge(prev_edge, 0.0, lp - d)
    right = _trim_edge(next_edge, d, ln)
    new_edges = []
    for i, edge in enumerate(edges):
        if i == idx:
            if left is not None:
                new_edges.append(left)
            new_edges.append(arc)
        elif i == (idx + 1) % len(edges):
            if right is not None:
                new_edges.append(right)
        else:
            new_edges.append(edge)
    return bd.Wire.make_wire(new_edges)


def _rounded_p2_face(p2_face):
    if not hasattr(p2_face, "outer_wire"):
        return p2_face, p2_face.wire(), []
    wire = p2_face.outer_wire()
    edges = list(wire.edges())
    failed = []
    if len(edges) < 3:
        return p2_face, wire, failed

    targets = []
    for i, edge in enumerate(edges):
        prev_edge = edges[(i - 1) % len(edges)]
        a = edge.start_point()
        t_in = prev_edge.tangent_at(float(prev_edge.length), bd.PositionMode.LENGTH)
        t_out = edge.tangent_at(0.0, bd.PositionMode.LENGTH)
        turn = math.atan2(t_in.X * t_out.Y - t_in.Y * t_out.X,
                          t_in.X * t_out.X + t_in.Y * t_out.Y)
        if abs(turn) < math.radians(0.5):
            continue
        angle_deg = _vertex_interior_angle(t_in, t_out)
        cap = max(0.30, min(float(prev_edge.length), float(edge.length)) / 4.0)
        radius = min(adaptive_corner_radius(angle_deg), cap)
        targets.append((a, radius, angle_deg))

    for point, radius, angle_deg in targets:
        vertex = min(
            wire.vertices(),
            key=lambda v: (bd.Vector(v.X, v.Y, 0) - bd.Vector(point.X, point.Y, 0)).length,
        )
        try:
            wire = _fillet_wire_at_vertex(wire, vertex, radius)
        except Exception:
            failed.append({
                "point": [round(point.X, 3), round(point.Y, 3)],
                "angle": round(angle_deg, 2),
                "radius": round(radius, 3),
            })

    try:
        face = bd.Face.make_from_wires(wire)
        if face.is_valid() and face.area > 1e-6:
            return face, wire, failed
    except Exception:
        pass
    return p2_face, wire, failed


def _p1_vertex_angles(p1_face):
    """计算 P1 外边每个顶点的内角信息。

    返回 (有序边列表, infos)。infos[k] 对应顶点 k（边 k-1 与边 k 的连接处）：
    angle=内角(度)，smooth=转向<0.5°（共线/相切过渡，视作 180°）。
    用面法向校正环绕方向，角度与绕向无关。
    """
    wire = p1_face.outer_wire()
    edges = _ordered_wire_edges(wire)
    try:
        nz = p1_face.normal_at(p1_face.center()).Z
    except Exception:
        nz = 1.0
    if abs(nz) < 1e-9:
        nz = 1.0
    orient = 1.0 if nz > 0 else -1.0
    infos = []
    for i, edge in enumerate(edges):
        prev_edge = edges[(i - 1) % len(edges)]
        t_in = prev_edge.tangent_at(float(prev_edge.length), bd.PositionMode.LENGTH)
        t_out = edge.tangent_at(0.0, bd.PositionMode.LENGTH)
        turn = math.atan2(t_in.X * t_out.Y - t_in.Y * t_out.X,
                          t_in.X * t_out.X + t_in.Y * t_out.Y)
        smooth = abs(turn) < math.radians(0.5)
        angle_deg = 180.0 if smooth else (math.degrees(math.pi - turn * orient) % 360.0)
        infos.append({
            "index": i,
            "point": edge.start_point(),
            "turn": turn,
            "angle": angle_deg,
            "smooth": smooth,
        })
    return edges, infos


def _rounded_p1_wire(p1_face):
    """P1 顶点圆角：内角 <180° 做切线圆角（半径 P1_CORNER_R）；>=180° 保持尖角。

    内角 <15° 或 >345° 的顶点跳过处理（过于尖锐，圆角无法稳定放置），
    记录红字提示。直线-弧线、弧线-弧线的顶点用切线角计算，同样适用。
    返回 (face, wire, applied_count, warnings)。
    """
    wire = p1_face.outer_wire()
    edges, infos = _p1_vertex_angles(p1_face)
    if len(edges) < 3:
        return p1_face, wire, 0, []

    targets = []
    warnings = []
    for info in infos:
        i = info["index"]
        if info["smooth"]:
            continue  # 近直线/相切连接，无角可圆
        turn = info["turn"]
        angle_deg = info["angle"]
        label = f"边{(i - 1) % len(edges)}→边{i} 顶点内角 {angle_deg:.1f}°"
        if angle_deg < P1_CORNER_MIN_ANGLE or angle_deg > P1_CORNER_MAX_ANGLE:
            warnings.append(
                "当前图形某内角小于15°，暂不支持"
                if angle_deg < P1_CORNER_MIN_ANGLE
                else "当前图形某内角大于345°，暂不支持"
            )
            continue
        if angle_deg >= 180.0:
            continue  # 凹角：不处理
        lp = float(edges[(i - 1) % len(edges)].length)
        ln = float(edges[i].length)
        tan_half = math.tan(abs(turn) / 2.0)
        radius = P1_CORNER_R
        if tan_half > 1e-9:
            fit = min(lp, ln) - 0.05
            if fit <= 1e-6:
                warnings.append(f"{label}，邻边过短，{P1_CORNER_R:g}mm 圆角无法放置")
                continue
            radius = min(radius, fit / tan_half)
        if radius < 0.02:
            warnings.append(f"{label}，邻边过短，{P1_CORNER_R:g}mm 圆角无法放置")
            continue
        targets.append((info["point"], radius, label))

    applied = 0
    for point, radius, label in targets:
        vertex = min(
            wire.vertices(),
            key=lambda v: (bd.Vector(v.X, v.Y, 0) - bd.Vector(point.X, point.Y, 0)).length,
        )
        try:
            wire = _fillet_wire_at_vertex(wire, vertex, radius)
            applied += 1
        except Exception:
            warnings.append(f"{label}，圆角生成失败，保持尖角")

    warnings = list(dict.fromkeys(warnings))  # 同类提示去重

    try:
        face = bd.Face.make_from_wires(wire)
        if face.is_valid() and face.area > 1e-6:
            return face, wire, applied, warnings
    except Exception:
        pass
    warnings.append("P1 圆角后无法重建面，保持原始 P1 外形")
    return p1_face, p1_face.wire(), 0, warnings


def _facet_circular(parts, chord: float = 1.0):
    """圆弧路径段拆成短弦直线段。

    弧扫掠实体（frenet 管道）在部分组合下会让 OCC 布尔并静默丢弃实体；
    折线化后全部是直线扫掠，布尔稳定。弦长 ≤1mm 时弦高差 ≈0.002mm，不可见。
    """
    out = []
    for part in parts:
        if part.geom_type == bd.GeomType.CIRCLE:
            L = float(part.length)
            steps = max(2, int(math.ceil(L / chord)))
            for j in range(steps):
                s0 = L * j / steps
                s1 = L * (j + 1) / steps
                a = part.position_at(s0, bd.PositionMode.LENGTH)
                b = part.position_at(s1, bd.PositionMode.LENGTH)
                out.append(bd.Edge.make_line(a, b))
        else:
            out.append(part)
    return out


def _sweep_wrap_segment(section, parts, orient: float, start_pt: bd.Vector):
    """沿包边路径段扫掠截面：先整段 frenet 扫掠（凸角圆弧连续无缝），
    失败或法线翻转（实体铺到板内侧）时回退逐段扫掠。所有结果都做
    "外侧探针"校验，确保包边在外侧。

    orient 为 P1 外边环绕向（法向 Z>0 为 +1，否则 -1）；外侧法线统一按
    绕向取切线右法线×orient，与顶点角度计算一致（不能按"远离面心"判定，
    凹多边形靠近凹角的边会因此把包边铺到板内侧）。
    返回有效实体列表；全部失败返回 []。
    """
    def _radial_outward(tangent):
        return bd.Vector(tangent.Y * orient, -tangent.X * orient, 0)

    def _probe_outward(solid, path_parts):
        """校验实体在每个路径段中点外侧 1.5mm 处存在材料。"""
        for part in path_parts:
            s = float(part.length) / 2.0
            pt = part.position_at(s, bd.PositionMode.LENGTH)
            tt = part.tangent_at(s, bd.PositionMode.LENGTH)
            radial = _radial_outward(tt)
            probe = bd.Vector(pt.X, pt.Y, 0) + radial * 1.5
            box = bd.Solid.make_box(0.2, 0.2, 0.2).translate(
                (probe.X - 0.1, probe.Y - 0.1, THICKNESS / 2.0 - 0.1))
            try:
                if solid.intersect(box).volume < 1e-6:
                    return False
            except Exception:
                return False
        return True

    def _try(path_list, spt, tang):
        radial = _radial_outward(tang)
        plane = bd.Plane(bd.Vector(spt.X, spt.Y, THICKNESS / 2.0),
                         x_dir=radial, z_dir=(0, 0, 1))
        w = bd.Wire.make_wire([e.translate((0, 0, THICKNESS / 2.0)) for e in path_list])
        return bd.Solid.sweep(section.moved(plane.location), w, is_frenet=True)

    start_t = parts[0].tangent_at(0.0, bd.PositionMode.LENGTH)
    try:
        s = _try(parts, start_pt, start_t)
        if s.is_valid() and s.volume > 1e-6 and _probe_outward(s, parts):
            return [s]
    except Exception:
        pass
    return _per_part_sweeps(section, parts, orient)


def _per_part_sweeps(section, parts, orient: float):
    """逐段独立扫掠（每段用自己的外侧法线），返回有效实体列表。"""
    def _radial_outward(tangent):
        return bd.Vector(tangent.Y * orient, -tangent.X * orient, 0)

    def _probe_outward(solid, path_parts):
        for part in path_parts:
            s = float(part.length) / 2.0
            pt = part.position_at(s, bd.PositionMode.LENGTH)
            tt = part.tangent_at(s, bd.PositionMode.LENGTH)
            radial = _radial_outward(tt)
            probe = bd.Vector(pt.X, pt.Y, 0) + radial * 1.5
            box = bd.Solid.make_box(0.2, 0.2, 0.2).translate(
                (probe.X - 0.1, probe.Y - 0.1, THICKNESS / 2.0 - 0.1))
            try:
                if solid.intersect(box).volume < 1e-6:
                    return False
            except Exception:
                return False
        return True

    sols = []
    for part in parts:
        # 外侧方向按原始 part 的方向计算（与反序无关）。
        base_t = part.tangent_at(0.0, bd.PositionMode.LENGTH)
        radial = _radial_outward(base_t)
        for path in (part, part.reversed()):
            try:
                st = path.start_point()
                # 起点平面必须与整段扫掠一致地放在板中面 Z=1.5，
                # 否则回退包边会整体下沉 1.5mm。
                sp = bd.Plane(bd.Vector(st.X, st.Y, THICKNESS / 2.0),
                              x_dir=radial, z_dir=(0, 0, 1))
                one = bd.Solid.sweep(section.moved(sp.location),
                                     path.translate((0, 0, THICKNESS / 2.0)),
                                     is_frenet=True)
                if one.is_valid() and one.volume > 1e-6 and _probe_outward(one, [part]):
                    sols.append(one)
                    break
            except Exception:
                continue
    return sols


def _sweep_wrap_segment_faceted(section, parts, orient: float):
    """折线化 + 纯逐段扫掠。

    frenet 弧扫掠实体在部分组合下会被 OCC 布尔并静默丢弃（直线-弧线凹角
    的两段互斥）；折线化（弦长≤1mm，弦高差≈0.002mm）后全部是直线扫掠，
    布尔稳定。仅用于整段扫掠结果融合失败时的重试。
    """
    return _per_part_sweeps(section, _facet_circular(parts), orient)


def _arc_p1_midpoint(edge_obj, radius: float, direction: str) -> bd.Vector:
    """P0 弧边对应的 P1 弧中点。"""
    a = edge_obj.start_point()
    b = edge_obj.end_point()
    L = float((b - a).length)
    t = (b - a).normalized()
    n = bd.Vector(t.Y, -t.X, 0)
    sign = 1.0 if direction == "out" else -1.0
    h = math.sqrt(max(0.0, radius * radius - (L / 2.0) ** 2))
    R1 = radius - INSET1 * sign
    M = (a + b) * 0.5
    return M + n * sign * (R1 - h)


def _arc_p1_chord_geometry(edge_obj, radius: float, direction: str):
    """旧版弧边卡扣定位：安装弦平面两端点落在 P1 圆上。

    返回 (弦中点 anchor, 用于端面旋转的 P1 弧 edge)。
    凹弧旋转轴为 e_out，需要按实际旋转角补偿 e_in 的偏移。
    """
    a = edge_obj.start_point()
    b = edge_obj.end_point()
    L = float((b - a).length)
    t = (b - a).normalized()
    n = bd.Vector(t.Y, -t.X, 0)
    sign = 1.0 if direction == "out" else -1.0
    h = math.sqrt(max(0.0, radius * radius - (L / 2.0) ** 2))
    radial = n * sign
    C = (a + b) * 0.5 - radial * h
    R1 = radius - INSET1 * sign

    base_y = math.sqrt(max(0.0, R1 * R1 - 25.0))
    base_anchor = C + radial * base_y
    rotate_edge = None
    if R1 > 5.01:
        base_left = base_anchor - t * 5.0
        base_right = base_anchor + t * 5.0
        p1_mid = C + radial * R1
        rotate_edge = bd.Edge.make_three_point_arc(base_left, p1_mid, base_right)

    anchor = base_anchor
    half_span = 5.0
    if direction == "in" and rotate_edge is not None:
        # _clip_with_rotated_end_faces 对凹弧绕 e_out(x=W) 旋转。
        # 旋转后新的 e_in 在局部坐标为 (dx, ±ly)，需反推弦平面位置。
        Lp = float(rotate_edge.length)
        tmid = rotate_edge.tangent_at(Lp / 2.0, bd.PositionMode.LENGTH)
        outward = bd.Vector(tmid.Y, -tmid.X, 0)
        tan = rotate_edge.tangent_at(Lp / 2.0 + 5.0, bd.PositionMode.LENGTH)
        tx = tan.X * outward.X + tan.Y * outward.Y
        ty = tan.X * tmid.X + tan.Y * tmid.Y
        phi = abs(math.atan2(-tx, ty))
        W = 3.0
        dx = W * (1.0 - math.cos(phi))
        ly = 5.0 + W * math.sin(phi)
        y1 = dx + math.sqrt(max(0.0, R1 * R1 - ly * ly))
        anchor = C + radial * y1
        half_span = ly
    return anchor, rotate_edge, half_span


def _match_p1_line_edge(p1_face, target: bd.Vector, outward: bd.Vector):
    """为直线边找 P1 上同向、投影距离最近的直线段。"""
    t_p0 = bd.Vector(-outward.Y, outward.X, 0)
    best = None
    best_dist = 1e18
    for edge in p1_face.outer_wire().edges():
        if edge.geom_type != bd.GeomType.LINE:
            continue
        a = edge.start_point()
        b = edge.end_point()
        tt = (b - a).normalized()
        if abs(tt.X * t_p0.X + tt.Y * t_p0.Y) < 1.0 - 1e-9:
            continue
        v = target - a
        proj = v.X * tt.X + v.Y * tt.Y
        dist = abs(v.X * tt.Y - v.Y * tt.X)
        if proj < -0.1 or proj > float((b - a).length) + 0.1:
            continue
        if dist < best_dist:
            best_dist = dist
            best = edge
    return best


def _closest_p1_edge(p1_face, target: bd.Vector):
    """在 P1 外边中找离 target 最近的边，用于弧边卡扣端面旋转。"""
    best = None
    best_d = 1e18
    for edge in p1_face.outer_wire().edges():
        mid = edge.position_at(float(edge.length) / 2.0, bd.PositionMode.LENGTH)
        d = (mid - target).length
        if d < best_d:
            best_d = d
            best = edge
    return best


def _closest_param_on_edge(edge, target: bd.Vector, samples: int = 240):
    """找 P1 边上离 target 最近的弧长参数。"""
    L = float(edge.length)
    best_s = 0.0
    best_d = 1e18
    for k in range(samples + 1):
        s = L * k / samples
        p = edge.position_at(s, bd.PositionMode.LENGTH)
        d = (p - target).length
        if d < best_d:
            best_d = d
            best_s = s
    return best_s


def _sub_edge(raw, s0: float, s1: float):
    """截取 P1 边界边的 [s0,s1] 弧长区间。"""
    if s1 - s0 < 1e-6:
        return None
    a = raw.position_at(s0, bd.PositionMode.LENGTH)
    b = raw.position_at(s1, bd.PositionMode.LENGTH)
    if raw.geom_type == bd.GeomType.LINE:
        return bd.Edge.make_line(a, b)
    mid = raw.position_at((s0 + s1) / 2.0, bd.PositionMode.LENGTH)
    return bd.Edge.make_three_point_arc(a, mid, b)


def _plane_crossings(wire_edges, center: bd.Vector, normal: bd.Vector, samples: int = 300):
    """找垂直平面与 2D 闭合路径的交点（edge_idx, 弧长参数, 点）。

    除符号变号外，还按容差识别"交点恰好落在路径顶点/贴边穿过"的情况
    （|d| < 0.02mm 视为穿过），否则这类交点会在相邻两条边上都被漏掉。
    """
    out = []
    tol = 0.02
    for ei, edge in enumerate(wire_edges):
        L = float(edge.length)
        prev_p = edge.position_at(0.0, bd.PositionMode.LENGTH)
        prev_d = (bd.Vector(prev_p.X, prev_p.Y, 0) - center).X * normal.X + (bd.Vector(prev_p.X, prev_p.Y, 0) - center).Y * normal.Y
        if abs(prev_d) < tol:
            out.append((ei, 0.0, prev_p))
        for k in range(1, samples + 1):
            s = L * k / samples
            pt = edge.position_at(s, bd.PositionMode.LENGTH)
            pv = bd.Vector(pt.X, pt.Y, 0)
            d = (pv - center).X * normal.X + (pv - center).Y * normal.Y
            if abs(prev_d) < tol:
                out.append((ei, (k - 1) * L / samples, edge.position_at((k - 1) * L / samples, bd.PositionMode.LENGTH)))
            elif prev_d * d < 0.0:
                s0 = (k - 1) * L / samples
                s1 = s
                p0 = edge.position_at(s0, bd.PositionMode.LENGTH)
                p1 = pt
                t = prev_d / (prev_d - d)
                pc = bd.Vector(p0.X + (p1.X - p0.X) * t, p0.Y + (p1.Y - p0.Y) * t, 0)
                out.append((ei, s0 + (s1 - s0) * t, pc))
            prev_p = pt
            prev_d = d
    return out


def _clip_end_faces(placed_clip, outward: bd.Vector, anchor: bd.Vector, tangent: bd.Vector, half_span: float):
    """按局部切线投影选择卡扣左右两个扫掠端面。"""
    candidates = []
    for f in placed_clip.faces():
        if f.geom_type != bd.GeomType.PLANE:
            continue
        c = f.center()
        n = f.normal_at(c)
        bb = f.bounding_box()
        if abs(n.Z) > 0.3:
            continue
        if abs(n.X * outward.X + n.Y * outward.Y) > 0.5:
            continue
        if bb.max.Z - bb.min.Z < THICKNESS * 0.5:
            continue
        d = (c - anchor).X * tangent.X + (c - anchor).Y * tangent.Y
        candidates.append((d, f))
    if not candidates:
        return []
    left = min(candidates, key=lambda x: abs(x[0] + half_span))[1]
    right = min(candidates, key=lambda x: abs(x[0] - half_span))[1]
    return [left, right]


def build_p0_body(verts: List[bd.Vector], arcs: Dict[int, dict], hollow: bool = False,
                  params: dict = None):
    """主体：实心=P1 内部拉伸；镂空=P1−P2 夹层拉伸。卡扣仍锚定 P0 中点。"""
    edges = make_edges(verts)
    p0_edges = []
    for i, e in enumerate(edges):
        a, b = e[0], e[1]
        arc = arcs.get(i)
        if arc and float(arc.get("radius", 0)) > 0:
            r = float(arc["radius"])
            direction = str(arc.get("direction", "out"))
            mid = _arc_midpoint(a, b, r, direction)
            p0_edges.append(bd.Edge.make_three_point_arc(a, mid, b))
        else:
            p0_edges.append(bd.Edge.make_line(a, b))

    p0_wire = bd.Wire.make_wire(p0_edges)
    p0_face = bd.Face.make_from_wires(p0_wire)
    p0_center = p0_face.center()

    p1_face = None
    p2_face = None
    p1_loops = []
    p2_loops = []
    reference_errors = []

    p0_area = p0_face.area
    try:
        p1_sketch = _offset_sketch(p0_face, -INSET1)
        p1_faces = _check_inward_offset(p1_sketch, p0_area, "P1")
        p1_loops = _sketch_to_loops(p1_sketch)
        if p1_faces:
            p1_face = max(p1_faces, key=lambda f: f.area)
    except Exception as exc:
        reference_errors.append("P1 内缩失败：" + _friendly_offset_error(exc))

    p1_wire = None
    p1_corner_applied = 0
    p1_corner_warnings = []
    p1_vertex_infos = []
    if p1_face is not None:
        try:
            # 顶点角度必须在圆角前计算（圆角后尖角被替换为切线连接）。
            _pe, p1_vertex_infos = _p1_vertex_angles(p1_face)
        except Exception:
            p1_vertex_infos = []
        try:
            p1_face, p1_wire, p1_corner_applied, p1_corner_warnings = _rounded_p1_wire(p1_face)
            if p1_wire is not None:
                p1_loops = [_wire_to_loop(p1_wire)]
        except Exception as exc:
            reference_errors.append("P1 顶点圆角失败：" + _friendly_offset_error(exc))

    p2_wire = None
    p2_fillet_failed = []
    try:
        p2_sketch = _offset_sketch(p0_face, -INSET2)
        p2_faces = _check_inward_offset(p2_sketch, p0_area, "P2")
        if p2_faces:
            p2_face = max(p2_faces, key=lambda f: f.area)
            # P2 尖角先做 2D 圆角，再做后续倒角/圆角。
            p2_face, p2_wire, p2_fillet_failed = _rounded_p2_face(p2_face)
            p2_loops = [_wire_to_loop(p2_wire)] if p2_wire is not None else []
        else:
            p2_loops = []
    except Exception as exc:
        p2_face = None
        reference_errors.append("P2 内缩失败：" + _friendly_offset_error(exc))

    if p1_face is None:
        detail = reference_errors[0] if reference_errors else "P0 过薄或内凹弧使 3mm 内缩无解"
        raise ValueError(f"P1 内缩失败，无法生成 3D 主体（{detail}）")
    if hollow and (p2_face is None or p2_face.area < 1e-6):
        raise ValueError("P2 内缩失败或面积已退化，无法生成镂空夹层")

    body = bd.extrude(p1_face, amount=THICKNESS)

    # 凹弧：填充安装弦平面与 P1 弧之间的月牙空隙；
    # 凸弧：切掉 P1 弧伸出安装弦平面的月牙。
    arc_contact_solids = []
    if p1_face is not None:
        for i, edge_obj in enumerate(p0_edges):
            arc = arcs.get(i)
            if arc and float(arc.get("radius", 0)) > 0:
                radius = float(arc["radius"])
                direction = str(arc.get("direction", "out"))
                chord_anchor, rotate_edge, _half_span = _arc_p1_chord_geometry(edge_obj, radius, direction)
                if rotate_edge is None:
                    continue
                t_chord = (edge_obj.end_point() - edge_obj.start_point()).normalized()
                sign = 1.0 if direction == "out" else -1.0
                n = edges[i][3]
                radial = n * sign
                h = math.sqrt(max(0.0, radius * radius - (float((edge_obj.end_point() - edge_obj.start_point()).length) / 2.0) ** 2))
                C = (edge_obj.start_point() + edge_obj.end_point()) * 0.5 - radial * h
                R1 = radius - INSET1 * sign
                p1_mid = C + radial * R1
                if direction == "in":
                    # 凹弧空隙填充：月牙面向凹弧方向多伸 1mm，
                    # 保证填充体与主体有稳定重叠。
                    p1_mid = C + radial * (R1 + 1.0)
                try:
                    left = chord_anchor - t_chord * 5.0
                    right = chord_anchor + t_chord * 5.0
                    arc_edge = bd.Edge.make_three_point_arc(left, p1_mid, right)
                    chord_edge = bd.Edge.make_line(right, left)
                    wire = bd.Wire.make_wire([arc_edge, chord_edge])
                    face = bd.Face.make_from_wires(wire)
                    if face.is_valid() and face.area > 1e-6:
                        solid = bd.extrude(face, amount=THICKNESS)
                        bb = solid.bounding_box()
                        if abs(bb.min.Z) > 1e-6:
                            solid = solid.translate((0, 0, -bb.min.Z))
                        if direction == "in":
                            arc_contact_solids.append((solid, direction))
                except Exception:
                    continue

    for contact_solid, direction in arc_contact_solids:
        if direction != "in":
            continue
        try:
            candidate = body.fuse(contact_solid)
            if candidate.is_valid() and len(candidate.solids()) > 0:
                body = candidate
        except Exception:
            continue

    # 圆角：实心直接圆 P1 外竖直侧棱；镂空先圆 P1 外竖直侧棱，再挖 P2 内孔。
    # P2 内孔边缘不在这里圆角，交给后面的 0.75 倒角 + R1.5 圆角处理。
    body = _fillet_vertical_edges(body, CORNER_R)
    if hollow:
        p2_solid = bd.extrude(p2_face, amount=THICKNESS)
        body = body.cut(p2_solid)

    # 仅镂空：内孔上下棱做 0.75 倒角，倒角后新棱 R1.5 圆角。
    hole_report = {"chamferFailed": [], "filletFailed": []}
    if hollow and p2_wire is not None:
        body, hole_report = _apply_inner_hole_chamfer_fillet(body, p2_wire)

    clip = load_clip()
    clip_instances = []
    clip_origins = []
    clip_outward_normals = []
    clip_half_spans = []
    clip_errors = []

    for i, edge_obj in enumerate(p0_edges):
        try:
            L = float(edge_obj.length)
            mid_pt = edge_obj.position_at(L / 2.0, bd.PositionMode.LENGTH)
            outward = edges[i][3]
            arc = arcs.get(i)
            half_span = 5.0
            if arc and float(arc.get("radius", 0)) > 0:
                direction = str(arc.get("direction", "out"))
                chord_anchor, rotate_edge, _half_span = _arc_p1_chord_geometry(edge_obj, float(arc["radius"]), direction)
                half_span = _half_span
                if rotate_edge is None and p1_face is not None:
                    rotate_edge = _closest_p1_edge(p1_face, chord_anchor)
                anchor = bd.Vector(chord_anchor.X, chord_anchor.Y, THICKNESS / 2.0)
                try:
                    c = _clip_with_rotated_end_faces(rotate_edge, direction) if rotate_edge is not None else clip
                except Exception:
                    c = clip
            else:
                if p1_face is not None:
                    # 直线边锚点 = P0 边中点在 P1 直线段上的垂足。
                    target = bd.Vector(mid_pt.X, mid_pt.Y, 0) - outward * INSET1
                    p1_edge = _match_p1_line_edge(p1_face, target, outward)
                    if p1_edge is not None:
                        s_foot = _closest_param_on_edge(p1_edge, target)
                        foot_pt = p1_edge.position_at(s_foot, bd.PositionMode.LENGTH)
                        anchor = bd.Vector(foot_pt.X, foot_pt.Y, THICKNESS / 2.0)
                    else:
                        anchor = bd.Vector(mid_pt.X, mid_pt.Y, THICKNESS / 2.0)
                else:
                    anchor = bd.Vector(mid_pt.X, mid_pt.Y, THICKNESS / 2.0)
                c = clip
            origin = anchor - outward * CLIP_OVERLAP
            plane = bd.Plane(origin, x_dir=outward, z_dir=(0, 0, 1))
            placed_c = c.moved(plane.location)
            clip_instances.append(placed_c)
            clip_origins.append([round(origin.X, 3), round(origin.Y, 3), round(origin.Z, 3)])
            clip_outward_normals.append(outward)
            clip_half_spans.append(half_span)
        except Exception as exc:
            clip_errors.append({"edge": i, "error": str(exc)})

    final = body

    fused_edges = []
    detached_edges = []
    missing_edges = [item["edge"] for item in clip_errors]

    orders = [list(range(len(clip_instances)))]
    arc_indices = [i for i in range(len(clip_instances)) if i in arcs]
    if arc_indices:
        orders.append([i for i in range(len(clip_instances)) if i not in arc_indices] + arc_indices)

    best_result = None
    best_meta = None
    for order in orders:
        cur = final
        fused = []
        for i in order:
            c = clip_instances[i]
            before_volume = cur.volume
            before_solids = len(cur.solids())
            accepted = None
            for glue in (False, True):
                try:
                    candidate = cur.fuse(c, glue=glue)
                    if candidate.is_valid() and len(candidate.solids()) > 0 and candidate.volume > before_volume + 1e-6:
                        accepted = candidate
                        break
                except Exception:
                    continue
            if accepted is not None:
                cur = accepted
                fused.append(i)
        if cur.is_valid() and len(cur.solids()) == 1:
            if best_result is None or cur.volume > best_result.volume:
                best_result = cur
                best_meta = (fused, [i for i in range(len(clip_instances)) if i not in fused])

    if best_result is not None:
        final = best_result
        fused_edges = best_meta[0]
        detached_edges = best_meta[1]
    else:
        detached_shapes = [clip_instances[i] for i in range(len(clip_instances)) if i not in fused_edges]
        if detached_shapes:
            try:
                extra = final.fuse(*detached_shapes)
                if extra.is_valid() and extra.volume > final.volume + 1e-6:
                    final = extra
                else:
                    final = bd.Compound(children=[final] + detached_shapes)
            except Exception:
                final = bd.Compound(children=[final] + detached_shapes)

    # 卡扣融合后先清理/修复。注意：clean()/fix() 会就地修改底层 TopoDS，
    # 包边实体带相切边界，包边之后再调用会把实体破坏成 invalid，因此
    # 只允许在包边融合之前调用一次。
    if len(final.solids()) == 1:
        cleaned = final.clean()
        if cleaned.is_valid() and len(cleaned.solids()) > 0:
            final = cleaned
        try:
            fixed = final.fix()
            if fixed.is_valid() and len(fixed.solids()) > 0:
                final = fixed
        except Exception:
            pass

    # ------------------------------------------------------------
    # 包边：两相邻卡扣之间的轮廓段（每段恰好含一个顶点）。
    # 顶点内角 [15°,180°) 或 180° 平滑过渡 → 铺包边；
    # 凹角(>180°) / 极端角(<15° 或 >345°) → 不铺；
    # 卡扣端面区间退化（放不下包边/碰到卡扣）→ 跳过该段并记录可粘贴用例。
    # ------------------------------------------------------------
    wrap_solids = []
    wrap_applied = 0
    wrap_skipped_corners = []
    wrap_degenerate_cases = []
    wrap_failed_corners = []
    wrap_clip_overhang = []
    wrap_nudged = 0
    if p1_wire is not None and len(clip_instances) >= 3 \
            and len(p1_vertex_infos) == len(clip_instances):
        try:
            wrap_section = load_sweep_section(SWEEP_CLIP_OVERLAP)
            path_edges = _ordered_wire_edges(p1_wire)
            n = len(clip_instances)

            # 每个卡扣在圆角后 P1 路径上的前/后交点区间。
            clip_intervals = [None] * n
            for ci, placed_clip in enumerate(clip_instances):
                outward_i = clip_outward_normals[ci]
                tangent_i = bd.Vector(-outward_i.Y, outward_i.X, 0)
                anchor_i = bd.Vector(clip_origins[ci][0], clip_origins[ci][1], THICKNESS / 2.0)
                end_faces = _clip_end_faces(placed_clip, outward_i, anchor_i, tangent_i, clip_half_spans[ci])
                if len(end_faces) < 2:
                    continue
                anchor2 = bd.Vector(clip_origins[ci][0], clip_origins[ci][1], 0)
                best_edge = None
                best_s = 0.0
                best_d = 1e18
                for ei, edge in enumerate(path_edges):
                    s = _closest_param_on_edge(edge, anchor2)
                    pt = edge.position_at(s, bd.PositionMode.LENGTH)
                    d = (bd.Vector(pt.X, pt.Y, 0) - anchor2).length
                    if d < best_d:
                        best_d = d
                        best_edge = ei
                        best_s = s
                if best_edge is None:
                    continue
                tangent = path_edges[best_edge].tangent_at(best_s, bd.PositionMode.LENGTH)
                all_crossings = []
                for face in end_faces:
                    fc = face.center()
                    nrm = face.normal_at(fc)
                    all_crossings.extend(_plane_crossings(path_edges, fc, nrm))
                pos_cross = []
                neg_cross = []
                for ei, ps, pt in all_crossings:
                    rel = bd.Vector(pt.X - anchor2.X, pt.Y - anchor2.Y, 0)
                    sign = rel.X * tangent.X + rel.Y * tangent.Y
                    rec = (rel.length, {"edge": ei, "s": ps, "point": bd.Vector(pt.X, pt.Y, 0)})
                    (pos_cross if sign >= 0 else neg_cross).append(rec)
                fwd = min(pos_cross, key=lambda x: x[0])[1] if pos_cross else None
                bwd = min(neg_cross, key=lambda x: x[0])[1] if neg_cross else None
                clip_intervals[ci] = {"fwd": fwd, "bwd": bwd}
                if fwd is None or bwd is None:
                    # 卡扣（端面平面）超出 P1 范围：设计内不支持场景。
                    wrap_clip_overhang.append(ci)

            face_center_xy = bd.Vector(p1_face.center().X, p1_face.center().Y, 0)
            # 外边环绕向：用于确定包边扫掠的"外侧"法线方向。
            try:
                _nz = p1_face.normal_at(p1_face.center()).Z
            except Exception:
                _nz = 1.0
            if abs(_nz) < 1e-9:
                _nz = 1.0
            wrap_orient = 1.0 if _nz > 0 else -1.0
            for k in range(n):
                info = p1_vertex_infos[k]
                smooth = info["smooth"]
                angle = info["angle"]
                if not (smooth or (P1_CORNER_MIN_ANGLE <= angle < P1_CORNER_MAX_ANGLE)):
                    wrap_skipped_corners.append(k)
                    continue
                prev_clip = clip_intervals[(k - 1) % n]
                cur_clip = clip_intervals[k]
                record_text = _case_text(verts, arcs, params) if params is not None else ""
                start = prev_clip["fwd"] if prev_clip is not None else None
                end = cur_clip["bwd"] if cur_clip is not None else None
                reflex = (not smooth) and angle > 180.0
                # 收集该角的路径组：每个组 = (路径段列表, 扫掠起点)。
                parts_groups = []
                if reflex:
                    # 凹角（180°<α<345°）：拆段——段A：clip(k-1) 前向交点→顶点前；
                    # 延长段 + 段B：顶点→clip(k) 后向交点。整段 frenet 穿过尖锐
                    # 凹角会法线翻转 180°。某侧交点缺失（卡扣悬出边界等）时，
                    # 只铺有交点的那一侧。
                    vpos = bd.Vector(info["point"].X, info["point"].Y, 0)
                    eA = None
                    eB = None
                    for ei, pe in enumerate(path_edges):
                        if (pe.end_point() - vpos).length < 1e-4:
                            eA = ei
                        if (pe.start_point() - vpos).length < 1e-4:
                            eB = ei
                    if eA is None or eB is None:
                        wrap_degenerate_cases.append(record_text)
                        continue
                    # 段A：clip(k-1).fwd → 顶点前 WRAP_VERTEX_GAP 处（提前停止，
                    # 断开与段B 端帽的共面接触：到顶点的共面/共边端帽会让
                    # OCC 布尔把第二件判定为"已在实体中"体积不增）。
                    if start is None or start["edge"] != eA:
                        wrap_degenerate_cases.append(record_text)
                    else:
                        s0 = start["s"] + WRAP_END_GAP
                        s_end = float(path_edges[eA].length) - WRAP_VERTEX_GAP
                        if s_end - s0 < 0.5:
                            wrap_degenerate_cases.append(record_text)
                        else:
                            parts_a = _sub_edge(path_edges[eA], s0, s_end)
                            if parts_a is None:
                                wrap_degenerate_cases.append(record_text)
                            else:
                                parts_groups.append(
                                    ([parts_a],
                                     path_edges[eA].position_at(s0, bd.PositionMode.LENGTH)))
                    # 段B：顶点 → clip(k).bwd。凹角连续性由段A/段B 在顶点外侧
                    # 的空间交叉保证（>180° 必然交叉），不再需要延长段。
                    if end is None or end["edge"] != eB:
                        wrap_degenerate_cases.append(record_text)
                    else:
                        s1 = end["s"] - WRAP_END_GAP
                        if s1 <= 0.05:
                            wrap_degenerate_cases.append(record_text)
                        else:
                            parts_b = _sub_edge(path_edges[eB], 0.0, s1)
                            if parts_b is None:
                                wrap_degenerate_cases.append(record_text)
                            else:
                                parts_groups.append(
                                    ([parts_b],
                                     path_edges[eB].position_at(0.0, bd.PositionMode.LENGTH)))
                else:
                    # 凸角/平滑过渡：需要完整区间，从 start 沿路径走到 end 整段扫掠。
                    if start is None or end is None or start["edge"] == end["edge"]:
                        wrap_degenerate_cases.append(record_text)  # 区间退化
                        continue
                    out = []
                    m = start["edge"]
                    s0 = start["s"] + WRAP_END_GAP
                    if s0 < float(path_edges[m].length) - 0.05:
                        seg = _sub_edge(path_edges[m], s0, float(path_edges[m].length))
                        if seg is not None:
                            out.append(seg)
                    m = (m + 1) % len(path_edges)
                    guard = 0
                    while m != end["edge"] and guard < len(path_edges) + 2:
                        seg = _sub_edge(path_edges[m], 0.0, float(path_edges[m].length))
                        if seg is not None:
                            out.append(seg)
                        m = (m + 1) % len(path_edges)
                        guard += 1
                    if guard >= len(path_edges) + 2:
                        wrap_degenerate_cases.append(record_text)
                        continue
                    s1 = end["s"] - WRAP_END_GAP
                    if s1 > 0.05:
                        seg = _sub_edge(path_edges[m], 0.0, s1)
                        if seg is not None:
                            out.append(seg)
                    if not out or sum(float(p.length) for p in out) < 1.0:
                        wrap_degenerate_cases.append(record_text)
                        continue
                    parts_groups.append(
                        (out, path_edges[start["edge"]].position_at(s0, bd.PositionMode.LENGTH)))
                if not parts_groups:
                    continue

                has_arc = any(p.geom_type == bd.GeomType.CIRCLE
                              for pg, _spt in parts_groups for p in pg)

                def _fuse_pieces(pieces):
                    nonlocal final, wrap_nudged
                    ok_count = 0
                    for sol in pieces:
                        accepted = False
                        for delta in (0.0, 0.02, 0.05):
                            piece = sol
                            if delta > 0.0:
                                sc = sol.center()
                                nvec = bd.Vector(sc.X - face_center_xy.X, sc.Y - face_center_xy.Y, 0)
                                nl = nvec.length
                                if nl < 1e-9:
                                    break
                                nvec = nvec.normalized()
                                piece = sol.translate((-nvec.X * delta, -nvec.Y * delta, 0))
                            before = final.volume
                            ok = False
                            for glue in (False, True):
                                try:
                                    cand = final.fuse(piece, glue=glue)
                                    if cand.is_valid() and len(cand.solids()) > 0 \
                                            and cand.volume > before + 1e-6:
                                        final = cand
                                        accepted = True
                                        ok = True
                                        if delta > 0.0:
                                            wrap_nudged += 1
                                        break
                                except Exception:
                                    continue
                            if ok:
                                break
                        if accepted:
                            wrap_solids.append((sol, k))
                            ok_count += 1
                    return ok_count

                # 先按原样（弧保持圆弧）扫掠：凸角圆弧连续无缝。
                seg_solids = []
                for pg, spt in parts_groups:
                    seg_solids.extend(_sweep_wrap_segment(wrap_section, pg, wrap_orient, spt))
                if not seg_solids:
                    wrap_failed_corners.append(k)
                    continue
                if reflex and has_arc:
                    # 凹角 + 含弧：圆弧扫掠件的逐件融合不可靠（后融的弧件与
                    # 已有实体求交退化：glue=False invalid / glue=True 吞件，
                    # 直线-弧混合边类型必现）；Compound 一次布尔在含相邻角
                    # 包边的复杂主体上又极慢。折线化后全部是直线件，逐件
                    # 布尔稳定（弦高差≈0.002mm，打印不可见；凹角拆段本就
                    # 是断开的，无圆弧连续性损失）。
                    faceted = []
                    for pg, _spt in parts_groups:
                        faceted.extend(_sweep_wrap_segment_faceted(wrap_section, pg, wrap_orient))
                    seg_ok = _fuse_pieces(faceted) if faceted else 0
                else:
                    seg_ok = _fuse_pieces(seg_solids)
                # 弧扫掠实体在部分组合下会被布尔并静默丢弃（融合不完整）：
                # 折线化重试整组。
                if seg_ok < len(seg_solids) and has_arc and not reflex:
                    faceted = []
                    for pg, _spt in parts_groups:
                        faceted.extend(_sweep_wrap_segment_faceted(wrap_section, pg, wrap_orient))
                    if faceted:
                        seg_ok += _fuse_pieces(faceted)
                if seg_ok:
                    wrap_applied += 1
                else:
                    wrap_failed_corners.append(k)
        except Exception as exc:
            reference_errors.append("包边失败：" + str(exc))

    wrap_degenerate_cases = list(dict.fromkeys(wrap_degenerate_cases))

    solids = final.solids()

    actual_angles = []
    for j in range(len(edges)):
        prev_t = edges[(j - 1) % len(edges)][2]
        next_t = edges[j][2]
        turn = math.atan2(prev_t.X * next_t.Y - prev_t.Y * next_t.X,
                          prev_t.X * next_t.X + prev_t.Y * next_t.Y)
        angle = math.pi - turn
        if angle < 0.0:
            angle += 2.0 * math.pi
        actual_angles.append(round(math.degrees(angle), 2))

    bb = final.bounding_box()
    meta = {
        "sides": len(verts),
        "freeForm": True,
        "p0Only": True,
        "hollow": bool(hollow),
        "thickness": THICKNESS,
        "cornerR": CORNER_R,
        "p1CornerR": P1_CORNER_R,
        "p1CornerApplied": p1_corner_applied,
        "p1CornerWarnings": p1_corner_warnings,
        "holeChamfer": HOLE_CHAMFER,
        "holeFilletR": HOLE_FILLET_R,
        "lengths": [round(float((edges[i][1] - edges[i][0]).length), 3) for i in range(len(edges))],
        "interiorAngles": actual_angles,
        "arcEdges": {str(k): dict(v) for k, v in arcs.items()},
        "clipCount": len(clip_instances),
        "sweepCount": len(wrap_solids),
        "wrapApplied": wrap_applied,
        "wrapSkippedCorners": wrap_skipped_corners,
        "wrapDegenerateCases": wrap_degenerate_cases,
        "clipOverhangWarnings": [f"边{i + 1}太短，卡扣超出边框范围，无法生成几何片。" for i in wrap_clip_overhang],
        "wrapFailedCorners": wrap_failed_corners,
        "wrapNudged": wrap_nudged,
        "fusedClipCount": len(fused_edges),
        "detachedClipEdges": detached_edges,
        "missingClipEdges": missing_edges,
        "clipOrigins": clip_origins,
        "clipErrors": clip_errors,
        "referenceErrors": reference_errors,
        "p2FilletFailedVertices": p2_fillet_failed,
        "holeChamferFailed": hole_report["chamferFailed"],
        "holeFilletFailed": hole_report["filletFailed"],
        "solidCount": len(solids),
        "connected": len(solids) == 1,
        "isValid": bool(final.is_valid()),
        "volume": round(final.volume, 4),
        "faceCount": len(final.faces()),
        "bbox": [round(bb.min.X, 3), round(bb.min.Y, 3), round(bb.min.Z, 3),
                 round(bb.max.X, 3), round(bb.max.Y, 3), round(bb.max.Z, 3)],
    }
    return final, p0_edges, p1_loops, p2_loops, meta


# ------------------------------------------------------------
# STL / 预览
# ------------------------------------------------------------
def _repair_stl_boundaries(data: bytes, tol: float = 0.02, max_hole_mm: float = 5.0) -> Tuple[bytes, int]:
    """关闭 STL 中由微小圆弧/布尔运算产生的边界缝隙。

    只修补周长小于 max_hole_mm 的小边界环；不影响正常几何。
    """
    def dist3(p, q):
        return math.sqrt(sum((p[k] - q[k]) ** 2 for k in range(3)))

    n = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(n):
        off = 84 + 50 * i
        normal = struct.unpack("<3f", data[off:off + 12])
        v = [struct.unpack("<3f", data[off + 12 + 12 * j: off + 24 + 12 * j]) for j in range(3)]
        # 先移除零长度/退化三角片，它们会造成假边界
        if v[0] == v[1] or v[1] == v[2] or v[0] == v[2]:
            continue
        if min(dist3(v[a], v[b]) for a, b in ((0, 1), (1, 2), (2, 0))) < 1e-7:
            continue
        tris.append((v, normal))

    edge_count = {}
    edge_owner = {}
    for ti, (v, normal) in enumerate(tris):
        for a, b in ((0, 1), (1, 2), (2, 0)):
            key = tuple(sorted((tuple(round(v[a][k], 6) for k in range(3)),
                               tuple(round(v[b][k], 6) for k in range(3)))))
            edge_count[key] = edge_count.get(key, 0) + 1
            edge_owner.setdefault(key, []).append((ti, a, b))

    boundary = []
    for key, owners in edge_owner.items():
        if edge_count.get(key, 0) == 1:
            ti, a, b = owners[0]
            boundary.append((tris[ti][0][a], tris[ti][0][b], ti, key))

    used = [False] * len(boundary)
    filled = 0
    new_tris = list(tris)
    for si in range(len(boundary)):
        if used[si]:
            continue
        loop = [boundary[si]]
        used[si] = True
        while True:
            end = loop[-1][1]
            best = None
            best_d = tol
            for j, edge in enumerate(boundary):
                if used[j]:
                    continue
                d = dist3(end, edge[0])
                if d < best_d:
                    best_d = d
                    best = j
            if best is None:
                break
            used[best] = True
            loop.append(boundary[best])
            if dist3(loop[-1][1], loop[0][0]) < tol:
                break
            if len(loop) > 200:
                break
        if len(loop) < 3:
            continue
        perimeter = sum(dist3(loop[i][1], loop[(i + 1) % len(loop)][0]) for i in range(len(loop)))
        if perimeter > max_hole_mm:
            continue
        center = [0.0, 0.0, 0.0]
        for edge in loop:
            for k in range(3):
                center[k] += edge[0][k]
        center = tuple(c / len(loop) for c in center)
        for edge in loop:
            a, b = edge[0], edge[1]
            if dist3(a, center) < 1e-9 or dist3(b, center) < 1e-9:
                continue
            # 使用与相邻三角形相反的边方向，保证法线一致
            new_tris.append(((b, a, center), (0.0, 0.0, 0.0)))
            filled += 1

    out = io.BytesIO()
    out.write(b"\0" * 80)
    out.write(struct.pack("<I", len(new_tris)))
    for v, normal in new_tris:
        u = [v[1][k] - v[0][k] for k in range(3)]
        w = [v[2][k] - v[0][k] for k in range(3)]
        nx = u[1] * w[2] - u[2] * w[1]
        ny = u[2] * w[0] - u[0] * w[2]
        nz = u[0] * w[1] - u[1] * w[0]
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length < 1e-12:
            nx, ny, nz = normal
        else:
            nx, ny, nz = nx / length, ny / length, nz / length
        out.write(struct.pack("<3f", nx, ny, nz))
        for p in v:
            out.write(struct.pack("<3f", p[0], p[1], p[2]))
        out.write(struct.pack("<H", 0))
    return out.getvalue(), filled


def shape_to_stl_bytes(shape) -> Tuple[bytes, dict]:
    with tempfile.TemporaryDirectory() as tmp:
        raw = Path(tmp) / "raw.stl"
        bd.export_stl(shape, str(raw), tolerance=MESH_TOLERANCE, angular_tolerance=0.1)
        data = raw.read_bytes()
    tri_count = struct.unpack("<I", data[80:84])[0]
    stats = {"triangles": tri_count, "bytes": len(data)}
    return data, stats


def shape_to_preview(shape) -> dict:
    data, _stats = shape_to_stl_bytes(shape)
    n = struct.unpack("<I", data[80:84])[0]
    index = {}
    vertices = []
    faces = []
    for i in range(n):
        off = 84 + 50 * i
        tri_ids = []
        for j in range(3):
            p = struct.unpack("<3f", data[off + 12 + 12 * j: off + 24 + 12 * j])
            key = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
            if key not in index:
                index[key] = len(vertices)
                vertices.append([key[0], key[1], key[2]])
            tri_ids.append(index[key])
        faces.append(tri_ids)
    return {"vertices": vertices, "faces": faces,
            "vertexCount": len(vertices), "triangleCount": len(faces)}


# ------------------------------------------------------------
# 参数解析与生成入口
# ------------------------------------------------------------
def parse_geometry(params: dict):
    free_side_lengths = params.get("freeSideLengths")
    free_angles = params.get("freeAngles")
    free_solution = int(params.get("freeSolution", 0))
    arcs = {
        int(k): {"radius": float(v.get("radius", 0)), "direction": str(v.get("direction", "out"))}
        for k, v in (params.get("freeArcs") or {}).items() if isinstance(v, dict)
    }
    lengths = params.get("lengths")
    if free_side_lengths is not None:
        lens = [float(v) for v in free_side_lengths]
        angs = [float(v) for v in (free_angles or [])]
        solutions = solve_free_polygon_solutions(lens, angs)
        solution = max(0, min(free_solution, len(solutions) - 1))
        verts, _auto, _closing = solutions[solution][0], solutions[solution][1], solutions[solution][2]
        return verts, arcs
    if lengths is not None:
        return triangle_from_lengths([float(v) for v in lengths]), arcs
    sides = int(params.get("sides", 3))
    side_len = float(params.get("sideLen", 40.0))
    if sides < 3 or sides > 12:
        raise ValueError("边数范围 3–12")
    if side_len < MIN_SIDE_MM or side_len > MAX_SIDE_MM:
        raise ValueError(f"边长范围 {MIN_SIDE_MM:g}–{MAX_SIDE_MM:g} mm")
    # 正多边形也可能有多个解（如正五/六边形有 2 个解），按 freeSolution 选择；
    # 解1 即标准正多边形。求解失败时回退常规构造。
    try:
        lens = [side_len] * sides
        angs = [round((sides - 2) * 180.0 / sides, 4)] * max(0, sides - 3)
        solutions = solve_free_polygon_solutions(lens, angs)
        sol = max(0, min(free_solution, len(solutions) - 1))
        return solutions[sol][0], arcs
    except Exception:
        return regular_polygon(sides, side_len), arcs


# 深凸弧内缩崩溃防护：
# 弧端切线与相邻直边近共线（偏离度 0.5°~45°）时，INTERSECTION 内缩会在
# OCC 7.7.2 原生层段错误（交点落圆弧参数缝邻域，Python 拦不住，worker 进程死）。
# 该区间 TANGENT 与 INTERSECTION 几何等价且稳定（实测面积/顶点一致）；
# 精确共线（0°）是切点分支，两法皆安全；>45° 交点分离良好，保持 INTERSECTION。
INSET_DEV_LO = 0.5
INSET_DEV_HI = 45.0


def _dir_diff_deg(a: float, b: float) -> float:
    """两方向角之差，归一到 [-180, 180]。"""
    d = (a - b) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def _face_dangerous(p0_face) -> bool:
    """预检：存在弧-直线顶点且弧端切线与邻边近共线（偏离度∈(0.5°,45°)）→ True。"""
    try:
        edges = list(p0_face.outer_wire().edges())
    except Exception:
        return False
    n = len(edges)
    for i, e in enumerate(edges):
        if e.geom_type != bd.GeomType.CIRCLE:
            continue
        prev_e = edges[(i - 1) % n]
        next_e = edges[(i + 1) % n]
        L = float(e.length)
        t_in = e.tangent_at(0.0, bd.PositionMode.LENGTH)
        t_out = e.tangent_at(L, bd.PositionMode.LENGTH)
        a_in = math.degrees(math.atan2(t_in.Y, t_in.X))
        a_out = math.degrees(math.atan2(t_out.Y, t_out.X))
        if prev_e.geom_type == bd.GeomType.LINE:
            t_prev = prev_e.tangent_at(float(prev_e.length), bd.PositionMode.LENGTH)
            a_prev = math.degrees(math.atan2(t_prev.Y, t_prev.X))
            dev = abs(_dir_diff_deg(a_in, a_prev))
            dev = min(dev, 180.0 - dev)
            if INSET_DEV_LO < dev < INSET_DEV_HI:
                return True
        if next_e.geom_type == bd.GeomType.LINE:
            t_next = next_e.tangent_at(0.0, bd.PositionMode.LENGTH)
            a_next = math.degrees(math.atan2(t_next.Y, t_next.X))
            dev = abs(_dir_diff_deg(a_out, a_next))
            dev = min(dev, 180.0 - dev)
            if INSET_DEV_LO < dev < INSET_DEV_HI:
                return True
    return False


def _offset_sketch(p0_face, amount: float):
    """内缩 offset。

    预检弧-直线近共线顶点（偏离度 0.5°~45°）：危险 → TANGENT（该区间与
    INTERSECTION 几何等价且不崩），失败回退 ARC（ARC 在崩溃矩阵实测稳定，
    不再回 INTERSECTION，避免段错误路径）；其余 → INTERSECTION，失败回退 ARC。
    """
    last_error = None
    if amount < 0 and _face_dangerous(p0_face):
        kinds = (bd.Kind.TANGENT, bd.Kind.ARC)
    else:
        kinds = (bd.Kind.INTERSECTION, bd.Kind.ARC)
    for kind in kinds:
        try:
            return bd.offset(p0_face, amount=amount, kind=kind)
        except Exception as exc:
            last_error = exc
    raise last_error


def _friendly_offset_error(exc: Exception) -> str:
    msg = str(exc)
    mapping = {
        "Multiple Wires generated": "内缩结果分裂为多条闭合线，当前图形在该内缩距离下不再是单连通区域",
        "No offset generated": "内缩无结果：图形过薄或内缩距离过大",
        "Null TopoDS_Shape object": "内缩无结果：图形过薄或内缩距离过大",
    }
    return mapping.get(msg, msg)


def _check_inward_offset(sketch, p0_area: float, label: str):
    """内缩结果面积必须小于 P0；面积变大说明 offset 发生翻转/自交叠。"""
    faces = list(sketch.faces()) if hasattr(sketch, "faces") else [sketch]
    total_area = sum(f.area for f in faces)
    if total_area > p0_area + 1e-6:
        raise ValueError(f"{label} 内缩发生翻转/自交叠（面积 {total_area:.2f} > P0 {p0_area:.2f}）")
    return faces


def _case_text(verts, arcs, params):
    """生成 2D 右下角用例描述。"""
    if params.get("freeSideLengths") is None:
        if params.get("lengths") is not None:
            lens = [float(v) for v in params["lengths"]]
            return "三边预设：" + "，".join(f"{v:g}" for v in lens)
        text = f"正{params.get('sides', 3)}边形，边长{float(params.get('sideLen', 40)):g}"
        if int(params.get("freeSolution", 0)) > 0:
            text += f"；解{int(params.get('freeSolution', 0)) + 1}"
        return text
    edges = make_edges(verts)
    parts = []
    for i, e in enumerate(edges):
        chord = float((e[1] - e[0]).length)
        arc = arcs.get(i)
        if arc and float(arc.get("radius", 0)) > 0:
            direction = "凹弧" if str(arc.get("direction", "out")) == "in" else "凸弧"
            r = float(arc["radius"])
            a, b = e[0], e[1]
            L = float((b - a).length)
            arc_len = 2.0 * r * math.asin(max(-1.0, min(1.0, L / (2.0 * r))))
            parts.append(f"{chord:g}（{direction}，R{r:g}，弧长{arc_len:.2f}）")
        else:
            parts.append(f"{chord:g}")
    text = "，".join(parts)
    angs = params.get("freeAngles") or []
    if angs:
        text += "；角" + "，".join(f"{float(a):g}°" for a in angs)
    if int(params.get("freeSolution", 0)) > 0:
        text += f"；解{int(params.get('freeSolution', 0)) + 1}"
    return text


def get_p0_2d(params: dict):
    """只计算 2D 绘制数据，不生成 3D 实体，不导出 STL。"""
    solution_count = 1
    solution_index = 0
    if params.get("freeSideLengths") is not None:
        lens = [float(v) for v in params["freeSideLengths"]]
        angs = [float(v) for v in (params.get("freeAngles") or [])]
        solution_count = len(solve_free_polygon_solutions(lens, angs))
        solution_index = max(0, min(int(params.get("freeSolution", 0)), solution_count - 1))
    else:
        # 正多边形模式：等价自由多边形可能有多个解（如正五/六边形 2 解）。
        try:
            n = int(params.get("sides", 3))
            side_len = float(params.get("sideLen", 40.0))
            lens = [side_len] * n
            angs = [round((n - 2) * 180.0 / n, 4)] * max(0, n - 3)
            solution_count = len(solve_free_polygon_solutions(lens, angs))
            solution_index = max(0, min(int(params.get("freeSolution", 0)), solution_count - 1))
        except Exception:
            solution_count = 1
            solution_index = 0
    verts, arcs = parse_geometry(params)
    edges = make_edges(verts)
    p0_edges = []
    for i, e in enumerate(edges):
        arc = arcs.get(i)
        if arc and float(arc.get("radius", 0)) > 0:
            r = float(arc["radius"])
            direction = str(arc.get("direction", "out"))
            mid = _arc_midpoint(e[0], e[1], r, direction)
            p0_edges.append(bd.Edge.make_three_point_arc(e[0], mid, e[1]))
        else:
            p0_edges.append(bd.Edge.make_line(e[0], e[1]))

    p0_face = bd.Face.make_from_wires(bd.Wire.make_wire(p0_edges))
    p0_center = p0_face.center()

    p1_face = None
    p1_loops = []
    p2_loops = []
    reference_errors = []
    p0_area = p0_face.area
    try:
        p1_sketch = _offset_sketch(p0_face, -INSET1)
        p1_faces = _check_inward_offset(p1_sketch, p0_area, "P1")
        p1_loops = _sketch_to_loops(p1_sketch)
        if p1_faces:
            p1_face = max(p1_faces, key=lambda f: f.area)
    except Exception as exc:
        reference_errors.append("P1 内缩失败：" + _friendly_offset_error(exc))

    p1_corner_warnings = []
    if p1_face is not None:
        try:
            p1_face, p1_wire, _applied, p1_corner_warnings = _rounded_p1_wire(p1_face)
            if p1_wire is not None:
                p1_loops = [_wire_to_loop(p1_wire)]
        except Exception as exc:
            reference_errors.append("P1 顶点圆角失败：" + _friendly_offset_error(exc))

    try:
        p2_sketch = _offset_sketch(p0_face, -INSET2)
        p2_faces = _check_inward_offset(p2_sketch, p0_area, "P2")
        if p2_faces:
            p2_face, p2_wire, p2_fillet_failed = _rounded_p2_face(max(p2_faces, key=lambda f: f.area))
            p2_loops = [_wire_to_loop(p2_wire)] if p2_wire is not None else []
    except Exception as exc:
        reference_errors.append("P2 内缩失败：" + _friendly_offset_error(exc))

    clip_origins = []
    for i, edge_obj in enumerate(p0_edges):
        arc = arcs.get(i)
        L = float(edge_obj.length)
        mid_pt = edge_obj.position_at(L / 2.0, bd.PositionMode.LENGTH)
        outward = edges[i][3]
        if arc and float(arc.get("radius", 0)) > 0:
            # 2D 红点 = P0 弧中点在 P1 弧上的垂足。
            direction = str(arc.get("direction", "out"))
            foot = _arc_p1_midpoint(edge_obj, float(arc["radius"]), direction)
            anchor = bd.Vector(foot.X, foot.Y, THICKNESS / 2.0)
        else:
            if p1_face is not None:
                # 2D 红点 = P0 边中点在 P1 直线段上的垂足。
                target = bd.Vector(mid_pt.X, mid_pt.Y, 0) - outward * INSET1
                p1_edge = _match_p1_line_edge(p1_face, target, outward)
                if p1_edge is not None:
                    s_foot = _closest_param_on_edge(p1_edge, target)
                    foot_pt = p1_edge.position_at(s_foot, bd.PositionMode.LENGTH)
                    anchor = bd.Vector(foot_pt.X, foot_pt.Y, THICKNESS / 2.0)
                else:
                    anchor = bd.Vector(mid_pt.X, mid_pt.Y, THICKNESS / 2.0)
            else:
                anchor = bd.Vector(mid_pt.X, mid_pt.Y, THICKNESS / 2.0)
        origin = anchor - outward * CLIP_OVERLAP
        clip_origins.append([round(origin.X, 3), round(origin.Y, 3), round(origin.Z, 3)])

    # 卡扣悬出 P1 范围检测（与 3D 包边一致：按 3D 锚点与半宽判断）。
    clip_overhang_warnings = []
    if p1_face is not None:
        for i, edge_obj in enumerate(p0_edges):
            arc = arcs.get(i)
            half = 5.0
            if arc and float(arc.get("radius", 0)) > 0:
                chord_anchor, _re, _hs = _arc_p1_chord_geometry(
                    edge_obj, float(arc["radius"]), str(arc.get("direction", "out")))
                anchor3d = bd.Vector(chord_anchor.X, chord_anchor.Y, 0)
                half = _hs
            else:
                outward = edges[i][3]
                target = bd.Vector(edge_obj.position_at(float(edge_obj.length) / 2.0, bd.PositionMode.LENGTH).X,
                                   edge_obj.position_at(float(edge_obj.length) / 2.0, bd.PositionMode.LENGTH).Y, 0) \
                    - outward * INSET1
                p1_edge = _match_p1_line_edge(p1_face, target, outward)
                if p1_edge is None:
                    continue
                s_foot = _closest_param_on_edge(p1_edge, target)
                foot_pt = p1_edge.position_at(s_foot, bd.PositionMode.LENGTH)
                anchor3d = bd.Vector(foot_pt.X, foot_pt.Y, 0)
            best_e = None
            best_s = 0.0
            best_d = 1e18
            wire_edges = list(p1_face.outer_wire().edges())
            for ei, pe in enumerate(wire_edges):
                s = _closest_param_on_edge(pe, anchor3d)
                pt = pe.position_at(s, bd.PositionMode.LENGTH)
                d = (bd.Vector(pt.X, pt.Y, 0) - anchor3d).length
                if d < best_d:
                    best_d, best_e, best_s = d, ei, s
            if best_e is not None:
                Lp = float(wire_edges[best_e].length)
                if best_s - half < -0.1 or best_s + half > Lp + 0.1:
                    clip_overhang_warnings.append(f"边{i + 1}太短，卡扣超出边框范围，无法生成几何片。")

    p0_polylines = [_edge_to_polyline(e, 24) for e in p0_edges]
    return {
        "p0": p0_polylines,
        "p1": p1_loops,
        "p2": p2_loops,
        "clipOrigins": clip_origins,
        "referenceErrors": reference_errors,
        "p1CornerWarnings": p1_corner_warnings,
        "clipOverhangWarnings": clip_overhang_warnings,
        "solutionCount": solution_count,
        "solutionIndex": solution_index,
        "caseText": _case_text(verts, arcs, params),
    }


def get_p0_result(params: dict):
    verts, arcs = parse_geometry(params)
    hollow = bool(params.get("hollow", False))
    shape, p0_edges, p1_loops, p2_loops, meta = build_p0_body(verts, arcs, hollow=hollow, params=params)
    p0_polylines = [_edge_to_polyline(e, 24) for e in p0_edges]
    preview = shape_to_preview(shape)
    return shape, {
        "p0": p0_polylines,
        "p1": p1_loops,
        "p2": p2_loops,
        "preview": preview,
        "meta": meta,
    }


# ------------------------------------------------------------
# HTTP 服务
# ------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    server_version = "TileGeneratorV3/0.1"

    def log_message(self, fmt, *args):
        sys.stderr.write("[tile-v3] " + fmt % args + "\n")

    def _send_json(self, obj, status=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path, content_type):
        if not path.exists():
            self.send_error(404, "not found")
            return
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send_file(HTML_FILE, "text/html; charset=utf-8")
        elif path == "/vendor/three/three.module.js":
            self._send_file(VENDOR_DIR / "three" / "three.module.js", "text/javascript")
        elif path == "/vendor/three/OrbitControls.js":
            self._send_file(VENDOR_DIR / "three" / "OrbitControls.js", "text/javascript")
        elif path == "/api/health":
            self._send_json({"ok": True, "step": str(STEP_FILE), "stepExists": STEP_FILE.exists()})
        else:
            self.send_error(404, "not found")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length else b"{}"
            params = json.loads(body.decode("utf-8") or "{}")
        except Exception:
            self._send_json({"error": "invalid JSON"}, 400)
            return

        if path == "/api/preview2d":
            try:
                _status, result2d = _submit_build("2d", (params,))
            except TimeoutError as exc:
                self._send_json({"error": str(exc)}, 504)
                return
            except Exception as exc:
                self._send_json({"error": str(exc)}, 400)
                return
            self._send_json({"ok": True, **result2d})
            return

        if path == "/api/preview":
            try:
                _status, result = _submit_build("3d", ("preview", params))
            except TimeoutError as exc:
                self._send_json({"error": str(exc)}, 504)
                return
            except Exception as exc:
                self._send_json({"error": str(exc)}, 400)
                return
            self._send_json({"ok": True, **result})
            return

        if path == "/api/generate":
            try:
                _status, payload = _submit_build("3d", ("stl", params))
            except TimeoutError as exc:
                self._send_json({"error": str(exc)}, 504)
                return
            except Exception as exc:
                self._send_json({"error": f"STL export failed: {exc}"}, 500)
                return
            name = payload.get("name") or "tile_v3_p0.stl"
            quoted = urllib.parse.quote(name, safe="")
            self.send_response(200)
            self.send_header("Content-Type", "model/stl")
            self.send_header("Content-Disposition",
                             f'attachment; filename="tile_v3_p0.stl"; filename*=UTF-8\'\'{quoted}')
            self.send_header("Content-Length", str(len(payload["data"])))
            self.end_headers()
            self.wfile.write(payload["data"])
            return

        self._send_json({"error": "unknown endpoint"}, 404)


def _warmup_workers(timeout_s: float = 300.0) -> bool:
    """预热构建子进程，使其完成 spawn 与 build123d/OCCT 的加载。

    子进程采用 spawn 模式，需重新导入本模块、build123d 及约 771MB 的
    OCCT 动态库。若不预热，这份开销会计入**首个**用户请求，而内置的
    BUILD_TIMEOUT_S（60s）看门狗会把「冷启动 + 构建」一起计时——在较慢
    的机器上首请求会被误判为超时（504），且看门狗重启子进程后，下一次
    请求又回到冷启动，形成死循环。

    子进程在完成 import 后进入 q_in.get() 阻塞等待。因此只需等待
    「子进程存活」再留出一段导入时间即可；这里通过提交一个真实的最小
    构建任务来确认其已能响应，并临时放宽超时，避免把导入时间算作构建
    超时。

    返回 True 表示 2d 与 3d worker 均已就绪。
    """
    global BUILD_TIMEOUT_S
    _start_build_workers()
    ok = True
    saved_timeout = BUILD_TIMEOUT_S
    # 预热期间放宽看门狗：此时耗时主要是子进程导入几何内核，而非构建本身。
    BUILD_TIMEOUT_S = max(saved_timeout, timeout_s)
    try:
        for kind, probe in (("2d", ({"sides": 3, "sideLen": MIN_SIDE_MM},)),
                            ("3d", ("preview", {"sides": 3, "sideLen": MIN_SIDE_MM}))):
            st = _worker_state.get(kind)
            if st is None or st.get("proc") is None:
                ok = False
                continue
            try:
                _submit_build(kind, probe)
            except Exception as exc:
                print(f"警告：{kind} worker 预热失败：{exc}")
                ok = False
    finally:
        BUILD_TIMEOUT_S = saved_timeout
    return ok


def run_server(port=8768, no_browser=False, warmup=False, warmup_timeout=300.0):
    import time
    if not HTML_FILE.exists():
        print(f"警告：缺少 {HTML_FILE}")
    if not STEP_FILE.exists():
        print(f"警告：缺少 {STEP_FILE}")

    if warmup:
        print("正在预热几何构建子进程（首次运行需加载几何内核，请稍候）...")
        t0 = time.time()
        if _warmup_workers(warmup_timeout):
            print(f"预热完成（{time.time() - t0:.1f}s）")
        else:
            print(f"警告：预热未在 {warmup_timeout:g}s 内完成，首次请求可能较慢")
    else:
        # 后台预热：不阻塞服务启动与页面访问
        threading.Thread(target=_warmup_workers, daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"几何拼接片 V3 已启动： http://127.0.0.1:{port}")
    print("按 Ctrl+C 退出")
    if not no_browser:
        threading.Thread(target=lambda: (time.sleep(0.8), webbrowser.open(f"http://127.0.0.1:{port}")), daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="几何拼接片 V3 · P0 基准版")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--warmup", action="store_true",
                        help="启动时阻塞预热几何构建子进程（用于 CI/自动化，避免首请求超时）")
    parser.add_argument("--warmup-timeout", type=float, default=300.0,
                        help="预热等待上限（秒），默认 300")
    args = parser.parse_args()
    run_server(args.port, args.no_browser, args.warmup, args.warmup_timeout)
