# -*- coding: utf-8 -*-
# core.py (self-contained)
"""Pure algorithms for reconstructing a tower body from front and side views."""
from typing import Dict, List, Tuple, Optional, TypeAlias, Any, Set
import math
import re

Point3D: TypeAlias = Tuple[float, float, float]
Seg3D: TypeAlias = List[Point3D]
Model3DData: TypeAlias = Dict[str, Seg3D]


def _model_sort_key(value: Any) -> Tuple[Tuple[Tuple[int, Any], ...], str]:
    """Natural drawing order: 09 before 10, J1-9 before J1-10."""
    text = str(value)
    parts = re.split(r"(\d+)", text)
    key = tuple(
        (0, int(part)) if part.isdigit() else (1, part.lower())
        for part in parts
    )
    return key, text.lower()


def base_id(rid) -> str:
    """Return the original member ID without its duplicate-instance suffix."""
    return str(rid).split("_", 1)[0]


def node_id_base(rid) -> str:
    """Build a numeric node-ID prefix without losing a duplicate-instance suffix."""
    return str(rid).replace("_", "")

def _collect_endpoints(segdict: Model3DData) -> List[Point3D]:
    pts: List[Point3D] = []
    for seg in segdict.values():
        if seg and len(seg) >= 2:
            pts.append(seg[0]); pts.append(seg[1])
    return pts

def center_props(front3d: Model3DData, right3d: Model3DData):
    """Return the combined model center in XY and the minimum Z coordinate."""
    all_pts = _collect_endpoints(front3d) + _collect_endpoints(right3d)
    if not all_pts: return (0.0, 0.0, 0.0), 0.0
    xs = [p[0] for p in all_pts]; ys = [p[1] for p in all_pts]; zs = [p[2] for p in all_pts]
    x_center = (min(xs) + max(xs)) / 2.0
    y_center = (min(ys) + max(ys)) / 2.0
    z_min = min(zs)
    return (x_center, y_center, 0.0), z_min

def translate_model(front3d: Model3DData, right3d: Model3DData, translation_target: Tuple[float,float,float]):
    dx, dy, dz = -translation_target[0], -translation_target[1], -translation_target[2]
    def _apply(d: Model3DData) -> Model3DData:
        return {gid: [(p[0]+dx, p[1]+dy, p[2]+dz) for p in seg] for gid, seg in d.items()}
    return _apply(front3d), _apply(right3d)

def top_xmid_and_range(horiz_dict: Dict[str, List[Tuple[float, float]]],
                       preferred_key: Optional[str] = None,
                       y_top: Optional[float] = None,
                       tol_abs: Optional[float] = None,
                       tol_ratio: float = 0.05,
                       y_eps: float = 1e-6):
    if not horiz_dict: return None, None, None
    def _seg_info(seg):
        (x1, y1), (x2, y2) = seg[0], seg[1]
        length = abs(x2 - x1); y_mean = (y1 + y2) / 2.0
        return x1, y1, x2, y2, length, y_mean
    chosen_key, seg = None, None
    if preferred_key and preferred_key in horiz_dict:
        chosen_key, seg = preferred_key, horiz_dict[preferred_key]
    if seg is None and (y_top is not None):
        cand = []
        for k, s in horiz_dict.items():
            if not s or len(s) < 2: continue
            x1,y1,x2,y2,L,ym = _seg_info(s)
            if abs(y1 - y_top) <= y_eps and abs(y2 - y_top) <= y_eps:
                cand.append((L, k, s))
        if cand:
            cand.sort(reverse=True); _, chosen_key, seg = cand[0]
    if seg is None:
        best = None
        for k,s in horiz_dict.items():
            if not s or len(s) < 2: continue
            x1,y1,x2,y2,L,ym = _seg_info(s)
            if (best is None) or (ym > best[0]): best = (ym, k, s)
        if best: _, chosen_key, seg = best
    if seg is None or len(seg) < 2: return None, None, None
    x1,y1,x2,y2,seg_len,_ = _seg_info(seg)
    x_mid = (x1 + x2) / 2.0
    tol = float(tol_abs) if (tol_abs is not None) else (abs(seg_len) * (tol_ratio if tol_ratio is not None else 0.05) or 1.0)
    x_range = (x_mid - tol, x_mid + tol)
    meta = {"key": chosen_key, "seg_len": seg_len, "y": (y1+y2)/2.0, "tol": tol, "endpoints": [(x1,y1),(x2,y2)]}
    return x_mid, x_range, meta

def select_x_type(rest_dict: Dict[str, List[Tuple[float, float]]], x_range: Tuple[float, float]) -> Dict[str, List[Tuple[float, float]]]:
    if not rest_dict or not x_range: return {}
    lo, hi = x_range; out = {}
    for k, seg in rest_dict.items():
        if not seg or len(seg) < 2: continue
        (x1,y1),(x2,y2) = seg[0],seg[1]
        x_mid = (x1 + x2) / 2.0
        if lo <= x_mid <= hi: out[str(k)] = [tuple(seg[0]), tuple(seg[1])]
    return out

def scale_by_member(final_coords_map: Dict[str, List[Tuple[float, float, float]]], target_id: str, real_length: float):
    p1, p2 = final_coords_map.get(target_id, (None, None))
    if p1 is None or p2 is None: raise KeyError(f"member id {target_id} not found")
    L = math.sqrt(sum((a-b)**2 for a,b in zip(p1, p2)))
    if L < 1e-9: raise ValueError("selected member is too short")
    s = real_length / L
    return {mid: [(p[0]*s, p[1]*s, p[2]*s) for p in seg] for mid, seg in final_coords_map.items()}


# ====== Loader ======
# loader.py
import re
from typing import Dict, List, Tuple, TypeAlias

Coord: TypeAlias = Tuple[float, float]
CoordDict: TypeAlias = Dict[str, List[Coord]]

_BLOCK_RE = re.compile(
    r"coordinates(?P<name>Front|Overhead)_data\s*=\s*\{(?P<body>.*?)\}",
    re.IGNORECASE | re.DOTALL,
)
_PAIR_RE = re.compile(
    r"(?P<id>[0-9A-Za-z_]+)\s*:\s*\[\s*\(\s*(?P<x1>[-+0-9.eE]+)\s*,\s*(?P<y1>[-+0-9.eE]+)\s*\)\s*,\s*\(\s*(?P<x2>[-+0-9.eE]+)\s*,\s*(?P<y2>[-+0-9.eE]+)\s*\)\s*\]",
    re.IGNORECASE,
)

_FRONT_ALIASES = {"front", "front:", "[front]", "f:", "f"}
_RIGHT_ALIASES = {"right", "overhead", "overhead:", "right:", "[right]", "r:", "r"}
_NUM_RE = re.compile(r"[-+]?[\d.]+(?:e[-+]?\d+)?", re.IGNORECASE)


def _parse_block_dict(text: str, block_name: str) -> CoordDict:
    out: CoordDict = {}
    for m in _BLOCK_RE.finditer(text):
        name = m.group("name").lower()
        if (block_name == "front" and name != "front") or (block_name == "right" and name != "overhead"):
            continue
        body = m.group("body")
        for mm in _PAIR_RE.finditer(body):
            gid = str(mm.group("id"))
            x1, y1, x2, y2 = (float(mm.group("x1")), float(mm.group("y1")),
                              float(mm.group("x2")), float(mm.group("y2")))
            out[gid] = [(x1, y1), (x2, y2)]
    return out


def _split_blocks_by_headers(lines: List[str]):
    blocks = {"front": [], "right": []}
    current = None
    for raw in lines:
        s = raw.strip()
        low = s.lower()
        if low in _FRONT_ALIASES:
            current = "front"; continue
        if low in _RIGHT_ALIASES:
            current = "right"; continue
        if not s or current is None:
            continue
        blocks[current].append(s)
    return blocks


def _parse_section_lines(lines: List[str]) -> CoordDict:
    """Parse the first four numeric values of each section line as member endpoints."""
    out: CoordDict = {}
    idx = 0
    for ln in lines:
        nums = [float(x) for x in _NUM_RE.findall(ln)]
        if len(nums) < 4:
            continue
        x1, y1, x2, y2 = nums[:4]
        gid = f"{idx:05d}"
        out[gid] = [(x1, y1), (x2, y2)]
        idx += 1
    return out


def load_and_parse_data(filepath: str) -> Tuple[CoordDict, CoordDict]:
    """Load front and side member coordinate dictionaries from a drawing text file."""
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    front = _parse_block_dict(text, "front")
    right = _parse_block_dict(text, "right")

    if not front and not right:
        blocks = _split_blocks_by_headers(text.splitlines())
        front = _parse_section_lines(blocks.get("front", []))
        right = _parse_section_lines(blocks.get("right", []))

    return front, right

from typing import Dict, List, Tuple, Optional
import math
import statistics


# ----------------
# Basic 2D geometry helpers
# ----------------
def _len2(p1: Coord, p2: Coord) -> float:
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    return dx*dx + dy*dy


def _segment_length(p1: Coord, p2: Coord) -> float:
    return math.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1]))


def _numeric_member_id(member_id: object) -> float:
    try:
        return int(base_id(member_id))
    except (TypeError, ValueError):
        return float("inf")


def _member_sort_key(member_id: object):
    num_id = _numeric_member_id(member_id)
    if num_id == float("inf"):
        return (1, str(member_id))
    return (0, num_id)


def _is_main_rod_candidate(member_id: object) -> bool:
    """Return whether the base drawing ID is eligible to be a class-1 rod."""
    return base_id(member_id).endswith(("01", "02", "03"))


def _select_main_rods_by_geometry(
    coordinates_data: CoordDict,
    top_k: int,
    min_vertical_span_ratio: float = 0.8,
) -> List[str]:
    """Select full-height outer rods when drawing IDs do not follow convention.

    Main rods normally span almost the full drawing height and form the left
    and right envelope. Restricting the pool by vertical span first prevents
    shorter braces from winning; selecting the horizontal extremes then
    prevents full-height crossing diagonals from replacing the outer legs.
    """
    geometry: List[Tuple[str, float, float, float]] = []
    for member_id, endpoints in coordinates_data.items():
        if not isinstance(endpoints, (list, tuple)) or len(endpoints) != 2:
            continue
        p1, p2 = endpoints
        vertical_span = abs(float(p2[1]) - float(p1[1]))
        if vertical_span <= 1e-9:
            continue
        midpoint_x = (float(p1[0]) + float(p2[0])) / 2.0
        length = _segment_length(p1, p2)
        geometry.append((str(member_id), vertical_span, midpoint_x, length))

    if len(geometry) < top_k:
        return []

    max_vertical_span = max(item[1] for item in geometry)
    full_height = [
        item
        for item in geometry
        if item[1] >= max_vertical_span * min_vertical_span_ratio
    ]
    if len(full_height) < top_k:
        return []

    full_height.sort(key=lambda item: (item[2], -item[1], -item[3], _member_sort_key(item[0])))
    if top_k == 1:
        return [full_height[0][0]]

    selected = [full_height[0][0], full_height[-1][0]]
    if top_k > 2:
        selected_ids = set(selected)
        remaining = sorted(full_height[1:-1], key=lambda item: (-item[1], -item[3]))
        selected.extend(item[0] for item in remaining if item[0] not in selected_ids)
    return sorted(selected[:top_k], key=_member_sort_key)


def detect_main_rods_enhanced(coordinates_data: CoordDict, top_k: int = 2) -> List[str]:
    """
    Detect class-1/main rods using drawing IDs with a geometry fallback.

    The eligible candidates are ranked by length first, then fall back to the
    smallest eligible IDs. If fewer than two eligible IDs exist, or those rods
    are much shorter vertically than the drawing, full-height outer rods are
    selected geometrically. Duplicate-instance suffixes are ignored only when
    evaluating the drawing ID suffix.
    """
    if len(coordinates_data) < top_k:
        return []

    rod_items = []
    all_ids = []
    for rod_id, endpoints in coordinates_data.items():
        if (
            not _is_main_rod_candidate(rod_id)
            or not isinstance(endpoints, (list, tuple))
            or len(endpoints) != 2
        ):
            continue
        rod_key = str(rod_id)
        num_id = _numeric_member_id(rod_key)
        all_ids.append((rod_key, num_id))
        p1, p2 = endpoints
        rod_items.append((rod_key, num_id, _segment_length(p1, p2)))

    fallback_ids = _select_main_rods_by_geometry(coordinates_data, top_k)
    if len(rod_items) < top_k:
        return fallback_ids

    rod_items.sort(key=lambda item: item[2], reverse=True)
    candidates = [item[0] for item in rod_items[:top_k]]

    all_ids.sort(key=lambda item: (item[1], item[0]))
    min_two_ids = [item[0] for item in all_ids[:top_k]]
    if len(min_two_ids) < top_k:
        return fallback_ids

    candidates_set = set(candidates)
    min_two_set = set(min_two_ids)
    min_one = min_two_ids[0]

    if candidates_set == min_two_set:
        result = min_two_ids
    elif min_one in candidates_set:
        result = candidates
    else:
        result = min_two_ids

    result = sorted(result, key=_member_sort_key)
    max_vertical_span = max(
        abs(float(segment[1][1]) - float(segment[0][1]))
        for segment in coordinates_data.values()
        if isinstance(segment, (list, tuple)) and len(segment) == 2
    )
    selected_vertical_spans = [
        abs(float(coordinates_data[member_id][1][1]) - float(coordinates_data[member_id][0][1]))
        for member_id in result
    ]
    if (
        fallback_ids
        and selected_vertical_spans
        and min(selected_vertical_spans) < max_vertical_span * 0.8
    ):
        return fallback_ids
    return result

def clean_view(view: CoordDict, view_name: str, round_ndigits: Optional[int] = None) -> CoordDict:
    """Validate member endpoints, remove degenerate members, and optionally round coordinates."""
    out: CoordDict = {}
    for k, seg in view.items():
        if not isinstance(seg, (list, tuple)) or len(seg) != 2:
            continue
        p1 = (float(seg[0][0]), float(seg[0][1]))
        p2 = (float(seg[1][0]), float(seg[1][1]))
        if _len2(p1, p2) < 1e-12:
            continue
        if round_ndigits is not None:
            p1 = (round(p1[0], round_ndigits), round(p1[1], round_ndigits))
            p2 = (round(p2[0], round_ndigits), round(p2[1], round_ndigits))
        out[str(k)] = [p1, p2]
    return out


def remap_vertical_coordinates(
    view: CoordDict,
    source_height_span: Optional[Tuple[float, float]],
    target_height_span: Optional[Tuple[float, float]],
    source_support: Optional[CoordDict] = None,
    target_support: Optional[CoordDict] = None,
    support_tolerance: float = 35.0,
) -> CoordDict:
    """Map CAD heights and preserve attachments to adjusted support members.

    Secondary members are drawn against the original main legs.  When the
    main legs are refitted, a point on an original leg must be reprojected to
    that same leg at its remapped height; otherwise its X coordinate remains
    on the old profile and the front member protrudes outside the tower.
    """
    if source_height_span is None or target_height_span is None:
        return dict(view)

    source_low, source_high = source_height_span
    target_low, target_high = target_height_span
    source_height = float(source_high) - float(source_low)
    if abs(source_height) < 1e-9:
        return dict(view)

    source_support = source_support or {}
    target_support = target_support or {}

    def _distance_to_segment(point: Coord, start: Coord, end: Coord) -> float:
        px, py = point
        x1, y1 = start
        x2, y2 = end
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-12:
            return math.hypot(px - x1, py - y1)
        ratio = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
        return math.hypot(px - (x1 + ratio * dx), py - (y1 + ratio * dy))

    def _attached_support_id(point: Coord) -> Optional[str]:
        best_id = None
        best_distance = float("inf")
        for support_id, segment in source_support.items():
            if len(segment) != 2:
                continue
            distance = _distance_to_segment(point, segment[0], segment[1])
            if distance < best_distance:
                best_id = str(support_id)
                best_distance = distance
        return best_id if best_distance <= support_tolerance else None

    def _x_on_support(segment: List[Coord], y_value: float) -> Optional[float]:
        if len(segment) != 2:
            return None
        (x1, y1), (x2, y2) = segment
        dy = y2 - y1
        if abs(dy) < 1e-9:
            return None
        return x1 + (y_value - y1) * (x2 - x1) / dy

    out: CoordDict = {}
    for member_id, segment in view.items():
        points = []
        for x_value, y_value in segment:
            source_point = (float(x_value), float(y_value))
            level = (float(y_value) - float(source_low)) / source_height
            # Small CAD drafting offsets may put an endpoint slightly beyond a
            # support end.  Keep it on the physical tower interval.
            level = min(1.0, max(0.0, level))
            target_y = float(target_low) + level * (float(target_high) - float(target_low))
            target_x = float(x_value)
            support_id = _attached_support_id(source_point)
            target_segment = target_support.get(support_id) if support_id else None
            if target_segment:
                projected_x = _x_on_support(target_segment, target_y)
                if projected_x is not None:
                    target_x = projected_x
            points.append((target_x, target_y))
        out[str(member_id)] = points
    return out

# ----------------
# Support and horizontal-member detection
# ----------------
def find_supports(view: CoordDict) -> CoordDict:
    """Return the two detected class-1 support members for a view."""
    support_ids = detect_main_rods_enhanced(view, top_k=2)
    out = {}
    for sid in support_ids:
        seg = view.get(sid)
        if not seg:
            continue
        out[str(sid)] = [(float(seg[0][0]), float(seg[0][1])),
                         (float(seg[1][0]), float(seg[1][1]))]
    return out

def find_horizontals(view: CoordDict, tol_y: float, exclude_support: bool = True) -> CoordDict:
    """Return near-horizontal members, optionally excluding detected non-horizontal supports."""
    support_ids = set()
    if exclude_support:
        for sid in detect_main_rods_enhanced(view, top_k=2):
            seg = view.get(sid)
            if not seg:
                continue
            (_, y1), (_, y2) = seg
            if abs(float(y1) - float(y2)) > float(tol_y):
                support_ids.add(str(sid))

    out: CoordDict = {}
    for k, seg in view.items():
        if str(k) in support_ids:
            continue
        (x1, y1), (x2, y2) = seg
        if abs(float(y1) - float(y2)) <= float(tol_y):
            out[str(k)] = [(float(x1), float(y1)), (float(x2), float(y2))]
    return out

# ----------------
# Support-line models
# ----------------
class Line:
    __slots__ = ("k","b","vertical_x","ymin","ymax","id")
    def __init__(self, p1: Coord, p2: Coord, gid: str):
        x1,y1 = float(p1[0]), float(p1[1])
        x2,y2 = float(p2[0]), float(p2[1])
        self.id = gid
        self.ymin, self.ymax = (min(y1,y2), max(y1,y2))
        if abs(x1-x2) < 1e-12:
            self.k = None
            self.b = None
            self.vertical_x = x1
        else:
            self.k = (y2-y1)/(x2-x1)
            self.b = y1 - self.k*x1
            self.vertical_x = None

    def x_at(self, y: float) -> float:
        y = float(y)
        if self.vertical_x is not None:
            return float(self.vertical_x)
        # y = kx + b -> x = (y - b) / k
        if self.k is None or abs(self.k) < 1e-12:
            return float("nan")
        return (y - float(self.b or 0.0)) / self.k

    def x_mid(self) -> float:
        ym = (self.ymin + self.ymax)/2.0
        return self.x_at(ym)

def build_support_models(view_support: CoordDict) -> List[Line]:
    models: List[Line] = []
    for gid, (p1,p2) in view_support.items():
        ln = Line(p1,p2,str(gid))
        models.append(ln)
    models.sort(key=lambda L: L.x_mid())
    return models

def _extreme_x_at(models: List[Line], y: float) -> Tuple[float, float]:
    """Return the leftmost and rightmost support intersections at a given height."""
    xs = [m.x_at(y) for m in models]
    if not xs:
        return (0.0, 0.0)
    xs = [x for x in xs if math.isfinite(x)]
    if not xs:
        return (0.0, 0.0)
    return (min(xs), max(xs))

# ----------------
# Cross-view horizontal alignment
# ----------------
def correct_paired_horizontals(front_horizontal: CoordDict,
                               right_horizontal: CoordDict,
                               front_support_models: List[Line],
                               right_support_models: List[Line],
                               round_to_int: bool = False,
                               front_height_span: Optional[Tuple[float, float]] = None,
                               right_height_span: Optional[Tuple[float, float]] = None,
                               target_height_span: Optional[Tuple[float, float]] = None):
    """Match corresponding horizontal members across views and project them onto supports."""
    if not front_horizontal and not right_horizontal:
        return front_horizontal, right_horizontal

    def _to_list(hh: CoordDict):
        return [(k, (seg[0][1]+seg[1][1])/2.0) for k, seg in hh.items()]

    fl = _to_list(front_horizontal)
    rl = _to_list(right_horizontal)

    def _vertical_span(models: List[Line]):
        values = [value for model in models for value in (model.ymin, model.ymax)]
        if not values:
            return None
        low, high = min(values), max(values)
        return (low, high) if high - low >= 1e-9 else None

    def _relative_height(y: float, span):
        if span is None:
            return None
        low, high = span
        return (float(y) - low) / (high - low)

    matched_pairs = []
    # Support slopes are unified before this function runs.  Their resulting
    # common bounds cannot be used to compare CAD levels from two views with
    # different original vertical scales, otherwise corresponding members such
    # as 208/209 are rejected as being on different layers.
    front_span = front_height_span or _vertical_span(front_support_models)
    right_span = right_height_span or _vertical_span(right_support_models)
    if front_span and right_span:
        candidates = []
        for kf, yf in fl:
            front_level = _relative_height(yf, front_span)
            for kr, yr in rl:
                right_level = _relative_height(yr, right_span)
                candidates.append((abs(front_level - right_level), str(kf), yf, str(kr), yr))

        used_front = set()
        used_right = set()
        max_relative_height_gap = 0.04
        for gap, kf, yf, kr, yr in sorted(candidates):
            if gap > max_relative_height_gap:
                break
            if kf in used_front or kr in used_right:
                continue
            used_front.add(kf)
            used_right.add(kr)
            matched_pairs.append((kf, yf, kr, yr))

    for kf, yf, kr, yr in matched_pairs:
        if target_height_span and front_span and right_span:
            front_level = _relative_height(yf, front_span)
            right_level = _relative_height(yr, right_span)
            level = min(1.0, max(0.0, (front_level + right_level) / 2.0))
            y = target_height_span[0] + level * (target_height_span[1] - target_height_span[0])
        else:
            y = (yf + yr)/2.0

        Lf, Rf = _extreme_x_at(front_support_models, y)
        Lr, Rr = _extreme_x_at(right_support_models, y)

        if round_to_int:
            y, Lf, Rf, Lr, Rr = round(y), round(Lf), round(Rf), round(Lr), round(Rr)

        front_horizontal[str(kf)] = [(Lf, y), (Rf, y)]
        right_horizontal[str(kr)] = [(Lr, y), (Rr, y)]

    def _project_rest(hh: CoordDict, models: List[Line], used_keys: set):
        for k, seg in list(hh.items()):
            if k in used_keys:
                continue
            y = (seg[0][1]+seg[1][1])/2.0
            L, R = _extreme_x_at(models, y)
            if round_to_int:
                y, L, R = round(y), round(L), round(R)
            hh[str(k)] = [(L, y), (R, y)]
    _project_rest(front_horizontal, front_support_models, {kf for kf, _, _, _ in matched_pairs})
    _project_rest(right_horizontal, right_support_models, {kr for _, _, kr, _ in matched_pairs})

    return front_horizontal, right_horizontal

# ================================
# Cross-view support fitting
# ================================

def match_support_slopes(front_support: CoordDict,
                                     right_support: CoordDict,
                                     strategy: str = "mean",
                                     unify_bounds: bool = True,
                                     round_to_int: bool = False,
                                     y_round_to_int: bool = False) -> Tuple[CoordDict, CoordDict]:
    """Fit front and side support slopes to a common profile while preserving their anchors."""
    if not front_support or not right_support:
        return front_support, right_support

    ys = [p[1] for seg in front_support.values() for p in seg] + \
         [p[1] for seg in right_support.values() for p in seg]
    if not ys:
        return front_support, right_support

    gmin, gmax = (min(ys), max(ys))

    F_models = build_support_models(front_support)
    R_models = build_support_models(right_support)
    if len(F_models) == 0 or len(R_models) == 0:
        return front_support, right_support

    def _extremes(models):
        if len(models) == 1:
            return models[0], models[0]
        return models[0], models[-1]

    F_left, F_right = _extremes(F_models)
    R_left, R_right = _extremes(R_models)

    def _target_k(k1, k2):
        if strategy == "front": return k1
        if strategy == "right": return k2
        if k1 is None and k2 is None:
            return None
        if k1 is None:
            return k2
        if k2 is None:
            return k1
        return (k1 + k2) / 2.0

    kL = _target_k(F_left.k,  R_left.k)
    kR = _target_k(F_right.k, R_right.k)

    def _rebuild(line: Line, k_new):
        y_anchor = (gmin + gmax) / 2.0
        x_anchor = line.x_at(y_anchor)
        if k_new is None:
            x_top = x_anchor
            x_bot = x_anchor
        else:
            kk = float(k_new)
            if abs(kk) < 1e-12:
                kk = 1e-12 if kk >= 0 else -1e-12
            x_top = x_anchor + (gmin - y_anchor) / kk
            x_bot = x_anchor + (gmax - y_anchor) / kk
        y1, y2 = (gmin if unify_bounds else line.ymin, gmax if unify_bounds else line.ymax)
        if y_round_to_int:
            y1, y2 = round(y1), round(y2)
        if round_to_int:
            x_top, x_bot = round(x_top), round(x_bot)
        return [(x_top, y1), (x_bot, y2)]

    front_support = dict(front_support)
    right_support = dict(right_support)
    front_support[F_left.id]  = _rebuild(F_left,  kL)
    right_support[R_left.id]  = _rebuild(R_left,  kL)
    front_support[F_right.id] = _rebuild(F_right, kR)
    right_support[R_right.id] = _rebuild(R_right, kR)
    return front_support, right_support


def _highest_horizontal_key(view_h: CoordDict) -> Optional[str]:
    if not view_h: return None
    items = sorted(view_h.items(), key=lambda kv: (kv[1][0][1]+kv[1][1][1])/2.0)
    return items[0][0] if items else None

def _xset_at(models: List[Line], y: float) -> List[float]:
    xs = [m.x_at(y) for m in models]
    xs = [x for x in xs if math.isfinite(x)]
    xs.sort()
    return xs

def _best_pair_near_width(Xs: List[float], target: float) -> Optional[Tuple[float, float]]:
    if len(Xs) < 2: return None
    C = (min(Xs) + max(Xs)) / 2.0
    best = None
    for i in range(len(Xs)-1):
        for j in range(i+1, len(Xs)):
            L, R = Xs[i], Xs[j]
            w = R - L
            err = abs(w - target)
            ctr = (L + R) / 2.0
            cen = abs(ctr - C)
            cand = (err, cen, L, R)
            if best is None or cand < best:
                best = cand
    if best is None:
        return 0.0, 0.0
    _, _, L, R = best
    return L, R

def plan_top_span(
    front_support_models: List[Line], right_support_models: List[Line],
    front_horizontal: CoordDict, right_horizontal: CoordDict,
    length_mode: str = "min",            # "min" | "mean" | "front" | "right"
    pair_mode: str = "extreme"
):
    """Determine a common top elevation and matching top widths for both views."""
    if not front_support_models or not right_support_models:
        return None

    def _highest_y(view_h: CoordDict):
        if not view_h: return None
        k = _highest_horizontal_key(view_h)
        if k is None: return None
        seg = view_h[k]
        return (seg[0][1] + seg[1][1]) / 2.0

    y_candidates = []
    yf = _highest_y(front_horizontal)
    yr = _highest_y(right_horizontal)
    if yf is not None: y_candidates.append(yf)
    if yr is not None: y_candidates.append(yr)
    if not y_candidates:
        return None
    y_top = sum(y_candidates) / len(y_candidates)

    def _extremes(models):
        if len(models) == 1:
            return models[0], models[0]
        return models[0], models[-1]

    def _x_at(line_model: Line, y: float) -> float:
        if getattr(line_model, "vertical_x", None) is not None:
            return float(line_model.vertical_x or 0.0)  # type: ignore
        k = getattr(line_model, "k", None)
        b = getattr(line_model, "b", None)
        if k is None or abs(k) < 1e-12:
            return (getattr(line_model, "xmin", 0.0) + getattr(line_model, "xmax", 0.0)) / 2.0
        return (y - float(b or 0.0)) / k  # type: ignore

    if pair_mode == "extreme":
        F_left, F_right = _extremes(front_support_models)
        R_left, R_right = _extremes(right_support_models)
        Lf0, Rf0 = _x_at(F_left, y_top), _x_at(F_right, y_top)
        Lr0, Rr0 = _x_at(R_left, y_top), _x_at(R_right, y_top)
        if Lf0 > Rf0: Lf0, Rf0 = Rf0, Lf0
        if Lr0 > Rr0: Lr0, Rr0 = Rr0, Lr0
        Wf0, Wr0 = (Rf0 - Lf0), (Rr0 - Lr0)

        if length_mode == "front":
            Wt = Wf0
        elif length_mode == "right":
            Wt = Wr0
        elif length_mode == "mean":
            Wt = (Wf0 + Wr0) / 2.0
        else:
            Wt = min(Wf0, Wr0)

        Cf, Cr = (Lf0 + Rf0) / 2.0, (Lr0 + Rr0) / 2.0
        Lf, Rf = Cf - Wt / 2.0, Cf + Wt / 2.0
        Lr, Rr = Cr - Wt / 2.0, Cr + Wt / 2.0

    else:
        Xf = _xset_at(front_support_models, y_top)
        Xr = _xset_at(right_support_models, y_top)
        if len(Xf) < 2 or len(Xr) < 2:
            Lf, Rf = (min(Xf), max(Xf)) if len(Xf) >= 2 else (None, None)
            Lr, Rr = (min(Xr), max(Xr)) if len(Xr) >= 2 else (None, None)
        else:
            Wf = max(Xf) - min(Xf)
            Wr = max(Xr) - min(Xr)
            if length_mode == "front":
                Wt = Wf
            elif length_mode == "right":
                Wt = Wr
            elif length_mode == "mean":
                Wt = (Wf + Wr) / 2.0
            else:
                Wt = min(Wf, Wr)
            LfRf = _best_pair_near_width(Xf, Wt)
            LrRr = _best_pair_near_width(Xr, Wt)
            Lf, Rf = LfRf if LfRf else (min(Xf), max(Xf))
            Lr, Rr = LrRr if LrRr else (min(Xr), max(Xr))

    return {
        "y_top": y_top,
        "Lf": Lf, "Rf": Rf,
        "Lr": Lr, "Rr": Rr,
        "front_top_key": _highest_horizontal_key(front_horizontal),
        "right_top_key": _highest_horizontal_key(right_horizontal),
    }


def expand_to_top_span(
    front_support: CoordDict, right_support: CoordDict,
    front_support_models: List[Line], right_support_models: List[Line],
    y_top: float, Lf: float, Rf: float, Lr: float, Rr: float,
    unify_bounds: bool = True,
    round_to_int: bool = False,
    y_round_to_int: bool = False
) -> Tuple[CoordDict, CoordDict]:
    """Extend supports so that their top intersections match the planned top span."""
    if not front_support or not right_support:
        return front_support, right_support

    ys = [p[1] for seg in front_support.values() for p in seg] + \
         [p[1] for seg in right_support.values() for p in seg]
    gmin, gmax = (min(ys), max(ys)) if ys else (0.0, 1.0)

    def _extremes(models):
        if len(models) == 1: return models[0], models[0]
        return models[0], models[-1]

    F_left, F_right = _extremes(front_support_models)
    R_left, R_right = _extremes(right_support_models)

    def _rebuild_through(line_model: Line, x_at_y_top: float):
        if line_model.vertical_x is not None:
            x_top = x_at_y_top
            x_bot = x_at_y_top
        else:
            kk = line_model.k
            if kk is None or abs(kk) < 1e-12:
                kk = 1e-12 if (kk or 0.0) >= 0 else -1e-12
            def x_at(y): return x_at_y_top + (y - y_top) / kk
            x_top = x_at(gmin if unify_bounds else line_model.ymin)
            x_bot = x_at(gmax if unify_bounds else line_model.ymax)
        y1 = (gmin if unify_bounds else line_model.ymin)
        y2 = (gmax if unify_bounds else line_model.ymax)
        if y_round_to_int:
            y1, y2 = round(y1), round(y2)
        if round_to_int:
            x_top, x_bot = round(x_top), round(x_bot)
        return [(x_top, y1), (x_bot, y2)]

    front_support = dict(front_support)
    right_support = dict(right_support)

    if Lf is not None and Rf is not None:
        front_support[F_left.id]  = _rebuild_through(F_left,  Lf)
        front_support[F_right.id] = _rebuild_through(F_right, Rf)
    if Lr is not None and Rr is not None:
        right_support[R_left.id]  = _rebuild_through(R_left,  Lr)
        right_support[R_right.id] = _rebuild_through(R_right, Rr)

    return front_support, right_support


def correct_horizontals(
    front_support_models: List[Line], right_support_models: List[Line],
    front_horizontal: CoordDict, right_horizontal: CoordDict,
    skip_front_keys: Optional[set] = None,
    skip_right_keys: Optional[set] = None,
    round_to_int: bool = False,
    front_height_span: Optional[Tuple[float, float]] = None,
    right_height_span: Optional[Tuple[float, float]] = None,
    target_height_span: Optional[Tuple[float, float]] = None,
) -> Tuple[CoordDict, CoordDict]:
    """Correct unprotected horizontal members after paired horizontal correction."""
    skip_front_keys = skip_front_keys or set()
    skip_right_keys = skip_right_keys or set()

    fh_keep = {k:v for k,v in front_horizontal.items() if k in skip_front_keys}
    rh_keep = {k:v for k,v in right_horizontal.items() if k in skip_right_keys}
    fh_rest = {k:v for k,v in front_horizontal.items() if k not in skip_front_keys}
    rh_rest = {k:v for k,v in right_horizontal.items() if k not in skip_right_keys}

    fh_rest2, rh_rest2 = correct_paired_horizontals(
        fh_rest,
        rh_rest,
        front_support_models,
        right_support_models,
        round_to_int=round_to_int,
        front_height_span=front_height_span,
        right_height_span=right_height_span,
        target_height_span=target_height_span,
    )
    fh_rest2.update(fh_keep)
    rh_rest2.update(rh_keep)
    return fh_rest2, rh_rest2



def enforce_symmetry(support_orig: CoordDict,
                             horizontal_orig: CoordDict) -> Tuple[CoordDict, CoordDict]:
    """Enforce bilateral symmetry for supports and project horizontal members to the result."""
    if not support_orig and not horizontal_orig:
        return support_orig, horizontal_orig

    print("  - 对称校正已完成。")

    all_x = []
    for seg in list(support_orig.values()) + list(horizontal_orig.values()):
        all_x.extend([p[0] for p in seg])
    if not all_x:
        return support_orig, horizontal_orig
    x_center = (min(all_x) + max(all_x)) / 2.0

    support = dict(support_orig)
    horizontal = dict(horizontal_orig)

    s_models = build_support_models(support)
    if not s_models:
        return support, horizontal

    pairs = {}
    unpaired_ids = []
    left, right = 0, len(s_models) - 1
    while left < right:
        pairs[s_models[left].id] = s_models[right].id
        pairs[s_models[right].id] = s_models[left].id
        left += 1
        right -= 1
    if left == right:
        unpaired_ids.append(s_models[left].id)

    processed_supports = set()
    for l_id, r_id in pairs.items():
        if l_id in processed_supports or r_id in processed_supports:
            continue

        l_model = next(m for m in s_models if m.id == l_id)
        r_model = next(m for m in s_models if m.id == r_id)

        if l_model.k is not None and r_model.k is not None:
            avg_k_mag = (abs(l_model.k) + abs(r_model.k)) / 2.0
            new_lk, new_rk = -avg_k_mag, avg_k_mag
        else:
            new_lk, new_rk = l_model.k, r_model.k

        l_dist = x_center - l_model.x_mid()
        r_dist = r_model.x_mid() - x_center
        avg_dist = (l_dist + r_dist) / 2.0
        new_l_x_mid, new_r_x_mid = x_center - avg_dist, x_center + avg_dist

        for mid, model, new_k, new_x_mid in [(l_id, l_model, new_lk, new_l_x_mid),
                                             (r_id, r_model, new_rk, new_r_x_mid)]:
            p1_old, p2_old = support[mid]
            y_min, y_max = min(p1_old[1], p2_old[1]), max(p1_old[1], p2_old[1])
            y_mid = (y_min + y_max) / 2.0

            if new_k is None or abs(new_k) < 1e-9:
                x_at_min, x_at_max = new_x_mid, new_x_mid
            else:
                x_at_min = new_x_mid + (y_min - y_mid) / new_k
                x_at_max = new_x_mid + (y_max - y_mid) / new_k
            support[mid] = [(x_at_min, y_min), (x_at_max, y_max)]

        processed_supports.add(l_id)
        processed_supports.add(r_id)

    for mid_id in unpaired_ids:
        p1, p2 = support[mid_id]
        y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
        support[mid_id] = [(x_center, y_min), (x_center, y_max)]

    new_s_models = build_support_models(support)
    for h_id, h_seg in horizontal.items():
        p1, p2 = h_seg
        avg_y = (p1[1] + p2[1]) / 2.0

        xs_at_y = [m.x_at(avg_y) for m in new_s_models]
        xs_at_y = [x for x in xs_at_y if math.isfinite(x)]
        if not xs_at_y: continue

        x_left_support = min(xs_at_y)
        x_right_support = max(xs_at_y)

        horizontal[h_id] = [(x_left_support, avg_y), (x_right_support, avg_y)]

    return support, horizontal


# ================================
# Top-span alignment
# ================================
def align_to_top(support: CoordDict,
                                     horizontal: CoordDict) -> CoordDict:
    """Align the support system to the selected top horizontal member."""
    if not support or not horizontal:
        return support

    top_y, top_key = None, None
    for hid, seg in horizontal.items():
        y_mean = (seg[0][1] + seg[1][1]) / 2.0
        if top_y is None or y_mean < top_y:
            top_y, top_key = y_mean, hid

    if top_key is None:
        return support

    y_top_target = (horizontal[top_key][0][1] + horizontal[top_key][1][1]) / 2.0

    new_support = {}
    for gid, seg in support.items():
        p1, p2 = seg

        if p1[1] > p2[1]:
            p_bottom, p_top = p1, p2
        else:
            p_bottom, p_top = p2, p1

        line = Line(p1, p2, str(gid))

        if line.vertical_x is not None:
            new_top_x = line.vertical_x
        else:
            new_top_x = line.x_at(y_top_target)

        p_top_new = (new_top_x, y_top_target)

        new_support[gid] = [p_top_new, p_bottom]

    return new_support

from typing import Dict, List, Tuple, Optional
import math



def _mid(ptA: Coord, ptB: Coord) -> Tuple[float,float]:
    return ((float(ptA[0]) + float(ptB[0]))/2.0, (float(ptA[1]) + float(ptB[1]))/2.0)

def _clip(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))

def _select_top_horizontal(view_h: CoordDict, axis: str = "x") -> Optional[Tuple[Coord, Coord]]:
    """Select the horizontal member nearest the top of a view."""
    if not view_h:
        return None
    items = list(view_h.items())
    items.sort(key=lambda kv: (kv[1][0][1] + kv[1][1][1]) / 2.0)
    seg = items[0][1]
    (xa, za), (xb, zb) = (seg[0], seg[1])
    if axis == "x":
        return ( (xa,za), (xb,zb) ) if xa <= xb else ( (xb,zb), (xa,za) )
    else:  # axis == 'y'
        ya, yb = xa, xb
        if ya <= yb:
            return ( (xa,za), (xb,zb) )
        else:
            return ( (xb,zb), (xa,za) )

def _bottom_from_support(view_support: CoordDict, axis: str = "x") -> Optional[Tuple[Coord, Coord]]:
    """Derive the front-view base endpoints from supports and horizontal members."""
    if not view_support:
        return None
    bottom_pts: List[Coord] = []
    for seg in view_support.values():
        (x1,z1),(x2,z2) = seg
        if float(z1) >= float(z2):
            bottom_pts.append((float(x1), float(z1)))
        else:
            bottom_pts.append((float(x2), float(z2)))
    if len(bottom_pts) < 2:
        return None
    if axis == "x":
        bottom_pts.sort(key=lambda p: p[0])  # x
    else:
        bottom_pts.sort(key=lambda p: p[0])
    left, right = bottom_pts[0], bottom_pts[-1]
    return (left, right)



def extract_bases_front(front_support: CoordDict, front_horizontal: CoordDict):
    """Extract front-view reconstruction bases and reference geometry."""
    bottom = _bottom_from_support(front_support, axis="x")
    if bottom is None:
        return None
    (x4, z4), (x3, z3) = bottom
    
    bottom_center_x = (x3 + x4) / 2.0
    bottom_width = abs(x3 - x4)
    
    candidates = []
    for seg in front_support.values():
        (px1, pz1), (px2, pz2) = seg
        top_pt = (px1, pz1) if pz1 < pz2 else (px2, pz2)
        candidates.append(top_pt)
            
    if not candidates:
        return None

    valid_tops = []
    threshold = (bottom_width / 2.0) * 2.5
    
    for pt in candidates:
        dist = abs(pt[0] - bottom_center_x)
        if dist < threshold:
            valid_tops.append(pt)
            
    if len(valid_tops) < 2:
        valid_tops = candidates

    valid_tops.sort(key=lambda p: (p[1], p[0]))
    
    if len(valid_tops) < 2:
        return None
        
    t1, t2 = valid_tops[0], valid_tops[1]
    
    if t1[0] > t2[0]:
        t1, t2 = t2, t1
        
    (x1, z1), (x2, z2) = t1, t2

    # print(f"DEBUG: Bottom Center={bottom_center_x}, Filter Threshold={threshold}")
    # print(f"DEBUG: Rejected pts={[p for p in candidates if p not in valid_tops]}")
    # print(f"DEBUG: Selected Top={t1, t2}")

    return ((float(x1), float(z1)), (float(x2), float(z2)),
            (float(x3), float(z3)), (float(x4), float(z4)))


def extract_bases_side(right_support: CoordDict, right_horizontal: CoordDict):
    """Extract side-view reconstruction bases and reference geometry."""
    bottom = _bottom_from_support(right_support, axis="y")
    if bottom is None:
        return None
    (y7, z7), (y8, z8) = bottom
    
    bottom_center_y = (y7 + y8) / 2.0
    bottom_width = abs(y8 - y7)

    candidates = []
    for seg in right_support.values():
        (py1, pz1), (py2, pz2) = seg
        top_pt = (py1, pz1) if pz1 < pz2 else (py2, pz2)
        candidates.append(top_pt)
            
    if not candidates:
        return None

    valid_tops = []
    threshold = (bottom_width / 2.0) * 2.5 
    
    for pt in candidates:
        dist = abs(pt[0] - bottom_center_y)
        if dist < threshold:
            valid_tops.append(pt)
            
    if len(valid_tops) < 2:
        valid_tops = candidates

    valid_tops.sort(key=lambda p: (p[1], p[0]))
    
    if len(valid_tops) < 2:
        return None
        
    t1, t2 = valid_tops[0], valid_tops[1]
    
    if t1[0] > t2[0]:
        t1, t2 = t2, t1
        
    (y5, z5), (y6, z6) = t1, t2

    return ((float(y5), float(z5)), (float(y6), float(z6)),
            (float(y7), float(z7)), (float(y8), float(z8)))


def compute_heights_and_angles(front_bases, side_bases):
    """Calculate reconstruction heights and tilt angles from both base systems."""
    (x1,z1),(x2,z2),(x3,z3),(x4,z4) = front_bases
    (y5,z5),(y6,z6),(y7,z7),(y8,z8) = side_bases

    z_topF = (z1 + z2)/2.0
    z_topS = (z5 + z6)/2.0

    xm_top, zm_top   = (x1 + x2)/2.0, (z1 + z2)/2.0
    xm_bottom, zm_bottom = (x3 + x4)/2.0, (z3 + z4)/2.0
    h_front = math.hypot(xm_top - xm_bottom, zm_top - zm_bottom)

    ym_top, zm_top_s   = (y5 + y6)/2.0, (z5 + z6)/2.0
    ym_bottom, zm_bottom_s = (y7 + y8)/2.0, (z7 + z8)/2.0
    h_side = math.hypot(ym_top - ym_bottom, zm_top_s - zm_bottom_s)

    eps = 1e-9
    h_front = max(h_front, eps)
    h_side  = max(h_side, eps)

    cos_theta = _clip( (abs(y8 - y7) - abs(y6 - y5)) / (2.0 * h_front) )
    theta = math.acos(cos_theta)

    cos_delta = _clip( (abs(x3 - x4) - abs(x1 - x2)) / (2.0 * h_side) )
    delta = math.acos(cos_delta)

    return (z_topF, z_topS, h_front, h_side, theta, delta)

def _norm_front_point(x: float, z: float, x4: float, z_topF: float) -> Tuple[float,float]:
    xf2 = float(x) - float(x4)
    z1p = float(z) - float(z_topF)
    return xf2, z1p

def _norm_side_point(y: float, z: float, y7: float, z_topS: float) -> Tuple[float,float]:
    ys2 = float(y) - float(y7)
    z2p = float(z) - float(z_topS)
    return ys2, z2p

def sort_bases(bases):

    bottom_left, bottom_right, top_a, top_b = bases

    if top_a[0] < top_b[0]:
        top_left = top_a
        top_right = top_b
    else:
        top_left = top_b
        top_right = top_a

    if bottom_left[0] > bottom_right[0]:
        bottom_left, bottom_right = bottom_right, bottom_left

    return bottom_left, bottom_right, top_left, top_right

def reconstruct3d_front(front_total: CoordDict,
                        front_support: CoordDict,
                        front_horizontal: CoordDict,
                        side_bases) -> Dict[str, List[Point3D]]:
    """Reconstruct front-view members as three-dimensional coordinates."""
    fbases = extract_bases_front(front_support, front_horizontal)
    if not fbases or not side_bases: 
        return {}
    
    (x1, z1), (x2, z2), (x3, z3), (x4, z4) = fbases
    (y5, z5), (y6, z6), (y7, z7), (y8, z8) = side_bases

    
    Cx = (x3 + x4) / 2.0
    
    W_top_s = abs(y6 - y5) / 2.0
    W_bot_s = abs(y8 - y7) / 2.0
    
    # Height = Z_bot - Z_top
    Z_top = (z1 + z2) / 2.0
    Z_bot = (z3 + z4) / 2.0
    Height = Z_bot - Z_top
    
    out: Dict[str, List[Point3D]] = {}
    
    for gid, seg in front_total.items():
        pts: List[Point3D] = []
        for (x, z) in seg:
            X = x - Cx
            
            if abs(Height) < 1e-4:
                current_depth = W_top_s
            else:
                ratio = (z - Z_top) / Height
                current_depth = W_top_s + ratio * (W_bot_s - W_top_s)
            
            Y = -current_depth
            
            Z = z
            # print("DEBUG front seg:", gid, seg)
            pts.append((X, Y, Z))
        out[f"F_{gid}"] = pts
    
    return out


def reconstruct3d_right(right_total: CoordDict,
                        right_support: CoordDict,
                        right_horizontal: CoordDict,
                        front_bases) -> Dict[str, List[Point3D]]:
    """Reconstruct side-view members as three-dimensional coordinates."""
    sbases = extract_bases_side(right_support, right_horizontal)
    if not sbases or not front_bases: 
        return {}
    
    (x1, z1), (x2, z2), (x3, z3), (x4, z4) = front_bases
    (y5, z5), (y6, z6), (y7, z7), (y8, z8) = sbases


    Cy = (y7 + y8) / 2.0
    
    W_top_f = abs(x2 - x1) / 2.0
    W_bot_f = abs(x3 - x4) / 2.0
    
    # Z_top = z1
    # Z_bot = z3
    # Height = Z_bot - Z_top
    Z_top = (z5 + z6) / 2.0
    Z_bot = (z7 + z8) / 2.0
    Height = Z_bot - Z_top
    
    out: Dict[str, List[Point3D]] = {}
    
    for gid, seg in right_total.items():
        pts: List[Point3D] = []
        for (y, z) in seg:
            Y = y - Cy
            
            if abs(Height) < 1e-4:
                current_width = W_top_f
            else:
                ratio = (z - Z_top) / Height
                current_width = W_top_f + ratio * (W_bot_f - W_top_f)
            
            X = current_width
            
            Z = z

            # print("DEBUG right seg:", gid, seg)
            
            pts.append((X, Y, Z))
        out[f"R_{gid}"] = pts
    return out






import math
import numpy as np
from typing import Dict, List, Tuple, Set

AllModelsData: TypeAlias = Dict[str, Dict[str, Any]]


def _find_splicing_points(f3d: Model3DData, support_keys: Set[str], mode: str) -> List[Point3D]:
    """Find support endpoints used to splice adjacent reconstructed models."""
    support_endpoints = []
    for gid, seg in f3d.items():
        original_id = gid.replace("F_", "")
        if original_id in support_keys:
            support_endpoints.extend(seg)

    if not support_endpoints:
        raise ValueError("support endpoints are unavailable")

    reverse_sort = (mode == 'bottom')
    support_endpoints.sort(key=lambda p: p[2], reverse=reverse_sort)

    if len(support_endpoints) < 2:
        raise ValueError("not enough support endpoints for splicing")

    z_ref = support_endpoints[0][2]
    z_tolerance = 1.0
    
    same_z_layer = [p for p in support_endpoints if abs(p[2] - z_ref) < z_tolerance]
    
    if len(same_z_layer) < 2:
        same_z_layer = support_endpoints[:2]
    
    same_z_layer.sort(key=lambda p: p[0])
    
    left_point = same_z_layer[0]
    right_point = same_z_layer[-1]
    
    print(f"  - 拼接点({mode})：左={left_point}，右={right_point}")
    
    return [left_point, right_point]


def get_rotation_matrix(v1, v2):
    """Build a rotation matrix that maps one direction vector onto another."""
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    if np.allclose(v1, v2):
        return np.identity(3)
    if np.allclose(v1, -v2):
        return -np.identity(3)

    cross_prod = np.cross(v1, v2)
    dot_prod = np.dot(v1, v2)

    s = np.linalg.norm(cross_prod)
    c = dot_prod

    vx = np.array([
        [0, -cross_prod[2], cross_prod[1]],
        [cross_prod[2], 0, -cross_prod[0]],
        [-cross_prod[1], cross_prod[0], 0]
    ])

    rotation_matrix = np.identity(3) + vx + vx.dot(vx) * ((1 - c) / (s ** 2))
    return rotation_matrix


def _align_and_transform_model(model_data: Dict[str, Model3DData],
                               source_p1: Point3D, source_p2: Point3D,
                               target_p1: Point3D, target_p2: Point3D):
    """Translate and rotate a model so its selected splice points align with a target model."""
    A1 = np.array(source_p1)
    A2 = np.array(source_p2)
    B1 = np.array(target_p1)
    B2 = np.array(target_p2)

    dist_A = np.linalg.norm(A2 - A1)
    dist_B = np.linalg.norm(B2 - B1)
    if dist_A < 1e-9:
        raise ValueError("source connection points are too close")
    scale = dist_B / dist_A
    print(f"  - 拼接对齐缩放系数：{scale:.4f}")

    vec_A = A2 - A1
    vec_B = B2 - B1
    rotation_matrix = get_rotation_matrix(vec_A, vec_B)
    print("  - 拼接旋转参数已计算。")

    for view_data in model_data.values():
        for gid in view_data:
            transformed_seg = []
            for p_tuple in view_data[gid]:
                P = np.array(p_tuple)
                P_relative = P - A1
                P_scaled = P_relative * scale
                P_rotated = rotation_matrix.dot(P_scaled)
                P_new = P_rotated + B1
                transformed_seg.append(tuple(P_new))
            view_data[gid] = transformed_seg
    print("  - 拼接模型变换已完成。")


def splice_models(
    all_models_data: AllModelsData,
    auto_base_index: Optional[int] = None,
) -> Tuple[Model3DData, Model3DData]:
    """
    Splice all tower-body models into one shared 3D coordinate space.
    """
    # Keep this order consistent with dual_view_processor's input order.  A
    # plain string sort would place "10" before "9" and reverse their stack.
    model_names = sorted(all_models_data.keys(), key=_model_sort_key)
    print("\n" + "=" * 50)
    print("Model splicing")
    print("=" * 50)
    for i, name in enumerate(model_names):
        print(f"  {i + 1}: {name}")

    base_model_idx = -1
    if auto_base_index is not None and 0 <= auto_base_index < len(model_names):
        base_model_idx = auto_base_index
        print(f"Auto base model index: {auto_base_index + 1}")

    while base_model_idx < 0 or base_model_idx >= len(model_names):
        try:
            choice = input(f"Select base model (1-{len(model_names)}): ")
            idx = int(choice) - 1
            if 0 <= idx < len(model_names):
                base_model_idx = idx
            else:
                print("Invalid index, try again.")
        except ValueError:
            print("Please enter a valid number.")

    base_model_name = model_names.pop(base_model_idx)
    print(f"\nSelected base model: '{base_model_name}'")

    cumulative_f3d = all_models_data[base_model_name]['f3d'].copy()
    cumulative_r3d = all_models_data[base_model_name]['r3d'].copy()

    previous_model_data = all_models_data[base_model_name]
    remaining_models_to_splice = model_names
    print("Splice order:", " -> ".join([base_model_name] + remaining_models_to_splice))

    for attach_model_name in remaining_models_to_splice:
        print(f"\n--- Splicing '{previous_model_data['name']}' + '{attach_model_name}' ---")

        base_points = _find_splicing_points(
            previous_model_data['f3d'],
            previous_model_data['front_support_keys'],
            mode='bottom'
        )
        base_points.sort(key=lambda p: p[0])
        base_left, base_right = base_points[0], base_points[1]
        print(f"  - Base points: {base_left}, {base_right}")

        attach_model_data = all_models_data[attach_model_name]
        attach_points_orig = _find_splicing_points(
            attach_model_data['f3d'],
            attach_model_data['front_support_keys'],
            mode='top'
        )
        attach_points_orig.sort(key=lambda p: p[0])
        attach_left, attach_right = attach_points_orig[0], attach_points_orig[1]
        print(f"  - Attach points: {attach_left}, {attach_right}")

        current_attach_model_3d = {
            'f3d': attach_model_data['f3d'].copy(),
            'r3d': attach_model_data['r3d'].copy(),
        }
        _align_and_transform_model(
            current_attach_model_3d,
            source_p1=attach_left,
            source_p2=attach_right,
            target_p1=base_left,
            target_p2=base_right,
        )

        cumulative_f3d.update(current_attach_model_3d['f3d'])
        cumulative_r3d.update(current_attach_model_3d['r3d'])

        previous_model_data = {
            'name': attach_model_name,
            'f3d': current_attach_model_3d['f3d'],
            'r3d': current_attach_model_3d['r3d'],
            'front_support_keys': attach_model_data['front_support_keys'],
        }

    print("\n" + "=" * 50)
    print("Splicing complete")
    print("=" * 50)

    return cumulative_f3d, cumulative_r3d
# generate_final_output.py
import json
import numpy as np
from typing import Dict, List, Tuple

CoordMap: TypeAlias = Dict[str, Seg3D]
UniqueNodeDict: TypeAlias = Dict[Point3D, Dict[str, Any]]

TOLERANCE = 1e-4


class UniqueNodeIdentifier:
    """Store reconstructed members and allocate stable node identifiers."""

    def __init__(self, final_coords: CoordMap, all_models_data: AllModelsData):
        self.final_coords = final_coords
        self.all_models_data = all_models_data
        self.unique_nodes: UniqueNodeDict = {}
        self.base_coord_to_info: Dict[Point3D, Dict] = {}

        self._build_unique_node_map()
        self._assign_base_ids_from_semantics()

    def _get_base_coord_q3(self, point: Point3D) -> Point3D:
        x, y, z = point
        base_x = -abs(x) if abs(x) > TOLERANCE else 0.0
        base_y = -abs(y) if abs(y) > TOLERANCE else 0.0
        return (base_x, base_y, z)

    def _build_unique_node_map(self):
        all_points_info = []
        for mid_orig, seg in self.final_coords.items():
            mid = mid_orig.replace("F_", "").replace("R_", "")
            is_support = any(mid in m.get('ganjian_args', {}).get('front_support', {}) or \
                             mid in m.get('ganjian_args', {}).get('right_support', {}) \
                             for m in self.all_models_data.values())
            p1, p2 = seg
            if abs(p1[2] - p2[2]) < TOLERANCE:
                suffix1, suffix2 = ("10", "20") if p1[0] < p2[0] else ("20", "10")
            else:
                suffix1, suffix2 = ("10", "20") if p1[2] < p2[2] else ("20", "10")
            all_points_info.append({"coord": tuple(p1), "mid": mid, "is_support": is_support, "suffix": suffix1})
            all_points_info.append({"coord": tuple(p2), "mid": mid, "is_support": is_support, "suffix": suffix2})
        for p_info in all_points_info:
            found_match = False
            for unique_coord in self.unique_nodes.keys():
                if np.linalg.norm(np.array(p_info["coord"]) - np.array(unique_coord)) < TOLERANCE:
                    self.unique_nodes[unique_coord]["members"].append(p_info)
                    found_match = True
                    break
            if not found_match:
                self.unique_nodes[p_info["coord"]] = {"members": [p_info]}

    def _assign_base_ids_from_semantics(self):
        semantic_base_coords = set()
        for model_data in self.all_models_data.values():
            identifiers = model_data.get('base_node_identifiers')
            if not identifiers: continue

            left_support_id = identifiers.get('left_support_id')
            if left_support_id and left_support_id in self.final_coords:
                semantic_base_coords.update(map(tuple, self.final_coords[left_support_id]))

            for horiz_id in identifiers.get('horizontal_ids', []):
                if horiz_id in self.final_coords:
                    p1, p2 = self.final_coords[horiz_id]
                    left_endpoint = tuple(p1) if p1[0] <= p2[0] else tuple(p2)
                    semantic_base_coords.add(left_endpoint)

        for coord in semantic_base_coords:
            found_node_coord = None
            for unique_coord in self.unique_nodes.keys():
                if np.linalg.norm(np.array(coord) - np.array(unique_coord)) < TOLERANCE:
                    found_node_coord = unique_coord
                    break
            if not found_node_coord: continue

            data = self.unique_nodes[found_node_coord]
            support_members = [m for m in data["members"] if m["is_support"]]
            owner = support_members[0] if support_members else data["members"][0]
            is_support_node = any(m['is_support'] for m in data["members"])

            id_prefix_str = ''.join(filter(str.isdigit, owner["mid"]))
            base_id = f"{id_prefix_str}{owner['suffix']}"

            self.base_coord_to_info[found_node_coord] = {"id": base_id, "is_support": is_support_node}

    def get_node_id(self, point: Point3D) -> str:
        theoretical_base_coord = self._get_base_coord_q3(point)
        if not self.base_coord_to_info: return "-1"

        actual_base_coord = min(self.base_coord_to_info.keys(),
                                key=lambda bc: np.linalg.norm(np.array(bc) - np.array(theoretical_base_coord)))

        info = self.base_coord_to_info.get(actual_base_coord)
        if info is None: return "-1"

        base_id_str = info['id']
        x, y, z = point
        is_right = abs(x) > TOLERANCE and x > 0
        is_front = abs(y) > TOLERANCE and y > 0

        try:
            base_id_num = int(base_id_str)
            if is_right and not is_front:
                return str(base_id_num + 1)
            elif not is_right and is_front:
                return str(base_id_num + 2)
            elif is_right and is_front:
                return str(base_id_num + 3)
            else:
                return base_id_str
        except ValueError:
            return base_id_str



# === Paste the following into core.py (replace the old generate_outputs and add helpers) ===
from typing import Dict, List, Tuple
import math


# ---------- helpers (small, internal) ----------
def _build_pinjie_from_front_horiz(final_coords_map: dict,
                                   all_models_data: dict,
                                   ganjian: list) -> list:
    """Build splice records from front horizontal members shared by adjacent models."""
    front_h_ids = set()
    for m in (all_models_data or {}).values():
        ga = (m or {}).get('ganjian_args', {}) or {}
        fh = ga.get('front_horizontal') or {}
        for mid in fh.keys():
            front_h_ids.add(str(mid))

    if not front_h_ids:
        return []

    m_to_nodes = {}
    for g in (ganjian or []):
        mid = str(g.get('member_id'))
        n1  = str(g.get('node1_id'))
        n2  = str(g.get('node2_id'))
        if mid and (n1 is not None) and (n2 is not None):
            m_to_nodes[mid] = (n1, n2)

    # def _base_id(k: str) -> str:
    #     return str(k).split('_', 1)[0]
    def _base_id(k: str) -> str:
        k = str(k).replace("F_", "").replace("R_", "")
        return k.split('_', 1)[0]

    # 4) Collect endpoints by horizontal member, not by globally de-duplicated
    # node id. The stretcher interface consumes pinjie in groups of four:
    # two adjacent horizontal rods, each with left/right endpoints.
    horizontal_items = []
    seen_member_keys = set()
    for k, seg in (final_coords_map or {}).items():
        base = _base_id(k)
        if base not in front_h_ids:
            continue
        member_key = str(k)
        if member_key in seen_member_keys:
            continue
        if not isinstance(seg, (list, tuple)) or len(seg) != 2:
            continue

        node_ids = m_to_nodes.get(str(k))
        if node_ids is None:
            node_ids = m_to_nodes.get(base)
        if node_ids is None:
            continue

        n1, n2 = node_ids
        p1, p2 = seg
        c1 = [float(p1[0]), float(p1[1]), float(p1[2])]
        c2 = [float(p2[0]), float(p2[1]), float(p2[2])]
        endpoints = [(str(n1), c1), (str(n2), c2)]
        endpoints.sort(key=lambda item: item[1][0])
        z_avg = (c1[2] + c2[2]) / 2.0
        x_avg = (c1[0] + c2[0]) / 2.0
        horizontal_items.append((z_avg, x_avg, member_key, endpoints))
        seen_member_keys.add(member_key)

    # 5) Sort by tower height, then lateral position/member id for stability.
    horizontal_items.sort(key=lambda item: (item[0], item[1], item[2]))

    # 6) Flatten back to the legacy pinjie structure.
    pinjie = []
    for _, _, _, endpoints in horizontal_items:
        for node_id, coord in endpoints:
            pinjie.append([node_id, coord])
    return pinjie

def _last2(s: str) -> str:
    return s[-2:] if len(s) >= 2 else s

def _base(s: str) -> str:
    return s[:-2] if len(s) >= 2 and s[-2:].isdigit() else s

def _plus_suffix(s: str, delta: int) -> str:
    suf = _last2(s)
    if not suf.isdigit():
        return s
    n = int(suf) + delta
    return f"{_base(s)}{n:02d}"

def _sym_pt(pt: Point3D, sym_type: int) -> Point3D:
    x, y, z = pt
    if sym_type == 1:
        return (-x, y, z)
    if sym_type == 2:
        return (x, -y, z)
    if sym_type == 3:
        return (-x, -y, z)
    return pt

def _dist3(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def _sort_lr(p1: Point3D, p2: Point3D) -> Tuple[Point3D, Point3D]:
    if (p1[0], p1[1]) <= (p2[0], p2[1]):
        return p1, p2
    return p2, p1

def _top_bottom(p1: Point3D, p2: Point3D) -> Tuple[Point3D, Point3D]:
    if p1[2] <= p2[2]:
        return p1, p2
    return p2, p1

def _closest_id(pt: Point3D, known: Dict[str, Point3D], eps=1e-6) -> str:
    for k, v in known.items():
        if _dist3(pt, v) <= eps:
            return k
    return ""


def _reference_value(node_id: str) -> str:
    """Encode a node reference using the same leading-1 format as single view."""
    return f"1{str(node_id)}"


def _reference_node_values(pt: Point3D, ref_ids: Tuple[str, str], ref_seg: List[Point3D]) -> Dict[str, object]:
    """
    Build a node_type=12 coordinate payload.

    Two coordinate fields hold node references and the remaining field stores
    the real coordinate value along the strongest axis of the host rod.
    """
    axis_names = ("X", "Y", "Z")
    if ref_seg and len(ref_seg) >= 2:
        deltas = [abs(ref_seg[1][i] - ref_seg[0][i]) for i in range(3)]
        real_axis = max(range(3), key=lambda i: deltas[i])
        if deltas[real_axis] <= 1e-8:
            real_axis = 2
    else:
        real_axis = 2

    ref_iter = iter((_reference_value(ref_ids[0]), _reference_value(ref_ids[1])))
    values: Dict[str, object] = {}
    for idx, axis_name in enumerate(axis_names):
        if idx == real_axis:
            values[axis_name] = round(pt[idx], 3)
        else:
            values[axis_name] = next(ref_iter)
    return values





def generate_outputs(final_coords_map: Dict[str, List[Point3D]], all_models_data: Dict[str, dict]):
    """Generate member, node, and splice output records from reconstructed coordinates."""
    ganjian: List[dict] = []
    jiedian: List[dict] = []
    pinjie: List[list] = [] 

    EPS = 150.0

    for stem, pack in all_models_data.items():
        args = pack.get("ganjian_args", {})
        front_support: Dict[str, list] = {str(k): v for k, v in (args.get("front_support") or {}).items()}
        right_support: Dict[str, list] = {str(k): v for k, v in (args.get("right_support") or {}).items()}
        front_horizontal: Dict[str, list] = {str(k): v for k, v in (args.get("front_horizontal") or {}).items()}
        right_horizontal: Dict[str, list] = {str(k): v for k, v in (args.get("right_horizontal") or {}).items()}
        front_x: Dict[str, list] = {str(k): v for k, v in (args.get("front_x_fixed") or {}).items()}
        right_x: Dict[str, list] = {str(k): v for k, v in (args.get("right_x_fixed") or {}).items()}
        reference_tiers = {
            "F": args.get("front_reference_tiers") or {},
            "R": args.get("right_reference_tiers") or {},
        }
        member_nodes: Dict[Tuple[str, str], Tuple[str, str]] = {}
        member_points: Dict[Tuple[str, str], Tuple[Point3D, Point3D]] = {}

        def _tier_for(view_tag: str, member_id: str) -> str:
            tiers = reference_tiers.get(view_tag) or {}
            member_id = str(member_id)
            if member_id in (tiers.get("tier2") or set()):
                return "tier2"
            if member_id in (tiers.get("tier3") or set()):
                return "tier3"
            return "tier1"

        def _host_for_endpoint(view_tag: str, member_id: str, endpoint_index: int):
            tiers = reference_tiers.get(view_tag) or {}
            hosts = (tiers.get("endpoint_hosts") or {}).get(str(member_id)) or []
            if endpoint_index < len(hosts):
                return hosts[endpoint_index]
            return None

        def _register_node(
            node_id: str,
            pt: Point3D,
            view_tag: str,
            member_id: str,
            endpoint_index: int,
            force_real: bool = False,
        ) -> None:
            if force_real or _tier_for(view_tag, str(member_id)) == "tier1":
                jiedian.append({
                    "node_id": node_id,
                    "node_type": 11,
                    "X": round(pt[0], 3),
                    "Y": round(pt[1], 3),
                    "Z": round(pt[2], 3),
                    "symmetry_type": 4,
                })
            else:
                host = _host_for_endpoint(view_tag, str(member_id), endpoint_index) or {}
                host_key = (view_tag, str(host.get("host", "")))
                ref_ids = member_nodes.get(host_key)
                ref_seg = member_points.get(host_key)
                if not ref_ids or not ref_seg:
                    jiedian.append({
                        "node_id": node_id,
                        "node_type": 11,
                        "X": round(pt[0], 3),
                        "Y": round(pt[1], 3),
                        "Z": round(pt[2], 3),
                        "symmetry_type": 4,
                    })
                else:
                    values = _reference_node_values(pt, ref_ids, list(ref_seg))
                    jiedian.append({
                        "node_id": node_id,
                        "node_type": 12,
                        "X": values["X"],
                        "Y": values["Y"],
                        "Z": values["Z"],
                        "symmetry_type": 4,
                    })

            known_nodes[node_id] = pt
            known_nodes[_plus_suffix(node_id, +1)] = _sym_pt(pt, 1)
            known_nodes[_plus_suffix(node_id, +2)] = _sym_pt(pt, 2)
            known_nodes[_plus_suffix(node_id, +3)] = _sym_pt(pt, 3)

        def _remember_member(
            view_tag: str,
            member_id: str,
            node1_id: str,
            node2_id: str,
            p1: Point3D,
            p2: Point3D,
        ) -> None:
            member_key = (view_tag, str(member_id))
            member_nodes[member_key] = (str(node1_id), str(node2_id))
            member_points[member_key] = (p1, p2)

        base_sid = None
        min_cx = None
        for sid in front_support.keys():
            seg3d = final_coords_map.get(f"F_{sid}") 
            # seg3d = final_coords_map.get(str(sid))
            if not seg3d: continue
            cx = 0.5*(seg3d[0][0] + seg3d[1][0])
            if (min_cx is None) or (cx < min_cx):
                min_cx = cx
                base_sid = str(sid)
        if not base_sid: continue

        # pA, pB = final_coords_map[base_sid]
        pA, pB = final_coords_map[f"F_{base_sid}"]
        topP, botP = _top_bottom(pA, pB)
        sid10 = f"{node_id_base(base_sid)}10"
        sid20 = f"{node_id_base(base_sid)}20"

        known_nodes: Dict[str, Point3D] = {
            sid10: topP, sid20: botP,
            _plus_suffix(sid10, +1): _sym_pt(topP, 1),
            _plus_suffix(sid10, +2): _sym_pt(topP, 2),
            _plus_suffix(sid10, +3): _sym_pt(topP, 3),
            _plus_suffix(sid20, +1): _sym_pt(botP, 1),
            _plus_suffix(sid20, +2): _sym_pt(botP, 2),
            _plus_suffix(sid20, +3): _sym_pt(botP, 3),
        }

        _register_node(sid10, topP, "F", base_sid, 0, force_real=True)
        _register_node(sid20, botP, "F", base_sid, 1, force_real=True)
        _remember_member("F", base_sid, sid10, sid20, topP, botP)
        ganjian.append({"member_id": base_sid, "node1_id": sid10, "node2_id": sid20, "symmetry_type": 4})

        for sid in front_support.keys():
            if str(sid) == base_sid:
                continue
            # seg3d = final_coords_map.get(str(sid))
            seg3d = final_coords_map.get(f"F_{sid}") 
            if not seg3d:
                continue
            pA, pB = seg3d
            topP_other, botP_other = _top_bottom(pA, pB)
            sid10_other = f"{node_id_base(sid)}10"
            sid20_other = f"{node_id_base(sid)}20"
            
            reuse_top = _closest_id(topP_other, known_nodes, eps=EPS)
            reuse_bot = _closest_id(botP_other, known_nodes, eps=EPS)
            
            if not reuse_top:
                _register_node(sid10_other, topP_other, "F", str(sid), 0)
                node1_id = sid10_other
            else:
                node1_id = reuse_top
            
            if not reuse_bot:
                _register_node(sid20_other, botP_other, "F", str(sid), 1)
                node2_id = sid20_other
            else:
                node2_id = reuse_bot
            _remember_member("F", str(sid), node1_id, node2_id, topP_other, botP_other)
            ganjian.append({"member_id": str(sid), "node1_id": node1_id, "node2_id": node2_id, "symmetry_type": 4})

        for rid in right_support.keys():
            # seg3d = final_coords_map.get(str(rid))
            seg3d = final_coords_map.get(f"R_{rid}")
            if not seg3d:
                continue
            pA, pB = seg3d
            topP_r, botP_r = _top_bottom(pA, pB)
            rid10 = f"{node_id_base(rid)}10"
            rid20 = f"{node_id_base(rid)}20"
            
            reuse_top_r = _closest_id(topP_r, known_nodes, eps=EPS)
            reuse_bot_r = _closest_id(botP_r, known_nodes, eps=EPS)
            
            if not reuse_top_r:
                _register_node(rid10, topP_r, "R", str(rid), 0)
                node1_id_r = rid10
            else:
                node1_id_r = reuse_top_r
            
            if not reuse_bot_r:
                _register_node(rid20, botP_r, "R", str(rid), 1)
                node2_id_r = rid20
            else:
                node2_id_r = reuse_bot_r
            _remember_member("R", str(rid), node1_id_r, node2_id_r, topP_r, botP_r)
            ganjian.append({"member_id": str(rid), "node1_id": node1_id_r, "node2_id": node2_id_r, "symmetry_type": 4})

        def _sym1_partner(nid: str) -> str:
            """Return the symmetry-type-1 partner node identifier."""
            _m = {"0": "1", "1": "0", "2": "3", "3": "2"}
            return nid[:-1] + _m[nid[-1]] if nid and nid[-1] in _m else _plus_suffix(nid, +1)

        for hid in sorted(front_horizontal.keys(), key=lambda x: float(x)):
            # seg3d = final_coords_map.get(str(hid))
            seg3d = final_coords_map.get(f"F_{hid}")
            if not seg3d: continue
            L, R = _sort_lr(*seg3d)
            reuse_id = _closest_id(L, {sid10: topP, sid20: botP}, eps=EPS)
            
            if reuse_id:
                left_node_id = reuse_id  
            else:
                left_node_id = f"{node_id_base(hid)}10"
                _register_node(left_node_id, L, "F", str(hid), 0)

            _remember_member("F", str(hid), left_node_id, _sym1_partner(left_node_id), L, R)
            ganjian.append({
                "member_id": str(hid), "node1_id": left_node_id,
                "node2_id": _sym1_partner(left_node_id), "symmetry_type": 2
            })

        def _sym2_partner(nid: str) -> str:
            """Return the symmetry-type-2 partner node identifier."""
            _m = {"0": "2", "2": "0", "1": "3", "3": "1"}
            return nid[:-1] + _m[nid[-1]] if nid and nid[-1] in _m else _plus_suffix(nid, +2)

        for rid in sorted(right_horizontal.keys(), key=lambda x: float(x)):
            # seg3d = final_coords_map.get(str(rid))
            seg3d = final_coords_map.get(f"R_{rid}")
            if not seg3d: continue
            Lr, Rr = _sort_lr(*seg3d)
            nid_left_guess = _closest_id(Lr, known_nodes, eps=EPS)
            
            if nid_left_guess:
                left_node_id_r = nid_left_guess
            else:
                left_node_id_r = f"{node_id_base(rid)}30"
                _register_node(left_node_id_r, Lr, "R", str(rid), 0)
            # if nid_left_guess:
            #     left_node_id_r = nid_left_guess
            # else:
            #     left_node_id_r = f"{base_id(rid)}10" 
            #     jiedian.append({"node_id": left_node_id_r, "node_type": 11, "X": round(Lr[0],3), "Y": round(Lr[1],3), "Z": round(Lr[2],3), "symmetry_type": 4})
                if left_node_id_r not in known_nodes:
                    _register_node(left_node_id_r, Lr, "R", str(rid), 0)

            _remember_member("R", str(rid), left_node_id_r, _sym2_partner(left_node_id_r), Lr, Rr)
            ganjian.append({
                "member_id": str(rid), "node1_id": left_node_id_r,
                "node2_id": _sym2_partner(left_node_id_r), "symmetry_type": 1
            })




        def _x_node_prefix(view_tag: str, xid: str) -> str:
            return f"{view_tag}{str(xid).replace('_', '')}"

        def _register_x_node(
            preferred_id: str,
            pt: Point3D,
            view_tag: str,
            member_id: str,
            endpoint_index: int,
        ) -> str:
            reuse_id = _closest_id(pt, known_nodes, eps=EPS)
            if reuse_id:
                return reuse_id
            node_id = preferred_id
            if node_id in known_nodes:
                base = preferred_id[:-2] if len(preferred_id) >= 2 else preferred_id
                suffix = preferred_id[-2:] if len(preferred_id) >= 2 else "10"
                idx = 1
                while f"{base}{suffix}{idx}" in known_nodes:
                    idx += 1
                node_id = f"{base}{suffix}{idx}"
            _register_node(node_id, pt, view_tag, str(member_id), endpoint_index)
            return node_id

        def _emit_x_member(view_tag: str, xid: str, seg3d: List[Point3D]):
            if not seg3d or len(seg3d) < 2:
                return
            ordered = sorted(
                ((seg3d[0], 0), (seg3d[1], 1)),
                key=lambda item: (item[0][0], item[0][1]),
            )
            (p1, idx1), (p2, idx2) = ordered
            node1 = _register_x_node(f"{_x_node_prefix(view_tag, xid)}10", p1, view_tag, xid, idx1)
            node2 = _register_x_node(f"{_x_node_prefix(view_tag, xid)}20", p2, view_tag, xid, idx2)
            _remember_member(view_tag, str(xid), node1, node2, p1, p2)
            member_id = str(xid)

            sym_type = 2 if view_tag == "F" else 1

            ganjian.append({
                "member_id": str(member_id),
                "node1_id": node1,
                "node2_id": node2,
                "symmetry_type": sym_type,
            })

        tier_order = {"tier1": 0, "tier2": 1, "tier3": 2}

        def _x_sort_key(view_tag: str, xid: str):
            return (tier_order.get(_tier_for(view_tag, str(xid)), 9), str(xid))

        for xid in sorted(front_x.keys(), key=lambda value: _x_sort_key("F", str(value))):
            fseg = final_coords_map.get(f"F_{xid}")
            if fseg and len(fseg) >= 2:
                _emit_x_member("F", str(xid), fseg)

        for xid in sorted(right_x.keys(), key=lambda value: _x_sort_key("R", str(value))):
            rseg = final_coords_map.get(f"R_{xid}")
            if rseg and len(rseg) >= 2:
                _emit_x_member("R", str(xid), rseg)

        pinjie = _build_pinjie_from_front_horiz(final_coords_map, all_models_data, ganjian)
        
    return ganjian, jiedian, pinjie





# I/O
def drop_vertical_members(
    members: CoordDict,
    span_length: Optional[float],
    dx_ratio: float = 0.01,
    dx_abs: float = 4.0,
    min_len: float = 5.0,
    return_excluded: bool = False,
):
    """Remove vertical members that should be represented through symmetry reuse."""
    dx_th = float(dx_abs)
    if span_length is not None:
        dx_th = max(dx_th, float(dx_ratio) * float(span_length))

    kept: CoordDict = {}
    excluded: CoordDict = {}

    for k, seg in members.items():
        if not seg or len(seg) < 2:
            kept[str(k)] = seg
            continue
        (x1, y1), (x2, y2) = seg
        dx = abs(float(x2) - float(x1))
        dy = abs(float(y2) - float(y1))
        L  = math.hypot(dx, dy)
        if L < float(min_len):
            kept[str(k)] = seg
            continue
        if dx <= dx_th:
            excluded[str(k)] = seg
        else:
            kept[str(k)] = seg

    return (kept, excluded) if return_excluded else kept


# ====== New: Vertical stacking alias builder & post-processor ======
from typing import Set

def build_vertical_reuse_aliases(final_coords_map: Dict[str, List[Point3D]],
                                 all_models_data: Dict[str, dict],
                                 eps: float = 1e-6) -> Dict[str, str]:
    """Build aliases for vertically reused member and node identifiers."""
    parts = []
    for stem, pack in (all_models_data or {}).items():
        args = (pack or {}).get("ganjian_args", {}) or {}
        front_support: Dict[str, list] = {str(k): v for k, v in (args.get("front_support") or {}).items()}
        base_sid, min_cx = None, None
        for sid in sorted(front_support.keys(), key=lambda x: float(x)):
            # seg3d = final_coords_map.get(str(sid))
            seg3d = final_coords_map.get(f"F_{sid}")

            if not seg3d or len(seg3d) < 2:
                continue
            cx = 0.5 * (float(seg3d[0][0]) + float(seg3d[1][0]))
            if (min_cx is None) or (cx < min_cx):
                min_cx, base_sid = cx, str(sid)
        if not base_sid:
            continue
        # pA, pB = final_coords_map[base_sid]
        pA, pB = final_coords_map[f"F_{base_sid}"]
        topP, botP = _top_bottom(pA, pB)
        parts.append({"stem": stem, "sid": base_sid, "top": topP, "bot": botP})

    aliases: Dict[str, str] = {}
    for i in range(len(parts)):
        upper = parts[i]
        for j in range(len(parts)):
            if i == j: 
                continue
            lower = parts[j]
            if _dist3(upper["bot"], lower["top"]) <= float(eps):
                lower_sid10 = f"{lower['sid']}10"
                upper_sid20 = f"{upper['sid']}20"
                aliases[lower_sid10] = upper_sid20
                aliases[_plus_suffix(lower_sid10, +1)] = _plus_suffix(upper_sid20, +1)
                aliases[_plus_suffix(lower_sid10, +2)] = _plus_suffix(upper_sid20, +2)
                aliases[_plus_suffix(lower_sid10, +3)] = _plus_suffix(upper_sid20, +3)
    return aliases


def apply_id_aliases(ganjian: list, jiedian: list, pinjie: list, id_aliases: Dict[str, str]):
    """Apply member and node aliases to all exported record collections."""
    def canon(value: str) -> str:
        """Return an aliased identifier, resolving alias chains safely."""
        def _lift(one: str) -> str:
            seen: Set[str] = set()
            cur = one
            while cur in id_aliases and cur not in seen:
                seen.add(cur)
                cur = id_aliases[cur]
            return cur

        s = str(value)
        while True:
            direct = _lift(s)
            if direct != s:
                s = direct
                continue

            if s.startswith("1") and len(s) > 1:
                tail = s[1:]
                canon_tail = _lift(tail)
                if canon_tail != tail:
                    s = "1" + canon_tail
                    continue
            break
        return s

    # ---- ganjian ----
    new_g = []
    seen_g = set()
    for row in (ganjian or []):
        r = dict(row)
        r["member_id"] = str(r.get("member_id"))
        r["node1_id"] = canon(str(r.get("node1_id")))
        r["node2_id"] = canon(str(r.get("node2_id")))
        key = (r["member_id"], r["node1_id"], r["node2_id"], r.get("symmetry_type"))
        if key in seen_g:
            continue
        seen_g.add(key)
        new_g.append(r)

    # ---- jiedian ----
    new_j = []
    seen_j = set()
    for row in (jiedian or []):
        r = dict(row)
        nid = canon(str(r.get("node_id")))
        r["node_id"] = nid
        if r.get("node_type") == 12:
            if "X" in r: r["X"] = canon(str(r["X"]))
            if "Y" in r: r["Y"] = canon(str(r["Y"]))
            if "Z" in r and isinstance(r.get("Z"), str): r["Z"] = canon(str(r["Z"]))
            key = ("12", nid)
            if key in seen_j: 
                continue
            seen_j.add(key)
            new_j.append(r)
        else:
            key = ("11", nid)
            if key in seen_j:
                continue
            seen_j.add(key)
            new_j.append(r)

    # ---- pinjie ----
    new_p_items = []
    seen_p = set()
    for item in (pinjie or []):
        if not isinstance(item, list) or not item:
            continue
        nid = canon(str(item[0]))
        if nid in seen_p:
            continue
        seen_p.add(nid)
        coord = item[1] if len(item) > 1 else None
        new_p_items.append([nid, coord])

    return new_g, new_j, new_p_items
