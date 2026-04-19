# core.py (self-contained)
"""
精简版核心：把原先分散在多个模块的实现统一整合到一个文件里，
对外提供清晰的函数名，供 main.py 调用。无需其它 .py 依赖。
"""
from typing import Dict, List, Tuple, Optional, TypeAlias, Any, Set
import math

# ====== 通用 3D 模型中心属性与平移（保持接口不变） ======
Point3D: TypeAlias = Tuple[float, float, float]
Seg3D: TypeAlias = List[Point3D]
Model3DData: TypeAlias = Dict[str, Seg3D]


def base_id(rid) -> str:
    """
    从 '508_2' / '1310_1' / '508' 中提取基准构件号。
    统一所有新建节点 ID 的前缀来源，避免把后缀视为主 ID 一部分。
    """
    return str(rid).split("_", 1)[0]

def _collect_endpoints(segdict: Model3DData) -> List[Point3D]:
    pts: List[Point3D] = []
    for seg in segdict.values():
        if seg and len(seg) >= 2:
            pts.append(seg[0]); pts.append(seg[1])
    return pts

def center_props(front3d: Model3DData, right3d: Model3DData):
    """返回 ((x_center, y_center, 0), z_min)"""
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

# ====== X 型构件筛选 ======
def select_x_type(rest_dict: Dict[str, List[Tuple[float, float]]], x_range: Tuple[float, float]) -> Dict[str, List[Tuple[float, float]]]:
    if not rest_dict or not x_range: return {}
    lo, hi = x_range; out = {}
    for k, seg in rest_dict.items():
        if not seg or len(seg) < 2: continue
        (x1,y1),(x2,y2) = seg[0],seg[1]
        x_mid = (x1 + x2) / 2.0
        if lo <= x_mid <= hi: out[str(k)] = [tuple(seg[0]), tuple(seg[1])]
    return out

# ====== 按指定杆件实长缩放 ======
def scale_by_member(final_coords_map: Dict[str, List[Tuple[float, float, float]]], target_id: str, real_length: float):
    p1, p2 = final_coords_map.get(target_id, (None, None))
    if p1 is None or p2 is None: raise KeyError(f"杆件ID '{target_id}' 不存在。")
    L = math.sqrt(sum((a-b)**2 for a,b in zip(p1, p2)))
    if L < 1e-9: raise ValueError("所选杆件长度过小。")
    s = real_length / L
    return {mid: [(p[0]*s, p[1]*s, p[2]*s) for p in seg] for mid, seg in final_coords_map.items()}


# ====== Loader ======
# loader.py
import re
from typing import Dict, List, Tuple, TypeAlias

Coord: TypeAlias = Tuple[float, float]
CoordDict: TypeAlias = Dict[str, List[Coord]]

# 解析 block 形式：coordinatesFront_data={ id:[(x1,y1),(x2,y2)], ... }
_BLOCK_RE = re.compile(
    r"coordinates(?P<name>Front|Overhead)_data\s*=\s*\{(?P<body>.*?)\}",
    re.IGNORECASE | re.DOTALL,
)
_PAIR_RE = re.compile(
    r"(?P<id>[0-9A-Za-z_]+)\s*:\s*\[\s*\(\s*(?P<x1>[-+0-9.eE]+)\s*,\s*(?P<y1>[-+0-9.eE]+)\s*\)\s*,\s*\(\s*(?P<x2>[-+0-9.eE]+)\s*,\s*(?P<y2>[-+0-9.eE]+)\s*\)\s*\]",
    re.IGNORECASE,
)

# 解析“分节”形式：以 front/right 别名开头的一段行，行内含 4 个数（x1 y1 x2 y2）
_FRONT_ALIASES = {"front", "正面", "front:", "[front]", "f:", "f"}
_RIGHT_ALIASES = {"right", "侧面", "右侧面", "右视图", "overhead", "overhead:", "right:", "[right]", "r:", "r"}
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
    """
    每一行抓 4 个数字：x1 y1 x2 y2
    允许逗号/空格任意分隔
    """
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
    """
    返回 (front_dict, right_dict)；任一视图缺失则返回空 dict
    """
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    # 1) 优先尝试 block 形式
    front = _parse_block_dict(text, "front")
    right = _parse_block_dict(text, "right")

    # 2) 若为空，尝试“分节”形式
    if not front and not right:
        blocks = _split_blocks_by_headers(text.splitlines())
        front = _parse_section_lines(blocks.get("front", []))
        right = _parse_section_lines(blocks.get("right", []))

    return front, right

# ====== Processors (原 processors.py) ======
# processors.py —— 清洗、分类与几何修正
from typing import Dict, List, Tuple, Optional
import math
import statistics

# Coord 和 CoordDict 已在文件开头定义

# ----------------
# 基础清洗
# ----------------
def _len2(p1: Coord, p2: Coord) -> float:
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    return dx*dx + dy*dy

def clean_view(view: CoordDict, view_name: str, round_ndigits: Optional[int] = None) -> CoordDict:
    """
    - 坐标浮点化
    - 去除零长度线段
    - 可选：四舍五入（当前主流程传 None，表示不取整）
    """
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

# ----------------
# 分类：支撑与横向
# ----------------
def find_supports(view: CoordDict) -> CoordDict:
    """
    支撑杆件：ID 为纯数字、末尾 01/02/03/04、且不含下划线
    额外条件：杆件长度应该相对较长（贯穿大部分塔身高度）
    排除短的 X 型杆件（如 503/504）
    """
    # 第一步：收集所有候选支撑（ID 规则）
    candidates = {}
    for k, seg in view.items():
        ks = str(k)
        if ks.isdigit() and len(ks) >= 2 and ks[-2:] in {"01","02","03","04"} and "_" not in ks:
            candidates[ks] = [(float(seg[0][0]), float(seg[0][1])),
                             (float(seg[1][0]), float(seg[1][1]))]
    
    if not candidates:
        return {}
    
    # 第二步：计算所有候选的长度，找出最长的
    lengths = {}
    for k, seg in candidates.items():
        (x1, y1), (x2, y2) = seg
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        lengths[k] = length
    
    if not lengths:
        return {}
    
    max_length = max(lengths.values())
    # 支撑杆件应该是相对较长的（至少是最长杆件的 60%）
    length_threshold = max_length * 0.6
    
    # 第三步：筛选出足够长的杆件作为支撑
    out = {}
    for k, seg in candidates.items():
        if lengths[k] >= length_threshold:
            out[k] = seg
    
    return out if out else candidates  # 如果过滤太严格，回退到所有候选

def find_horizontals(view: CoordDict, tol_y: float, exclude_support: bool = True) -> CoordDict:
    """
    横向杆件：|y1 - y2| <= tol_y（自适应容差）
    可选：排除支撑（默认排除）
    """
    out: CoordDict = {}
    for k, seg in view.items():
        if exclude_support:
            ks = str(k)
            if ks.isdigit() and len(ks) >= 2 and ks[-2:] in {"01","02","03","04"} and "_" not in ks:
                continue
        (x1,y1),(x2,y2) = seg
        if abs(float(y1)-float(y2)) <= float(tol_y):
            out[str(k)] = [(float(x1), float(y1)), (float(x2), float(y2))]
    return out

# ----------------
# 支撑直线模型
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
        # 此时 self.b 不会是 None（因为 k 不是 None）
        return (y - float(self.b or 0.0)) / self.k

    def x_mid(self) -> float:
        ym = (self.ymin + self.ymax)/2.0
        return self.x_at(ym)

def build_support_models(view_support: CoordDict) -> List[Line]:
    models: List[Line] = []
    for gid, (p1,p2) in view_support.items():
        ln = Line(p1,p2,str(gid))
        models.append(ln)
    # 按“中心 x”排序，方便左右取极值
    models.sort(key=lambda L: L.x_mid())
    return models

def _extreme_x_at(models: List[Line], y: float) -> Tuple[float, float]:
    """
    在给定 y 处，求左右两根支撑对应的 x：取所有支撑的 x_at(y)，返回最小与最大
    """
    xs = [m.x_at(y) for m in models]
    if not xs:
        return (0.0, 0.0)
    xs = [x for x in xs if math.isfinite(x)]
    if not xs:
        return (0.0, 0.0)
    return (min(xs), max(xs))

# ----------------
# 旧：支撑统一到各自视图的全局上下边界（保持斜率不变）
# ----------------
def correct_paired_horizontals(front_horizontal: CoordDict,
                               right_horizontal: CoordDict,
                               front_support_models: List[Line],
                               right_support_models: List[Line],
                               round_to_int: bool = False):
    """
    其余横杆成对同高：对齐后两端强制落在支撑上
    """
    if not front_horizontal and not right_horizontal:
        return front_horizontal, right_horizontal

    def _to_list(hh: CoordDict):
        return [(k, (seg[0][1]+seg[1][1])/2.0) for k, seg in hh.items()]

    fl = sorted(_to_list(front_horizontal), key=lambda t: t[1])
    rl = sorted(_to_list(right_horizontal), key=lambda t: t[1])

    n = min(len(fl), len(rl))
    # 1) 成对处理
    for i in range(n):
        kf, yf = fl[i]; kr, yr = rl[i]
        y = (yf + yr)/2.0

        Lf, Rf = _extreme_x_at(front_support_models, y)
        Lr, Rr = _extreme_x_at(right_support_models, y)

        if round_to_int:
            y, Lf, Rf, Lr, Rr = round(y), round(Lf), round(Rf), round(Lr), round(Rr)

        front_horizontal[str(kf)] = [(Lf, y), (Rf, y)]
        right_horizontal[str(kr)] = [(Lr, y), (Rr, y)]

    # 2) 剩余未配对者各自按本视图支撑投影
    def _project_rest(hh: CoordDict, models: List[Line], used_keys: set):
        for k, seg in list(hh.items()):
            if k in used_keys:
                continue
            y = (seg[0][1]+seg[1][1])/2.0
            L, R = _extreme_x_at(models, y)
            if round_to_int:
                y, L, R = round(y), round(L), round(R)
            hh[str(k)] = [(L, y), (R, y)]
    _project_rest(front_horizontal, front_support_models, {kf for kf,_ in fl[:n]})
    _project_rest(right_horizontal, right_support_models, {kr for kr,_ in rl[:n]})

    return front_horizontal, right_horizontal

# ================================
# 新增：跨视图成对统一支撑斜率 + 顶横杆等长 + 支撑扩展 + 仅余横杆交点
# ================================

def match_support_slopes(front_support: CoordDict,
                                     right_support: CoordDict,
                                     strategy: str = "mean",
                                     unify_bounds: bool = True,
                                     round_to_int: bool = False,
                                     y_round_to_int: bool = False) -> Tuple[CoordDict, CoordDict]:
    """
    跨视图“成对”统一支撑斜率：以左右极值支撑配对（左↔左、右↔右），
    将配对后的斜率 k 一致（默认取均值），并把端点统一到跨视图全局 ymin/ymax。
    """
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
        # 默认均值；垂直线（k=None）与近水平特殊处理
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
            # 垂直线：保持竖直但通过锚点 x
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
    items = sorted(view_h.items(), key=lambda kv: (kv[1][0][1]+kv[1][1][1])/2.0)  # y越小越高
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
    pair_mode: str = "extreme"           # 新增："extreme"=最外侧成对；"best"=原先最接近目标宽度
):
    """
    计算“等长顶横杆”的共同顶高 y_top 以及两视图端点 L/R。
    - 当 pair_mode="extreme"：直接取“最左&最右”两根支撑，在 y_top 处代入求交得到 (Lf,Rf)/(Lr,Rr)。
    - 当 pair_mode="best"：保留原先在所有支撑里寻找最接近目标宽度的一对。
    """
    if not front_support_models or not right_support_models:
        return None

    # 1) 先按“最高横杆”求共同顶高（你现有的 y_top 计算保持不变）
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

    # 小工具：取“最左/最右”两根支撑（build_support_models 已按水平位置排序）
    def _extremes(models):
        if len(models) == 1:  # 只有一根也能返回同一根，避免崩
            return models[0], models[0]
        return models[0], models[-1]

    # 小工具：安全地在 y_top 处取交点 x（考虑竖直线/近水平线）
    def _x_at(line_model: Line, y: float) -> float:
        if getattr(line_model, "vertical_x", None) is not None:
            return float(line_model.vertical_x or 0.0)  # type: ignore
        k = getattr(line_model, "k", None)
        b = getattr(line_model, "b", None)
        if k is None or abs(k) < 1e-12:
            # 极端退化：近水平支撑；取该支撑当前“中心 x”作为兜底
            return (getattr(line_model, "xmin", 0.0) + getattr(line_model, "xmax", 0.0)) / 2.0
        return (y - float(b or 0.0)) / k  # type: ignore

    # 2) 直接代入到“最外侧支撑”在 y_top 的交点，得到原始端点对
    if pair_mode == "extreme":
        F_left, F_right = _extremes(front_support_models)
        R_left, R_right = _extremes(right_support_models)
        Lf0, Rf0 = _x_at(F_left, y_top), _x_at(F_right, y_top)
        Lr0, Rr0 = _x_at(R_left, y_top), _x_at(R_right, y_top)
        if Lf0 > Rf0: Lf0, Rf0 = Rf0, Lf0
        if Lr0 > Rr0: Lr0, Rr0 = Rr0, Lr0
        Wf0, Wr0 = (Rf0 - Lf0), (Rr0 - Lr0)

        # 3) 仍需“等长”——用 length_mode 决定目标宽度 Wt，并以各自中心为轴做对称收缩/扩张
        if length_mode == "front":
            Wt = Wf0
        elif length_mode == "right":
            Wt = Wr0
        elif length_mode == "mean":
            Wt = (Wf0 + Wr0) / 2.0
        else:  # "min"：保守取较小者
            Wt = min(Wf0, Wr0)

        Cf, Cr = (Lf0 + Rf0) / 2.0, (Lr0 + Rr0) / 2.0
        Lf, Rf = Cf - Wt / 2.0, Cf + Wt / 2.0
        Lr, Rr = Cr - Wt / 2.0, Cr + Wt / 2.0

    else:
        # 保留你原来的“在所有支撑集合中找一对最接近目标宽度”的逻辑（原代码放这里即可）
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
        # 把“最高横杆”的 key 带回去，供后续 correct_paired_horizontals_rest 跳过顶横杆重写
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
    """
    在保持“已统一”的左右支撑斜率不变的前提下，重建左右最外侧支撑，使其在 y_top 处通过 (Lf,Rf)/(Lr,Rr)。
    """
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
    round_to_int: bool = False
) -> Tuple[CoordDict, CoordDict]:
    """
    仅对“去掉顶横杆后的剩余横杆”执行交点强制（包装原 correct_paired_horizontals 逻辑），
    顶横杆保持等长后的结果不被覆盖。
    """
    skip_front_keys = skip_front_keys or set()
    skip_right_keys = skip_right_keys or set()

    fh_keep = {k:v for k,v in front_horizontal.items() if k in skip_front_keys}
    rh_keep = {k:v for k,v in right_horizontal.items() if k in skip_right_keys}
    fh_rest = {k:v for k,v in front_horizontal.items() if k not in skip_front_keys}
    rh_rest = {k:v for k,v in right_horizontal.items() if k not in skip_right_keys}

    fh_rest2, rh_rest2 = correct_paired_horizontals(
        fh_rest, rh_rest, front_support_models, right_support_models, round_to_int=round_to_int
    )
    fh_rest2.update(fh_keep)
    rh_rest2.update(rh_keep)
    return fh_rest2, rh_rest2


# processors.py (替换掉旧的 enforce_2d_view_symmetry 函数)

def enforce_symmetry(support_orig: CoordDict,
                             horizontal_orig: CoordDict) -> Tuple[CoordDict, CoordDict]:
    """
    在2D视图层面，强制支撑杆件和横向杆件关于其自身的中心垂直线 (x=a) 对称。
    这个函数会确保修正后的横向杆件端点精确地落在修正后的支撑杆件上，
    并且不会在2D阶段对整个图形进行全局平移。
    """
    if not support_orig and not horizontal_orig:
        return support_orig, horizontal_orig

    print("  - 正在对2D视图执行原地对称修正...")

    # 1. 计算当前视图的中心垂直线 x_center (对称轴)
    all_x = []
    for seg in list(support_orig.values()) + list(horizontal_orig.values()):
        all_x.extend([p[0] for p in seg])
    if not all_x:
        return support_orig, horizontal_orig
    x_center = (min(all_x) + max(all_x)) / 2.0

    support = dict(support_orig)
    horizontal = dict(horizontal_orig)

    # 2. 将支撑杆件模型化并配对
    s_models = build_support_models(support)
    if not s_models:  # 如果没有支撑杆，无法继续
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

    # 3. 对支撑杆件强制关于 x_center 对称
    processed_supports = set()
    for l_id, r_id in pairs.items():
        if l_id in processed_supports or r_id in processed_supports:
            continue

        l_model = next(m for m in s_models if m.id == l_id)
        r_model = next(m for m in s_models if m.id == r_id)

        # 斜率取平均绝对值
        if l_model.k is not None and r_model.k is not None:
            avg_k_mag = (abs(l_model.k) + abs(r_model.k)) / 2.0
            new_lk, new_rk = -avg_k_mag, avg_k_mag
        else:  # 保持垂直状态
            new_lk, new_rk = l_model.k, r_model.k

        # 位置(中点)关于 x_center 对称
        l_dist = x_center - l_model.x_mid()
        r_dist = r_model.x_mid() - x_center
        avg_dist = (l_dist + r_dist) / 2.0
        new_l_x_mid, new_r_x_mid = x_center - avg_dist, x_center + avg_dist

        # 重建对称的杆件坐标
        for mid, model, new_k, new_x_mid in [(l_id, l_model, new_lk, new_l_x_mid),
                                             (r_id, r_model, new_rk, new_r_x_mid)]:
            p1_old, p2_old = support[mid]
            y_min, y_max = min(p1_old[1], p2_old[1]), max(p1_old[1], p2_old[1])
            y_mid = (y_min + y_max) / 2.0

            # 使用点斜式反算端点x坐标
            if new_k is None or abs(new_k) < 1e-9:  # 垂直线
                x_at_min, x_at_max = new_x_mid, new_x_mid
            else:
                x_at_min = new_x_mid + (y_min - y_mid) / new_k
                x_at_max = new_x_mid + (y_max - y_mid) / new_k
            support[mid] = [(x_at_min, y_min), (x_at_max, y_max)]

        processed_supports.add(l_id)
        processed_supports.add(r_id)

    # 处理中心支撑杆件，强制其 x 坐标为 x_center
    for mid_id in unpaired_ids:
        p1, p2 = support[mid_id]
        y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
        support[mid_id] = [(x_center, y_min), (x_center, y_max)]

    # 4. 重新构建对称化的支撑模型，并重新投影横向杆件
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
# 新增：强制所有支撑顶端对齐至最高横杆
# ================================
def align_to_top(support: CoordDict,
                                     horizontal: CoordDict) -> CoordDict:
    """
    将一个视图内的所有支撑杆件进行“收顶”处理。

    - 找到视图中最高的横向杆件，确定其Y坐标为 y_top_target。
    - 遍历所有支撑杆件，保持其底部端点和斜率不变。
    - 重新计算每个支撑杆件的顶部端点，使其Y坐标精确地落在 y_top_target 上。

    Args:
        support: 视图的支撑杆件字典。
        horizontal: 视图的横向杆件字典。

    Returns:
        修正后的支撑杆件字典。
    """
    # 如果没有支撑或横杆，则无需处理
    if not support or not horizontal:
        return support

    # 1. 找到最高横杆的Y坐标
    top_y, top_key = None, None
    for hid, seg in horizontal.items():
        y_mean = (seg[0][1] + seg[1][1]) / 2.0
        if top_y is None or y_mean < top_y:
            top_y, top_key = y_mean, hid

    # 如果未能找到最高横杆，也直接返回
    if top_key is None:
        return support

    # 最高横杆的Y坐标就是我们的目标高度
    y_top_target = (horizontal[top_key][0][1] + horizontal[top_key][1][1]) / 2.0

    # 2. 遍历并修正所有支撑杆件
    new_support = {}
    for gid, seg in support.items():
        p1, p2 = seg

        # a. 识别底部和顶部端点 (Y坐标大的为底部)
        if p1[1] > p2[1]:
            p_bottom, p_top = p1, p2
        else:
            p_bottom, p_top = p2, p1

        # b. 建立直线模型以保持斜率
        line = Line(p1, p2, str(gid))

        # c. 计算在目标高度y_top_target处的新x坐标
        # 如果是垂直线，x坐标不变
        if line.vertical_x is not None:
            new_top_x = line.vertical_x
        else:
            new_top_x = line.x_at(y_top_target)

        # d. 创建新的顶部端点
        p_top_new = (new_top_x, y_top_target)

        # e. 保存修正后的杆件
        new_support[gid] = [p_top_new, p_bottom]

    return new_support

# ====== Reconstruct3D (原 reconstruct3d.py) ======
# reconstruct3d.py — 三维重建算法
from typing import Dict, List, Tuple, Optional
import math

# 类型别名已在文件开头定义

# ------------- 文档法：面1/面2角度映射（你的图公式） -------------

def _mid(ptA: Coord, ptB: Coord) -> Tuple[float,float]:
    return ((float(ptA[0]) + float(ptB[0]))/2.0, (float(ptA[1]) + float(ptB[1]))/2.0)

def _clip(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))

def _select_top_horizontal(view_h: CoordDict, axis: str = "x") -> Optional[Tuple[Coord, Coord]]:
    """
    取“最高”的横向杆件（z 最小）；返回左右端点（按 x/y 排序）
    axis = 'x' 用于正面，'y' 用于侧面
    """
    if not view_h:
        return None
    items = list(view_h.items())
    # 以端点 z 的平均决定“高度”
    items.sort(key=lambda kv: (kv[1][0][1] + kv[1][1][1]) / 2.0)  # z 小（更上）在前
    seg = items[0][1]
    (xa, za), (xb, zb) = (seg[0], seg[1])
    if axis == "x":
        return ( (xa,za), (xb,zb) ) if xa <= xb else ( (xb,zb), (xa,za) )
    else:  # axis == 'y'
        ya, yb = xa, xb  # 这里 seg 中的第一个元素是 y（侧面坐标是 (y,z) ）
        if ya <= yb:
            return ( (xa,za), (xb,zb) )
        else:
            return ( (xb,zb), (xa,za) )

def _bottom_from_support(view_support: CoordDict, axis: str = "x") -> Optional[Tuple[Coord, Coord]]:
    """
    从两根支撑的“最低端点”中选出下底的左右端点（按 x/y 排序）
    """
    if not view_support:
        return None
    bottom_pts: List[Coord] = []
    for seg in view_support.values():
        (x1,z1),(x2,z2) = seg
        # 取“低端点”（z 向下为正 => 取 z 较大的端点）
        if float(z1) >= float(z2):
            bottom_pts.append((float(x1), float(z1)))
        else:
            bottom_pts.append((float(x2), float(z2)))
    if len(bottom_pts) < 2:
        return None
    # 按 x 或 y 选“最左/最右”
    if axis == "x":
        bottom_pts.sort(key=lambda p: p[0])  # x
    else:
        bottom_pts.sort(key=lambda p: p[0])  # 侧面存的是 (y,z)，此处 p[0] 即 y
    left, right = bottom_pts[0], bottom_pts[-1]
    return (left, right)



def extract_bases_front(front_support: CoordDict, front_horizontal: CoordDict):
    """
    修正版 V2：不仅使用支撑顶点，还增加【中心对齐过滤】。
    剔除偏离塔身中心轴过远的噪点（如长横担端点）。
    """
    # 1. 获取底部基准点
    bottom = _bottom_from_support(front_support, axis="x")
    if bottom is None:
        return None
    (x4, z4), (x3, z3) = bottom  # bottom 函数返回已排序：左下(4), 右下(3)
    
    # 计算底部中心线 X 坐标和底部宽度
    bottom_center_x = (x3 + x4) / 2.0
    bottom_width = abs(x3 - x4)
    
    # 2. 收集所有支撑杆件的【顶点】
    candidates = []
    for seg in front_support.values():
        (px1, pz1), (px2, pz2) = seg
        # 取 Z 值较小（较高）的端点
        top_pt = (px1, pz1) if pz1 < pz2 else (px2, pz2)
        candidates.append(top_pt)
            
    if not candidates:
        return None

    # 3. 【关键修正】：过滤离谱的噪点
    # 规则：合法的顶部点，其偏离中心的距离，不应远大于底部半径的某个倍数（防止抓到横担端点）
    # 对于塔身，顶部宽度通常 <= 底部宽度。放宽到 1.5 倍以防万一。
    valid_tops = []
    threshold = (bottom_width / 2.0) * 2.5  # 阈值系数可微调，2.5倍通常足够容纳塔身但排除长横担
    
    for pt in candidates:
        dist = abs(pt[0] - bottom_center_x)
        if dist < threshold:
            valid_tops.append(pt)
            
    # 如果过滤太狠导致没点了，就回退到使用原始点（虽然可能是错的，但比崩溃好）
    if len(valid_tops) < 2:
        valid_tops = candidates

    # 4. 排序策略：
    # 优先按 Z (高度) 升序，Z 相同时按 X 升序
    valid_tops.sort(key=lambda p: (p[1], p[0]))
    
    # 取 Z 最小（最高）的两个点。
    # 经过上面的过滤，这里的“最高”应该是塔腿顶部，而不是避雷针或横担
    if len(valid_tops) < 2:
        return None
        
    t1, t2 = valid_tops[0], valid_tops[1]
    
    # 确保 t1 是左边，t2 是右边
    if t1[0] > t2[0]:
        t1, t2 = t2, t1
        
    (x1, z1), (x2, z2) = t1, t2

    # --- 调试打印 (可选) ---
    # print(f"DEBUG: Bottom Center={bottom_center_x}, Filter Threshold={threshold}")
    # print(f"DEBUG: Rejected pts={[p for p in candidates if p not in valid_tops]}")
    # print(f"DEBUG: Selected Top={t1, t2}")

    return ((float(x1), float(z1)), (float(x2), float(z2)),
            (float(x3), float(z3)), (float(x4), float(z4)))


def extract_bases_side(right_support: CoordDict, right_horizontal: CoordDict):
    """
    修正版 V2：增加【中心对齐过滤】，逻辑同 Front。
    """
    # 1. 获取底部基准点
    bottom = _bottom_from_support(right_support, axis="y")
    if bottom is None:
        return None
    (y7, z7), (y8, z8) = bottom
    
    bottom_center_y = (y7 + y8) / 2.0
    bottom_width = abs(y8 - y7)

    # 2. 收集顶点
    candidates = []
    for seg in right_support.values():
        (py1, pz1), (py2, pz2) = seg
        top_pt = (py1, pz1) if pz1 < pz2 else (py2, pz2)
        candidates.append(top_pt)
            
    if not candidates:
        return None

    # 3. 【关键修正】：过滤噪点
    valid_tops = []
    threshold = (bottom_width / 2.0) * 2.5 
    
    for pt in candidates:
        dist = abs(pt[0] - bottom_center_y) # 这里 pt[0] 是 y
        if dist < threshold:
            valid_tops.append(pt)
            
    if len(valid_tops) < 2:
        valid_tops = candidates

    # 4. 排序与选择
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
    """
    计算：
    - z_topF = (z1+z2)/2, z_topS = (z5+z6)/2
    - h_front, h_side （按图2/图3的“中点距离”公式）
    - θ, δ （按图4/图5的 cos 公式；实现用 arccos+裁剪）
    """
    (x1,z1),(x2,z2),(x3,z3),(x4,z4) = front_bases
    (y5,z5),(y6,z6),(y7,z7),(y8,z8) = side_bases

    # 顶面 z 平均
    z_topF = (z1 + z2)/2.0
    z_topS = (z5 + z6)/2.0

    # 高度（等腰梯形：上下底中点的欧氏距离）
    xm_top, zm_top   = (x1 + x2)/2.0, (z1 + z2)/2.0
    xm_bottom, zm_bottom = (x3 + x4)/2.0, (z3 + z4)/2.0
    h_front = math.hypot(xm_top - xm_bottom, zm_top - zm_bottom)

    ym_top, zm_top_s   = (y5 + y6)/2.0, (z5 + z6)/2.0
    ym_bottom, zm_bottom_s = (y7 + y8)/2.0, (z7 + z8)/2.0
    h_side = math.hypot(ym_top - ym_bottom, zm_top_s - zm_bottom_s)

    eps = 1e-9
    h_front = max(h_front, eps)
    h_side  = max(h_side, eps)

    # 角度（按你的图公式）
    # θ：用侧面上/下底长度差（|y8-y7| - |y6-y5|） / (2 h_front)
    cos_theta = _clip( (abs(y8 - y7) - abs(y6 - y5)) / (2.0 * h_front) )
    theta = math.acos(cos_theta)

    # δ：用正面上/下底长度差（|x3-x4| - |x1-x2|） / (2 h_side)
    cos_delta = _clip( (abs(x3 - x4) - abs(x1 - x2)) / (2.0 * h_side) )
    delta = math.acos(cos_delta)

    return (z_topF, z_topS, h_front, h_side, theta, delta)

def _norm_front_point(x: float, z: float, x4: float, z_topF: float) -> Tuple[float,float]:
    # 归一化：x_f'' = x - x4； z1'' = z - z_topF（向下为正，取非负）
    xf2 = float(x) - float(x4)
    z1p = float(z) - float(z_topF)
    return xf2, z1p

def _norm_side_point(y: float, z: float, y7: float, z_topS: float) -> Tuple[float,float]:
    # 归一化：y_s'' = y - y7； z2'' = z - z_topS（向下为正，取非负）
    ys2 = float(y) - float(y7)
    z2p = float(z) - float(z_topS)
    return ys2, z2p

def sort_bases(bases):

    bottom_left, bottom_right, top_a, top_b = bases

    # 判断顶部左右
    if top_a[0] < top_b[0]:
        top_left = top_a
        top_right = top_b
    else:
        top_left = top_b
        top_right = top_a

    # 判断底部左右
    if bottom_left[0] > bottom_right[0]:
        bottom_left, bottom_right = bottom_right, bottom_left

    return bottom_left, bottom_right, top_left, top_right

def reconstruct3d_front(front_total: CoordDict,
                        front_support: CoordDict,
                        front_horizontal: CoordDict,
                        side_bases) -> Dict[str, List[Point3D]]:
    """
    修正版 V3：基于中心轴的投影重建。
    不再计算复杂的倾角，而是直接根据侧视图的宽度，将正视图投影到 3D 空间的前平面。
    """
    # 1. 获取几何基准
    fbases = extract_bases_front(front_support, front_horizontal)
    # 注意：这里我们使用传进来的 side_bases，确保高度匹配
    if not fbases or not side_bases: 
        return {}
    
    (x1, z1), (x2, z2), (x3, z3), (x4, z4) = fbases
    (y5, z5), (y6, z6), (y7, z7), (y8, z8) = side_bases

    
    # 2. 计算正面视图的中心轴 X 坐标
    # 使用底部中心作为最稳健的参考
    Cx = (x3 + x4) / 2.0
    
    # 3. 计算侧视图在不同高度的“深度”（即塔身的厚度）
    # 侧面顶部半宽
    W_top_s = abs(y6 - y5) / 2.0
    # 侧面底部半宽
    W_bot_s = abs(y8 - y7) / 2.0
    
    # # 高度参考 (Z轴)
    # Z_top = z5  # 侧面顶部 Z
    # Z_bot = z7  # 侧面底部 Z
    # Height = Z_bot - Z_top
        # 🔥 修复：必须使用正面自己的 Z 边界来计算比例！
    Z_top = (z1 + z2) / 2.0
    Z_bot = (z3 + z4) / 2.0
    Height = Z_bot - Z_top
    
    out: Dict[str, List[Point3D]] = {}
    
    for gid, seg in front_total.items():
        pts: List[Point3D] = []
        for (x, z) in seg:
            # --- X 坐标重建 ---
            # 直接相对于中心轴偏移
            X = x - Cx
            
            # --- Y 坐标重建 (深度) ---
            # 根据当前点的 Z 值，线性插值计算该高度下塔身的深度
            if abs(Height) < 1e-4:
                # 防止除以零（虽然不太可能）
                current_depth = W_top_s
            else:
                ratio = (z - Z_top) / Height
                current_depth = W_top_s + ratio * (W_bot_s - W_top_s)
            
            # 正面视图的数据，通常放置在塔身的“前面板”
            # 在 3D 坐标系中，设为 -Y 方向 (深度为负)
            Y = -current_depth
            
            # --- Z 坐标 ---
            # 保持原始 Z (后续流程会统一平移)
            Z = z
            # print("DEBUG front seg:", gid, seg)
            pts.append((X, Y, Z))
        out[f"F_{gid}"] = pts
    
    return out


def reconstruct3d_right(right_total: CoordDict,
                        right_support: CoordDict,
                        right_horizontal: CoordDict,
                        front_bases) -> Dict[str, List[Point3D]]:
    """
    修正版 V3：基于中心轴的投影重建。
    根据正视图的宽度，将侧视图投影到 3D 空间的右平面。
    """
    # 1. 获取几何基准
    sbases = extract_bases_side(right_support, right_horizontal)
    if not sbases or not front_bases: 
        return {}
    
    (x1, z1), (x2, z2), (x3, z3), (x4, z4) = front_bases
    (y5, z5), (y6, z6), (y7, z7), (y8, z8) = sbases


    # 2. 计算侧面视图的中心轴 Y 坐标
    Cy = (y7 + y8) / 2.0
    
    # 3. 计算正视图在不同高度的“宽度”
    # 正面顶部半宽
    W_top_f = abs(x2 - x1) / 2.0
    # 正面底部半宽
    W_bot_f = abs(x3 - x4) / 2.0
    
    # # 高度参考
    # Z_top = z1
    # Z_bot = z3
    # Height = Z_bot - Z_top
        # 🔥 修复：必须使用侧面自己的 Z 边界来计算比例！
    Z_top = (z5 + z6) / 2.0
    Z_bot = (z7 + z8) / 2.0
    Height = Z_bot - Z_top
    
    out: Dict[str, List[Point3D]] = {}
    
    for gid, seg in right_total.items():
        pts: List[Point3D] = []
        for (y, z) in seg:
            # --- Y 坐标重建 ---
            # 相对于侧面中心轴偏移
            Y = y - Cy
            
            # --- X 坐标重建 (宽度) ---
            # 根据当前点的 Z 值，插值计算该高度下塔身的宽度（决定了侧面的 X 位置）
            if abs(Height) < 1e-4:
                current_width = W_top_f
            else:
                ratio = (z - Z_top) / Height
                current_width = W_top_f + ratio * (W_bot_f - W_top_f)
            
            # 侧面视图的数据，放置在塔身的“右面板”
            # 在 3D 坐标系中，设为 +X 方向
            X = current_width
            
            # --- Z 坐标 ---
            Z = z

            # print("DEBUG right seg:", gid, seg)
            
            pts.append((X, Y, Z))
        out[f"R_{gid}"] = pts
    return out






# ====== Splicing (原 splicing.py) ======
# splicing.py - 模型拼接功能模块 (已修正为基于端点的精确对齐)
import math
import numpy as np
from typing import Dict, List, Tuple, Set

# 类型别名已在文件开头定义
AllModelsData: TypeAlias = Dict[str, Dict[str, Any]]


def _find_splicing_points(f3d: Model3DData, support_keys: Set[str], mode: str) -> List[Point3D]:
    """
    在模型正视图的三维数据中，找到支撑杆件最上/最下的那一层中的左右两个端点。
    - f3d: 正视图的三维数据字典
    - support_keys: 正视图中哪些ID是支撑杆件
    - mode: 'top' 或 'bottom'
    
    关键修复：
    1. 收集所有支撑杆件端点
    2. 按Z坐标排序，找到最上/最下的那一层（Z值相近的点）
    3. 在该层中找到左右两个最外侧的点（X坐标最小和最大）
    4. 返回这两个点作为连接点
    """
    support_endpoints = []
    for gid, seg in f3d.items():
        original_id = gid.replace("F_", "")
        if original_id in support_keys:
            support_endpoints.extend(seg)

    if not support_endpoints:
        raise ValueError("在模型中未能找到任何支撑杆件的端点。")

    # 按Z坐标排序
    reverse_sort = (mode == 'bottom')
    support_endpoints.sort(key=lambda p: p[2], reverse=reverse_sort)

    if len(support_endpoints) < 2:
        raise ValueError("支撑杆件端点少于2个，无法执行拼接。")

    # 找到最上/最下的那一层（Z值相近的点）
    z_ref = support_endpoints[0][2]
    z_tolerance = 1.0  # Z值在1mm以内认为是同一层
    
    same_z_layer = [p for p in support_endpoints if abs(p[2] - z_ref) < z_tolerance]
    
    if len(same_z_layer) < 2:
        # 如果同一层点数不足2个，就用最上/最下的两个点
        same_z_layer = support_endpoints[:2]
    
    # 在该层中按X坐标排序，找到左右两个最外侧的点
    same_z_layer.sort(key=lambda p: p[0])
    
    # 返回左右两个最外侧的点
    left_point = same_z_layer[0]
    right_point = same_z_layer[-1]
    
    print(f"  - 拼接点 ({mode}): 左={left_point}, 右={right_point}")
    
    return [left_point, right_point]


def get_rotation_matrix(v1, v2):
    """ 计算从向量v1到v2的旋转矩阵R，使得 R*v1 与 v2 平行 """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # 如果向量已经平行或反向平行，则无需旋转或旋转180度
    if np.allclose(v1, v2):
        return np.identity(3)
    if np.allclose(v1, -v2):
        return -np.identity(3)

    # 使用向量叉乘计算旋转轴和角度
    cross_prod = np.cross(v1, v2)
    dot_prod = np.dot(v1, v2)

    s = np.linalg.norm(cross_prod)
    c = dot_prod

    vx = np.array([
        [0, -cross_prod[2], cross_prod[1]],
        [cross_prod[2], 0, -cross_prod[0]],
        [-cross_prod[1], cross_prod[0], 0]
    ])

    # 罗德里格旋转公式
    rotation_matrix = np.identity(3) + vx + vx.dot(vx) * ((1 - c) / (s ** 2))
    return rotation_matrix


def _align_and_transform_model(model_data: Dict[str, Model3DData],
                               source_p1: Point3D, source_p2: Point3D,
                               target_p1: Point3D, target_p2: Point3D):
    """
    对模型进行完整的刚体变换（平移、缩放、旋转），
    使得源点对(source_p1, source_p2)精确映射到目标点对(target_p1, target_p2)。
    """
    # 将输入点转换为numpy数组
    A1 = np.array(source_p1)
    A2 = np.array(source_p2)
    B1 = np.array(target_p1)
    B2 = np.array(target_p2)

    # 1. 计算缩放因子
    dist_A = np.linalg.norm(A2 - A1)
    dist_B = np.linalg.norm(B2 - B1)
    if dist_A < 1e-9:
        raise ValueError("源模型的连接点距离过近，无法进行变换。")
    scale = dist_B / dist_A
    print(f"  - 计算缩放因子: {scale:.4f}")

    # 2. 计算旋转矩阵
    vec_A = A2 - A1
    vec_B = B2 - B1
    rotation_matrix = get_rotation_matrix(vec_A, vec_B)
    print("  - 已计算旋转矩阵。")

    # 3. 对模型中每个点应用变换
    for view_data in model_data.values():  # 遍历 f3d 和 r3d
        for gid in view_data:
            transformed_seg = []
            for p_tuple in view_data[gid]:
                P = np.array(p_tuple)
                # a. 相对化: 将点P平移到以A1为原点的坐标系
                P_relative = P - A1
                # b. 缩放
                P_scaled = P_relative * scale
                # c. 旋转
                P_rotated = rotation_matrix.dot(P_scaled)
                # d. 绝对化: 平移到以B1为原点的目标坐标系
                P_new = P_rotated + B1
                transformed_seg.append(tuple(P_new))
            view_data[gid] = transformed_seg
    print("  - 已对附加模型应用完整的对齐变换。")


def splice_models(
    all_models_data: AllModelsData,
    auto_base_index: Optional[int] = None,
) -> Tuple[Model3DData, Model3DData]:
    """
    执行模型拼接的主函数（使用修正后的端点对齐算法）。
    """
    model_names = sorted(all_models_data.keys())
    print("\n" + "=" * 50)
    print("模型拼接流程启动 (精确端点对齐模式)")
    print("=" * 50)
    print("当前已处理的模型如下：")
    for i, name in enumerate(model_names):
        print(f"  {i + 1}: {name}")

    base_model_idx = -1
    if auto_base_index is not None and 0 <= auto_base_index < len(model_names):
        base_model_idx = auto_base_index
        print(f"自动选择基准模型序号 {auto_base_index + 1}")

    while base_model_idx < 0 or base_model_idx >= len(model_names):
        try:
            choice = input(f"请输入您想选择的基准模型的序号 (1-{len(model_names)}): ")
            idx = int(choice) - 1
            if 0 <= idx < len(model_names):
                base_model_idx = idx
            else:
                print("错误：序号超出范围，请重新输入。")
        except ValueError:
            print("错误：请输入一个有效的数字。")

    base_model_name = model_names.pop(base_model_idx)
    print(f"\n您选择了 '{base_model_name}' 作为基准模型。")

    cumulative_f3d = all_models_data[base_model_name]['f3d'].copy()
    cumulative_r3d = all_models_data[base_model_name]['r3d'].copy()

    previous_model_data = all_models_data[base_model_name]
    remaining_models_to_splice = model_names
    print("将按照以下顺序进行拼接：", " -> ".join([base_model_name] + remaining_models_to_splice))

    for attach_model_name in remaining_models_to_splice:
        print(f"\n--- 正在拼接: '{previous_model_data['name']}' + '{attach_model_name}' ---")

        # 1. 找到前续模型的底部连接点对
        base_points = _find_splicing_points(
            previous_model_data['f3d'],
            previous_model_data['front_support_keys'],
            mode='bottom'
        )
        base_points.sort(key=lambda p: p[0])  # 按X坐标排序，区分左右
        base_left, base_right = base_points[0], base_points[1]
        print(f"  - 基准连接点 (左/右): {base_left}, {base_right}")

        # 2. 找到附加模型的顶部连接点对
        attach_model_data = all_models_data[attach_model_name]
        attach_points_orig = _find_splicing_points(
            attach_model_data['f3d'],
            attach_model_data['front_support_keys'],
            mode='top'
        )
        attach_points_orig.sort(key=lambda p: p[0])  # 按X坐标排序，区分左右
        attach_left, attach_right = attach_points_orig[0], attach_points_orig[1]
        print(f"  - 附加连接点 (底面接头): {attach_left}, {attach_right}")

        # 3. 应用精确的对齐变换
        current_attach_model_3d = {'f3d': attach_model_data['f3d'].copy(), 'r3d': attach_model_data['r3d'].copy()}
        _align_and_transform_model(
            current_attach_model_3d,
            source_p1=attach_left, source_p2=attach_right,
            target_p1=base_left, target_p2=base_right
        )

        # 4. 合并数据
        cumulative_f3d.update(current_attach_model_3d['f3d'])
        cumulative_r3d.update(current_attach_model_3d['r3d'])

        # 5. 更新“前续模型”为当前附加的模型，为下一次拼接做准备
        previous_model_data = {
            'name': attach_model_name,
            'f3d': current_attach_model_3d['f3d'],
            'r3d': current_attach_model_3d['r3d'],
            'front_support_keys': attach_model_data['front_support_keys']
        }

    print("\n" + "=" * 50)
    print("所有模型拼接完成！")
    print("=" * 50)

    return cumulative_f3d, cumulative_r3d

# ====== Final Output (原 generate_final_output.py) ======
# generate_final_output.py
import json
import numpy as np
from typing import Dict, List, Tuple

# 类型别名已在文件开头定义
CoordMap: TypeAlias = Dict[str, Seg3D]
UniqueNodeDict: TypeAlias = Dict[Point3D, Dict[str, Any]]

TOLERANCE = 1e-4


class UniqueNodeIdentifier:
    """
    核心类，负责创建全局唯一的节点字典，并提供ID查询功能。
    - 所有ID均作为字符串处理。
    """

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
            # 修改点: 直接生成字符串ID
            base_id = f"{id_prefix_str}{owner['suffix']}"

            self.base_coord_to_info[found_node_coord] = {"id": base_id, "is_support": is_support_node}

    def get_node_id(self, point: Point3D) -> str:
        theoretical_base_coord = self._get_base_coord_q3(point)
        if not self.base_coord_to_info: return "-1"

        actual_base_coord = min(self.base_coord_to_info.keys(),
                                key=lambda bc: np.linalg.norm(np.array(bc) - np.array(theoretical_base_coord)))

        info = self.base_coord_to_info.get(actual_base_coord)
        if info is None: return "-1"

        # 修改点: 所有ID操作基于字符串
        base_id_str = info['id']
        x, y, z = point
        is_right = abs(x) > TOLERANCE and x > 0
        is_front = abs(y) > TOLERANCE and y > 0

        # 将字符串转为整数进行计算，再转回字符串
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
            return base_id_str  # 如果ID不是纯数字，直接返回



# === Paste the following into core.py (replace the old generate_outputs and add helpers) ===
from typing import Dict, List, Tuple
import math

# 类型别名已在文件开头定义

# ---------- helpers (small, internal) ----------
# ===== helper: 构建 pinjie（正面视图所有横向杆件端点，绝对坐标；沿用 ID 复用规则；按 Z↑, X↑ 排序） =====
def _build_pinjie_from_front_horiz(final_coords_map: dict,
                                   all_models_data: dict,
                                   ganjian: list) -> list:
    """
    返回 pinjie: List[[node_id(str), [x,y,z]]]
    规则：
      - 仅收集“正面视图”的所有横向杆件端点；
      - 端点 ID 严格沿用 ganjian 中该杆件端点的 node1_id/node2_id（从而继承复用规则）；
      - 输出绝对坐标（final_coords_map 中的 3D 坐标）；
      - 去重：同一 node_id 只保留一次；
      - 排序：按 Z 升序，再按 X 升序。
    """
    # 1) 取出所有“正面横向杆件”的原始 ID（未带拼接后缀）
    front_h_ids = set()
    for m in (all_models_data or {}).values():
        ga = (m or {}).get('ganjian_args', {}) or {}
        fh = ga.get('front_horizontal') or {}
        for mid in fh.keys():
            front_h_ids.add(str(mid))

    if not front_h_ids:
        return []

    # 2) 建立 “member_id -> (node1_id, node2_id)” 映射（来自 ganjian）
    m_to_nodes = {}
    for g in (ganjian or []):
        mid = str(g.get('member_id'))
        n1  = str(g.get('node1_id'))
        n2  = str(g.get('node2_id'))
        if mid and (n1 is not None) and (n2 is not None):
            m_to_nodes[mid] = (n1, n2)

    # # 3) 将 final_coords_map 中与 front_h_ids 匹配的（考虑拼接产生的后缀 _1/_2…）都加入
    # #    匹配方式：以 “去掉后缀的基 ID = k.split('_')[0]” 与 front_h_ids 比较
    # def _base_id(k: str) -> str:
    #     return str(k).split('_', 1)[0]
    # 找到 _base_id 方法
    def _base_id(k: str) -> str:
        # 新增去除 F_ 和 R_ 的安全过滤
        k = str(k).replace("F_", "").replace("R_", "")
        return k.split('_', 1)[0]

    # 4) 收集端点：按 ganjian 的 node1_id / node2_id 赋 ID
    id_to_coord = {}  # 去重：node_id -> (x,y,z)
    for k, seg in (final_coords_map or {}).items():
        base = _base_id(k)
        if base not in front_h_ids:
            continue
        if not isinstance(seg, (list, tuple)) or len(seg) != 2:
            continue

        # 从 ganjian 拿同名 member 的端点 ID；
        # 优先找完整 key（含后缀），找不到再用 base 尝试一次。
        node_ids = m_to_nodes.get(str(k))
        if node_ids is None:
            node_ids = m_to_nodes.get(base)
        if node_ids is None:
            # 如果 ganjian 中没有这个 member（极少数边缘情况），就跳过，避免污染 pinjie
            continue

        (n1, n2) = node_ids
        p1, p2 = seg  # 绝对坐标

        # 去重按 node_id；同一个 ID 出现多次只保留一次
        if n1 not in id_to_coord:
            id_to_coord[n1] = [float(p1[0]), float(p1[1]), float(p1[2])]
        if n2 not in id_to_coord:
            id_to_coord[n2] = [float(p2[0]), float(p2[1]), float(p2[2])]

    # 5) 排序：Z 升序，再 X 升序
    items = list(id_to_coord.items())
    items.sort(key=lambda kv: (kv[1][2], kv[1][0]))

    # 6) 组装 pinjie 结构
    pinjie = [[str(node_id), coord] for node_id, coord in items]
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
    if sym_type == 1:   # 左右
        return (-x, y, z)
    if sym_type == 2:   # 前后
        return (x, -y, z)
    if sym_type == 3:   # 中心
        return (-x, -y, z)
    return pt           # 4: 自身（保持不动）

def _dist3(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def _sort_lr(p1: Point3D, p2: Point3D) -> Tuple[Point3D, Point3D]:
    # 以 x 为主，y 为次，保证 (left, right)
    if (p1[0], p1[1]) <= (p2[0], p2[1]):
        return p1, p2
    return p2, p1

def _top_bottom(p1: Point3D, p2: Point3D) -> Tuple[Point3D, Point3D]:
    # z 越小越靠上
    if p1[2] <= p2[2]:
        return p1, p2
    return p2, p1

def _closest_id(pt: Point3D, known: Dict[str, Point3D], eps=1e-6) -> str:
    for k, v in known.items():
        if _dist3(pt, v) <= eps:
            return k
    return ""





def generate_outputs(final_coords_map: Dict[str, List[Point3D]], all_models_data: Dict[str, dict]):
    """
    终极修正版：直接输出绝对 3D 坐标 (node_type: 11)。
    废弃导致坐标串位和抹杀锥度的 node_type: 12 引用写法。
    """
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

        # === 1) 基准支撑提取 ===
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
        sid10 = f"{base_sid}10"
        sid20 = f"{base_sid}20"

        known_nodes: Dict[str, Point3D] = {
            sid10: topP, sid20: botP,
            _plus_suffix(sid10, +1): _sym_pt(topP, 1),
            _plus_suffix(sid10, +2): _sym_pt(topP, 2),
            _plus_suffix(sid10, +3): _sym_pt(topP, 3),
            _plus_suffix(sid20, +1): _sym_pt(botP, 1),
            _plus_suffix(sid20, +2): _sym_pt(botP, 2),
            _plus_suffix(sid20, +3): _sym_pt(botP, 3),
        }

        # 输出支撑基准节点 (Type 11)
        jiedian.append({"node_id": sid10, "node_type": 11, "X": round(topP[0],3), "Y": round(topP[1],3), "Z": round(topP[2],3), "symmetry_type": 4})
        jiedian.append({"node_id": sid20, "node_type": 11, "X": round(botP[0],3), "Y": round(botP[1],3), "Z": round(botP[2],3), "symmetry_type": 4})
        ganjian.append({"member_id": base_sid, "node1_id": sid10, "node2_id": sid20, "symmetry_type": 4})

        # === 修复：输出所有其他支撑杆件 ===
        for sid in front_support.keys():
            if str(sid) == base_sid:
                continue  # 跳过已处理的基准支撑
            # seg3d = final_coords_map.get(str(sid))
            seg3d = final_coords_map.get(f"F_{sid}") 
            if not seg3d:
                continue
            pA, pB = seg3d
            topP_other, botP_other = _top_bottom(pA, pB)
            sid10_other = f"{sid}10"
            sid20_other = f"{sid}20"
            
            # 检查是否与已知节点重合（复用）
            reuse_top = _closest_id(topP_other, known_nodes, eps=EPS)
            reuse_bot = _closest_id(botP_other, known_nodes, eps=EPS)
            
            if not reuse_top:
                jiedian.append({"node_id": sid10_other, "node_type": 11, "X": round(topP_other[0],3), "Y": round(topP_other[1],3), "Z": round(topP_other[2],3), "symmetry_type": 4})
                known_nodes[sid10_other] = topP_other
                known_nodes[_plus_suffix(sid10_other, +1)] = _sym_pt(topP_other, 1)
                known_nodes[_plus_suffix(sid10_other, +2)] = _sym_pt(topP_other, 2)
                known_nodes[_plus_suffix(sid10_other, +3)] = _sym_pt(topP_other, 3)
                node1_id = sid10_other
            else:
                node1_id = reuse_top
            
            if not reuse_bot:
                jiedian.append({"node_id": sid20_other, "node_type": 11, "X": round(botP_other[0],3), "Y": round(botP_other[1],3), "Z": round(botP_other[2],3), "symmetry_type": 4})
                known_nodes[sid20_other] = botP_other
                known_nodes[_plus_suffix(sid20_other, +1)] = _sym_pt(botP_other, 1)
                known_nodes[_plus_suffix(sid20_other, +2)] = _sym_pt(botP_other, 2)
                known_nodes[_plus_suffix(sid20_other, +3)] = _sym_pt(botP_other, 3)
                node2_id = sid20_other
            else:
                node2_id = reuse_bot
            
            ganjian.append({"member_id": str(sid), "node1_id": node1_id, "node2_id": node2_id, "symmetry_type": 4})

        # === 修复：输出所有右侧支撑杆件 ===
        for rid in right_support.keys():
            # seg3d = final_coords_map.get(str(rid))
            seg3d = final_coords_map.get(f"R_{rid}")
            if not seg3d:
                continue
            pA, pB = seg3d
            topP_r, botP_r = _top_bottom(pA, pB)
            rid10 = f"{rid}10"
            rid20 = f"{rid}20"
            
            # 检查是否与已知节点重合（复用）
            reuse_top_r = _closest_id(topP_r, known_nodes, eps=EPS)
            reuse_bot_r = _closest_id(botP_r, known_nodes, eps=EPS)
            
            if not reuse_top_r:
                jiedian.append({"node_id": rid10, "node_type": 11, "X": round(topP_r[0],3), "Y": round(topP_r[1],3), "Z": round(topP_r[2],3), "symmetry_type": 4})
                known_nodes[rid10] = topP_r
                known_nodes[_plus_suffix(rid10, +1)] = _sym_pt(topP_r, 1)
                known_nodes[_plus_suffix(rid10, +2)] = _sym_pt(topP_r, 2)
                known_nodes[_plus_suffix(rid10, +3)] = _sym_pt(topP_r, 3)
                node1_id_r = rid10
            else:
                node1_id_r = reuse_top_r
            
            if not reuse_bot_r:
                jiedian.append({"node_id": rid20, "node_type": 11, "X": round(botP_r[0],3), "Y": round(botP_r[1],3), "Z": round(botP_r[2],3), "symmetry_type": 4})
                known_nodes[rid20] = botP_r
                known_nodes[_plus_suffix(rid20, +1)] = _sym_pt(botP_r, 1)
                known_nodes[_plus_suffix(rid20, +2)] = _sym_pt(botP_r, 2)
                known_nodes[_plus_suffix(rid20, +3)] = _sym_pt(botP_r, 3)
                node2_id_r = rid20
            else:
                node2_id_r = reuse_bot_r
            
            ganjian.append({"member_id": str(rid), "node1_id": node1_id_r, "node2_id": node2_id_r, "symmetry_type": 4})

        # === 2) 正面横向 ===
        def _sym1_partner(nid: str) -> str:
            """左右对称(sym_type=1)的伙伴节点: 0↔1, 2↔3"""
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
                # 【关键修复点】：不使用 Type 12 造成坐标串位，直接输出 Type 11 和真实的 X, Y, Z
                left_node_id = f"{hid}10"
                jiedian.append({"node_id": left_node_id, "node_type": 11, "X": round(L[0],3), "Y": round(L[1],3), "Z": round(L[2],3), "symmetry_type": 4})
                known_nodes[left_node_id] = L
                known_nodes[_plus_suffix(left_node_id, +1)] = _sym_pt(L, 1)
                known_nodes[_plus_suffix(left_node_id, +2)] = _sym_pt(L, 2)
                known_nodes[_plus_suffix(left_node_id, +3)] = _sym_pt(L, 3)

            ganjian.append({
                "member_id": str(hid), "node1_id": left_node_id,
                "node2_id": _sym1_partner(left_node_id), "symmetry_type": 2
            })

        # === 3) 右侧横向 ===
        def _sym2_partner(nid: str) -> str:
            """前后对称(sym_type=2)的伙伴节点: 0↔2, 1↔3"""
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
                # 避开横向可能产生的冲突，改为 30
                left_node_id_r = f"{base_id(rid)}30" 
                jiedian.append({"node_id": left_node_id_r, "node_type": 11, "X": round(Lr[0],3), "Y": round(Lr[1],3), "Z": round(Lr[2],3), "symmetry_type": 4})
            # if nid_left_guess:
            #     left_node_id_r = nid_left_guess
            # else:
            #     # 【关键修复点】：同上，改为 Type 11，直接写入坐标
            #     left_node_id_r = f"{base_id(rid)}10" 
            #     jiedian.append({"node_id": left_node_id_r, "node_type": 11, "X": round(Lr[0],3), "Y": round(Lr[1],3), "Z": round(Lr[2],3), "symmetry_type": 4})
                known_nodes[left_node_id_r] = Lr
                known_nodes[_plus_suffix(left_node_id_r, +1)] = _sym_pt(Lr, 1)
                known_nodes[_plus_suffix(left_node_id_r, +2)] = _sym_pt(Lr, 2)
                known_nodes[_plus_suffix(left_node_id_r, +3)] = _sym_pt(Lr, 3)

            ganjian.append({
                "member_id": str(rid), "node1_id": left_node_id_r,
                "node2_id": _sym2_partner(left_node_id_r), "symmetry_type": 1
            })




        # === 4) X 型杆件：按面内分别生成，避免 front/right 跨面串接 ===
        def _x_node_prefix(view_tag: str, xid: str) -> str:
            # 保留视图维度，避免前/右视同一个 xid 产生相同节点前缀
            return f"{view_tag}{str(xid).replace('_', '')}"

        def _register_x_node(preferred_id: str, pt: Point3D) -> str:
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
            jiedian.append({
                "node_id": node_id,
                "node_type": 11,
                "X": round(pt[0], 3),
                "Y": round(pt[1], 3),
                "Z": round(pt[2], 3),
                "symmetry_type": 4,
            })
            known_nodes[node_id] = pt
            return node_id

        def _emit_x_member(view_tag: str, xid: str, seg3d: List[Point3D]):
            if not seg3d or len(seg3d) < 2:
                return
            p1, p2 = _sort_lr(*seg3d)
            node1 = _register_x_node(f"{_x_node_prefix(view_tag, xid)}10", p1)
            node2 = _register_x_node(f"{_x_node_prefix(view_tag, xid)}20", p2)
            # 同一个 xid 在 front/right 两个面内分别生成独立杆件，member_id 也区分开
            member_id = f"{view_tag}_{xid}"

            # 正面X杆件仅前后对称(2)，侧面X杆件仅左右对称(1)
            sym_type = 2 if view_tag == "F" else 1

            ganjian.append({
                "member_id": str(member_id),
                "node1_id": node1,
                "node2_id": node2,
                "symmetry_type": sym_type,
            })

        for xid in sorted(front_x.keys(), key=str):
            fseg = final_coords_map.get(f"F_{xid}")
            if fseg and len(fseg) >= 2:
                _emit_x_member("F", str(xid), fseg)

        for xid in sorted(right_x.keys(), key=str):
            rseg = final_coords_map.get(f"R_{xid}")
            if rseg and len(rseg) >= 2:
                _emit_x_member("R", str(xid), rseg)

        # 拼接数据
        pinjie = _build_pinjie_from_front_horiz(final_coords_map, all_models_data, ganjian)
        
    return ganjian, jiedian, pinjie





# I/O
def drop_vertical_members(
    members: CoordDict,
    span_length: Optional[float],
    dx_ratio: float = 0.01,   # 相对阈值：占顶横长度的 1%
    dx_abs: float = 4.0,      # 绝对阈值：按你坐标量纲可调（如 2~6）
    min_len: float = 5.0,     # 短线不过滤，避免数值抖动
    return_excluded: bool = False,
):
    """
    仅按水平投影 Δx 判定“近竖直”，从 X 候选中剔除：
        竖直判定：|Δx| <= max(dx_abs, dx_ratio * span_length)
    - span_length: 参考尺度（建议用顶横杆长度 seg_len）；None 时只用 dx_abs。
    - min_len: 线段总长 < min_len 不判竖直，直接保留。
    返回：保留字典，或 (保留, 被剔除)（当 return_excluded=True）
    """
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
    """
    构建“上下相邻部件基准支撑端点复用”的别名表：
      - 若 上部件 基准支撑下端点(sid20) 与 下部件 基准支撑上端点(sid10) 在3D中重合(<=eps)；
        则 alias[下部件sid10(+对称1/2/3)] = 上部件sid20(+对称1/2/3)。
    备注：基准支撑选择规则与 generate_outputs 完全一致：每个部件正面支撑中点X最小者。
    """
    parts = []
    # 挑出每个部件的“基准支撑”以及上下端点坐标
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
        topP, botP = _top_bottom(pA, pB)  # z 小者为上端
        parts.append({"stem": stem, "sid": base_sid, "top": topP, "bot": botP})

    aliases: Dict[str, str] = {}
    # 两两检查：上部件的 bot 与 下部件的 top 若重合，则登记 alias(下sid10 -> 上sid20)
    for i in range(len(parts)):
        upper = parts[i]
        for j in range(len(parts)):
            if i == j: 
                continue
            lower = parts[j]
            if _dist3(upper["bot"], lower["top"]) <= float(eps):
                lower_sid10 = f"{lower['sid']}10"
                upper_sid20 = f"{upper['sid']}20"
                # 原始点
                aliases[lower_sid10] = upper_sid20
                # 对称点 (+1/+2/+3)
                aliases[_plus_suffix(lower_sid10, +1)] = _plus_suffix(upper_sid20, +1)
                aliases[_plus_suffix(lower_sid10, +2)] = _plus_suffix(upper_sid20, +2)
                aliases[_plus_suffix(lower_sid10, +3)] = _plus_suffix(upper_sid20, +3)
    return aliases


def apply_id_aliases(ganjian: list, jiedian: list, pinjie: list, id_aliases: Dict[str, str]):
    """
    对三张表进行 ID 正名化（别名替换），并做必要的去重：
      - ganjian: 替换 node1_id/node2_id，去重相同记录
      - jiedian: 
          * 11类：同一 node_id 只保留一条；若 node_id 被映射到其他ID，则输出“正名”ID，丢弃别名ID
          * 12类：node_id 及其 X/Y 参考ID都做别名替换；同一 node_id 只保留一条
      - pinjie: node_id 做别名替换，去重（保留第一次）
    """
    def canon(value: str) -> str:
        """
        将任意节点/引用 ID 规整到最终基准 ID。
        支持带有前缀 '1' 的引用形式：会先解析前缀后面的真实 ID，
        完成别名折叠后再恢复原此前缀。
        """
        def _lift(one: str) -> str:
            seen: Set[str] = set()
            cur = one
            while cur in id_aliases and cur not in seen:
                seen.add(cur)
                cur = id_aliases[cur]
            return cur

        s = str(value)
        # 迭代直到收敛，避免链式映射或前缀替换后仍可继续精简
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
            key = ("12", nid)
            if key in seen_j: 
                continue
            seen_j.add(key)
            new_j.append(r)
        else:  # 11 或其它
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