# main.py
import os, glob, json
import re
from typing import Dict, List, Tuple, Optional, cast, Any
import math
import statistics

# ===== 入口只做流程编排，具体实现放在 core 模块 =====
from dual_view_core import (
    load_and_parse_data, clean_view, find_supports, find_horizontals,
    enforce_symmetry, match_support_slopes, build_support_models,
    plan_top_span, expand_to_top_span, correct_horizontals, align_to_top,
    extract_bases_front, extract_bases_side, reconstruct3d_front, reconstruct3d_right,
    translate_model, center_props, 
    top_xmid_and_range, select_x_type, splice_models,
    generate_outputs, scale_by_member,
    build_vertical_reuse_aliases, apply_id_aliases,
    drop_vertical_members,  # 原有
    remap_vertical_coordinates,
)

# ====== 拼接与定标开关 ======
# 这些开关决定流程在“多模型拼接”和“全局定标”阶段的交互方式，方便交付他人时快速切换模式。
AUTO_SPLICE_SELECTION = False  # True: 自动选择第一个模型作为基准; False: 运行时人工选择
AUTO_SPLICE_BASE_INDEX = 0     # 自动模式下默认选择排序后的第 0 个模型作为塔身地基
AUTO_SCALE_ENABLED = False     # True: 直接按 AUTO_SCALE_MEMBER_* 进行自动定标，跳过人工输入
AUTO_SCALE_MEMBER_ID = "911"   # 自动定标时参考的杆件 ID，不存在则会退回人工模式
AUTO_SCALE_MEMBER_LENGTH = 0.9 # 自动定标时该杆件在真实世界中的长度(米)

# 统一配置中文字体，保证 matplotlib 输出正常
import matplotlib
from matplotlib import font_manager

def _setup_cjk_font():
    candidates = [
        (r"C:\\Windows\\Fonts\\msyh.ttc", "Microsoft YaHei"),
        (r"C:\\Windows\\Fonts\\simhei.ttf", "SimHei"),
        (r"C:\\Windows\\Fonts\\Deng.ttf", "DengXian"),
        (r"/System/Library/Fonts/PingFang.ttc", "PingFang SC"),
        (r"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "Noto Sans CJK SC"),
    ]
    chosen = None
    for path, name in candidates:
        if os.path.exists(path):
            try:
                font_manager.fontManager.addfont(path)
            except Exception:
                pass
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans", "Arial"]
            chosen = name
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
    if chosen:
        print(f"[font] Using CJK font: {chosen}")
    else:
        print("[font] 未找到中文字体，将使用默认字体（中文可能显示为方块）。")

_setup_cjk_font()

# 类型别名（用于模型中心化）
Point3D = Tuple[float, float, float]
Seg3D = List[Point3D]
Model3DData = Dict[str, Seg3D]
Coord = Tuple[float, float]
CoordDict = Dict[str, List[Coord]]

# =====================================================================================
# ====================== 新增：X型修正全流程（在 main.py 内自包含） =====================
# =====================================================================================
class _LineModel:
    __slots__ = ("k","b","vertical_x","ymin","ymax")
    def __init__(self, p1: Coord, p2: Coord):
        x1,y1 = float(p1[0]), float(p1[1])
        x2,y2 = float(p2[0]), float(p2[1])
        self.ymin, self.ymax = (min(y1,y2), max(y1,y2))
        if abs(x1-x2) < 1e-12:
            self.k = None; self.b=None; self.vertical_x = x1
        else:
            self.k = (y2-y1)/(x2-x1); self.b = y1 - self.k*x1; self.vertical_x=None
    def x_at(self, y: float) -> float:
        if self.vertical_x is not None:
            return float(self.vertical_x)
        if self.k is None or abs(self.k) < 1e-12 or self.b is None:
            return float("nan")
        return (float(y) - float(self.b)) / self.k

def _models_from_supports(support_dict: CoordDict) -> List[_LineModel]:
    return [_LineModel(seg[0], seg[1]) for seg in support_dict.values()]

def _extreme_x_at(models: List[_LineModel], y: float) -> Tuple[float,float]:
    xs = [m.x_at(y) for m in models]
    xs = [x for x in xs if math.isfinite(x)]
    if not xs: return (0.0,0.0)
    return (min(xs), max(xs))

def _seg_slope(p1: Coord, p2: Coord) -> float:
    dx = float(p2[0])-float(p1[0]); dy=float(p2[1])-float(p1[1])
    if abs(dx) < 1e-9: return float("inf") if dy>=0 else -float("inf")
    return dy/dx

def _center_y(seg: List[Coord]) -> float:
    return (float(seg[0][1])+float(seg[1][1]))/2.0

def _order_four(segA: List[Coord], segB: List[Coord], xmid: float) -> Tuple[Coord,Coord,Coord,Coord]:
    def LR(p1:Coord,p2:Coord): return (p1,p2) if p1[0]<=p2[0] else (p2,p1)
    A_L,A_R = LR(*segA); B_L,B_R = LR(*segB)
    left_top  = A_L if A_L[1] <= B_L[1] else B_L
    left_bot  = A_L if A_L[1] >  B_L[1] else B_L
    right_top = A_R if A_R[1] <= B_R[1] else B_R
    right_bot = A_R if A_R[1] >  B_R[1] else B_R
    return left_top, right_top, left_bot, right_bot

class _XPair:
    __slots__=("idA","idB","topL","topR","botL","botR","y_top","y_bot","fixed")
    def __init__(self, idA:str, idB:str, topL:Coord, topR:Coord, botL:Coord, botR:Coord):
        self.idA=idA; self.idB=idB
        self.topL, self.topR, self.botL, self.botR = topL, topR, botL, botR
        self.y_top = (topL[1]+topR[1])/2.0
        self.y_bot = (botL[1]+botR[1])/2.0
        self.fixed = {"topL": False, "topR": False, "botL": False, "botR": False}

def _pair_in_view(x_dict: CoordDict, x_mid: Optional[float], overlap_thr: float = 0.65) -> List[_XPair]:
    if not x_dict: return []
    pos=[]; neg=[]
    for sid, seg in x_dict.items():
        if _seg_slope(*seg) >= 0: pos.append((sid, seg))
        else: neg.append((sid, seg))
    pos.sort(key=lambda t:_center_y(t[1]))
    neg.sort(key=lambda t:_center_y(t[1]))
    i=j=0; pairs: List[_XPair]=[]
    def yband(s):
        y1,y2=sorted([s[0][1], s[1][1]]); return y1,y2
    while i<len(pos) and j<len(neg):
        sidA, sA = pos[i]; sidB, sB = neg[j]
        ya1,ya2 = yband(sA); yb1,yb2 = yband(sB)
        inter = max(0.0, min(ya2,yb2) - max(ya1,yb1))
        union = max(ya2,yb2) - min(ya1,yb1)
        overlap = inter/union if union>1e-9 else 0.0
        if overlap >= overlap_thr:
            topL, topR, botL, botR = _order_four(sA, sB, x_mid if x_mid is not None else (_center_y(sA)+_center_y(sB)))
            pairs.append(_XPair(sidA, sidB, topL, topR, botL, botR))
            i+=1; j+=1
        elif _center_y(sA) < _center_y(sB):
            i+=1
        else:
            j+=1
    pairs.sort(key=lambda p: p.y_top)  # y小更高
    return pairs

def _hard_snap_for_view(pairs: List[_XPair], horizontals: CoordDict, support_models: List[_LineModel]):
    if not pairs: return
    # 最高两道横杆
    items = sorted(horizontals.items(), key=lambda kv: (kv[1][0][1]+kv[1][1][1])/2.0)
    H1 = items[0][1] if len(items)>=1 else None
    H2 = items[1][1] if len(items)>=2 else None
    topP = pairs[0]
    if H1:
        L,R = (H1[0], H1[1]) if H1[0][0]<=H1[1][0] else (H1[1], H1[0])
        topP.topL, topP.topR = L, R
        topP.fixed["topL"] = True; topP.fixed["topR"] = True
    if H2:
        L,R = (H2[0], H2[1]) if H2[0][0]<=H2[1][0] else (H2[1], H2[0])
        topP.botL, topP.botR = L, R
        topP.fixed["botL"] = True; topP.fixed["botR"] = True
    # 最底一对的底端对齐到支撑底端
    botP = pairs[-1]
    bottoms = []
    for m in support_models:
        yb=m.ymax; bottoms.append((m.x_at(yb), yb))
    if len(bottoms)>=2:
        bottoms.sort(key=lambda t:t[0])
        botP.botL, botP.botR = bottoms[0], bottoms[-1]
        botP.fixed["botL"] = True; botP.fixed["botR"] = True


def _match_pairs(fpairs: List[_XPair], rpairs: List[_XPair], band_eps_rel: float=0.1, band_eps_abs: float=30.0):
    matched = []
    used=set()
    for i,f in enumerate(fpairs):
        best=None
        for j,r in enumerate(rpairs):
            if j in used: continue
            fh = f.y_bot - f.y_top; rh = r.y_bot - r.y_top
            thr = max(band_eps_rel*max(fh,rh), band_eps_abs)
            dt1=abs(f.y_top-r.y_top); dt2=abs(f.y_bot-r.y_bot)
            if dt1<=thr and dt2<=thr:
                cost = dt1+dt2
                if best is None or cost<best[0]: best=(cost,j)
        if best is not None: matched.append((i,best[1])); used.add(best[1])
        else: matched.append((i,None))
    for j in range(len(rpairs)):
        if j not in used: matched.append((None,j))
    return matched


def _snap_adjacent_xpairs(pairs: List[_XPair], support_models: List[_LineModel],
                    thr_y: float, use_projection: bool = True):
    if not pairs or len(pairs) < 2:
        return pairs
    for i in range(1, len(pairs)):
        up = pairs[i-1]; dn = pairs[i]
        dyL = abs(float(up.botL[1]) - float(dn.topL[1]))
        dyR = abs(float(up.botR[1]) - float(dn.topR[1]))
        if dyL > thr_y or dyR > thr_y:
            continue
        fix_upL = bool(getattr(up, "fixed", {}).get("botL", False))
        fix_upR = bool(getattr(up, "fixed", {}).get("botR", False))
        fix_dnL = bool(getattr(dn, "fixed", {}).get("topL", False))
        fix_dnR = bool(getattr(dn, "fixed", {}).get("topR", False))
        if fix_upL and not fix_dnL:
            dn.topL = up.botL
        elif fix_dnL and not fix_upL:
            up.botL = dn.topL
        elif (not fix_upL) and (not fix_dnL):
            yL = (float(up.botL[1]) + float(dn.topL[1])) / 2.0
            yR = (float(up.botR[1]) + float(dn.topR[1])) / 2.0
            y_bar = (yL + yR) / 2.0
            if use_projection and support_models:
                xL, xR = _extreme_x_at(support_models, y_bar)
                up.botL = (xL, y_bar); dn.topL = (xL, y_bar)
            else:
                x_bar = (float(up.botL[0]) + float(dn.topL[0])) / 2.0
                up.botL = (x_bar, y_bar); dn.topL = (x_bar, y_bar)
        if fix_upR and not fix_dnR:
            dn.topR = up.botR
        elif fix_dnR and not fix_upR:
            up.botR = dn.topR
        elif (not fix_upR) and (not fix_dnR):
            yL = (float(up.botL[1]) + float(dn.topL[1])) / 2.0
            yR = (float(up.botR[1]) + float(dn.topR[1])) / 2.0
            y_bar = (yL + yR) / 2.0
            if use_projection and support_models:
                xL, xR = _extreme_x_at(support_models, y_bar)
                up.botR = (xR, y_bar); dn.topR = (xR, y_bar)
            else:
                x_bar = (float(up.botR[0]) + float(dn.topR[0])) / 2.0
                up.botR = (x_bar, y_bar); dn.topR = (x_bar, y_bar)
    return pairs

def _pairs_to_dict(pairs: List[_XPair]) -> Dict[str, List[Coord]]:
    out: Dict[str, List[Coord]] = {}
    for p in pairs:
        out[p.idA] = [p.topL, p.botR]
        out[p.idB] = [p.topR, p.botL]
    return out


def _point_to_segment_distance(p: Coord, a: Coord, b: Coord) -> float:
    x0, y0 = p
    x1, y1 = a
    x2, y2 = b
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx * dx + dy * dy
    if l2 <= 1e-12:
        return math.hypot(x0 - x1, y0 - y1)
    t = max(0.0, min(1.0, ((x0 - x1) * dx + (y0 - y1) * dy) / l2))
    px = x1 + t * dx
    py = y1 + t * dy
    return math.hypot(x0 - px, y0 - py)


def _classify_reference_tiers(
    all_members: CoordDict,
    class1_members: CoordDict,
    tolerance: float = 35.0,
) -> Dict[str, Any]:
    """
    Classify rods with the same topology rule used by the single-view path.

    The existing dual-view visual categories are left untouched. This helper
    only records which host rod each endpoint is attached to so final node
    export can decide between real coordinates and reference coordinates.
    """
    tier1 = {str(k): v for k, v in (class1_members or {}).items()}
    remaining = {
        str(k): v for k, v in (all_members or {}).items()
        if str(k) not in tier1
    }

    def find_host(pt: Coord, host_dict: CoordDict) -> Optional[str]:
        best_d = float("inf")
        best_k: Optional[str] = None
        for host_key, seg in (host_dict or {}).items():
            if not seg or len(seg) < 2:
                continue
            d = _point_to_segment_distance(pt, seg[0], seg[1])
            if d < best_d:
                best_d = d
                best_k = str(host_key)
        return best_k if best_d <= tolerance else None

    tier2: CoordDict = {}
    tier3: CoordDict = {}
    endpoint_hosts: Dict[str, List[Optional[Dict[str, str]]]] = {}

    for member_id, seg in list(remaining.items()):
        if not seg or len(seg) < 2:
            continue
        h1 = find_host(seg[0], tier1)
        h2 = find_host(seg[1], tier1)
        if h1 and h2:
            tier2[member_id] = seg
            endpoint_hosts[member_id] = [
                {"kind": "tier1", "host": h1},
                {"kind": "tier1", "host": h2},
            ]
            del remaining[member_id]

    for member_id, seg in list(remaining.items()):
        if not seg or len(seg) < 2:
            continue
        h1_t1 = find_host(seg[0], tier1)
        h2_t1 = find_host(seg[1], tier1)
        h1_t2 = find_host(seg[0], tier2) if not h1_t1 else None
        h2_t2 = find_host(seg[1], tier2) if not h2_t1 else None

        if h1_t1 and h2_t1:
            continue

        host1 = (
            {"kind": "tier1", "host": h1_t1} if h1_t1
            else {"kind": "tier2", "host": h1_t2} if h1_t2
            else None
        )
        host2 = (
            {"kind": "tier1", "host": h2_t1} if h2_t1
            else {"kind": "tier2", "host": h2_t2} if h2_t2
            else None
        )
        if host1 and host2:
            tier3[member_id] = seg
            endpoint_hosts[member_id] = [host1, host2]

    return {
        "tier1": set(tier1.keys()),
        "tier2": set(tier2.keys()),
        "tier3": set(tier3.keys()),
        "endpoint_hosts": endpoint_hosts,
    }

# def _fix_xmembers(front_x: CoordDict, right_x: CoordDict,
#                   front_support: CoordDict, right_support: CoordDict,
#                   front_horizontal: CoordDict, right_horizontal: CoordDict,
#                   front_x_mid: Optional[float], right_x_mid: Optional[float],
#                   span_len_f: Optional[float], span_len_r: Optional[float],
#                   params: Optional[dict]=None):
#     params = params or {}
#     overlap_thr = float(params.get("overlap_thr", 0.65))
#     band_eps_rel = float(params.get("band_eps_rel", 0.10))
#     band_eps_abs = float(params.get("band_eps_abs", 60.0))

#     fpairs = _pair_in_view(front_x, front_x_mid, overlap_thr=overlap_thr)
#     rpairs = _pair_in_view(right_x, right_x_mid, overlap_thr=overlap_thr)

#     def _span_len(sup: CoordDict)->float:
#         xs=[(seg[0][0]+seg[1][0])/2.0 for seg in sup.values()]
#         return (max(xs)-min(xs)) if xs else 100.0
#     spanF=_span_len(front_support); spanR=_span_len(right_support)
#     HmedF = statistics.median([abs(p.y_bot-p.y_top) for p in fpairs]) if fpairs else 100.0
#     HmedR = statistics.median([abs(p.y_bot-p.y_top) for p in rpairs]) if rpairs else 100.0
#     tau_y_f = max(20.0, 0.06*HmedF); tau_x_f = max(25.0, 0.06*spanF)
#     tau_y_r = max(20.0, 0.06*HmedR); tau_x_r = max(25.0, 0.06*spanR)

#     f_models = _models_from_supports(front_support)
#     r_models = _models_from_supports(right_support)
#     _hard_snap_for_view(fpairs, front_horizontal, f_models)
#     _hard_snap_for_view(rpairs, right_horizontal, r_models)

#     matches = _match_pairs(fpairs, rpairs, band_eps_rel, band_eps_abs)
#     for (i_f, i_r) in matches:
#         pts=[];
#         if i_f is not None: pts += [fpairs[i_f].topL, fpairs[i_f].topR]
#         if i_r is not None: pts += [rpairs[i_r].topL, rpairs[i_r].topR]
#         if pts:
#             Ybar = statistics.mean([p[1] for p in pts])
#             Lf,Rf = _extreme_x_at(f_models, Ybar); Lr,Rr = _extreme_x_at(r_models, Ybar)
#             if i_f is not None: fpairs[i_f].topL, fpairs[i_f].topR = (Lf,Ybar),(Rf,Ybar)
#             if i_r is not None: rpairs[i_r].topL, rpairs[i_r].topR = (Lr,Ybar),(Rr,Ybar)
#         pts=[]
#         if i_f is not None: pts += [fpairs[i_f].botL, fpairs[i_f].botR]
#         if i_r is not None: pts += [rpairs[i_r].botL, rpairs[i_r].botR]
#         if pts:
#             Ybar = statistics.mean([p[1] for p in pts])
#             Lf,Rf = _extreme_x_at(f_models, Ybar); Lr,Rr = _extreme_x_at(r_models, Ybar)
#             if i_f is not None: fpairs[i_f].botL, fpairs[i_f].botR = (Lf,Ybar),(Rf,Ybar)
#             if i_r is not None: rpairs[i_r].botL, rpairs[i_r].botR = (Lr,Ybar),(Rr,Ybar)

#     return _pairs_to_dict(fpairs), _pairs_to_dict(rpairs), fpairs, rpairs
def _fix_xmembers(front_x: CoordDict, right_x: CoordDict,
                  front_support: CoordDict, right_support: CoordDict,
                  front_horizontal: CoordDict, right_horizontal: CoordDict,
                  front_x_mid: Optional[float], right_x_mid: Optional[float],
                  span_len_f: Optional[float], span_len_r: Optional[float],
                  params: Optional[dict]=None):
    """
    【极简重构版】：
    废除所有 2D 阶段的强制对齐（_hard_snap）、相邻捏合（_snap_adjacent）和跨视图高度配对（_match_pairs）。
    因为这些操作会严重扭曲 CAD 原始坐标，导致连续 X 型或不规则 K/V 型杆件在 3D 重建前就损坏或丢失。
    我们只负责：
    1. 把 \\ 和 / 组合成 X 型对（fpairs / rpairs），并适度放宽重叠要求。
    2. 原封不动地输出它们的 2D 坐标。
    3. 把没有配对成功的单根斜材也保留下来，防止漏杆件。
    真正的对齐和缝合，将由 generate_outputs 在 3D 空间中通过 _closest_id 完美完成。
    """
    params = params or {}
    overlap_thr = float(params.get("overlap_thr", 0.40))  # 放宽重叠要求到 40%，拯救画得不标准的 X

    # 1. 尝试将 \ 和 / 组合成 X 型对
    fpairs = _pair_in_view(front_x, front_x_mid, overlap_thr=overlap_thr)
    rpairs = _pair_in_view(right_x, right_x_mid, overlap_thr=overlap_thr)

    # 2. 将配对成功的 X 型杆件提取为字典
    out_f = _pairs_to_dict(fpairs)
    out_r = _pairs_to_dict(rpairs)

    # 3. 【关键防漏机制】：
    # 无论是半宽的 K/V 型，还是因为重叠率不够没配上对的 X 型，
    # 只要存在于 front_x / right_x 中，且没有进入 out_f / out_r，统统原样保留！
    for k, seg in (front_x or {}).items():
        sk = str(k)
        if sk not in out_f:
            out_f[sk] = [(float(seg[0][0]), float(seg[0][1])), (float(seg[1][0]), float(seg[1][1]))]
            
    for k, seg in (right_x or {}).items():
        sk = str(k)
        if sk not in out_r:
            out_r[sk] = [(float(seg[0][0]), float(seg[0][1])), (float(seg[1][0]), float(seg[1][1]))]

    return out_f, out_r, fpairs, rpairs



def _drawing_sort_key(value):
    """Natural order for drawing names like 07.txt, J1-7.txt, J1-11.txt."""
    stem = os.path.splitext(os.path.basename(str(value)))[0]
    parts = re.split(r"(\d+)", stem)
    key = tuple((0, int(part)) if part.isdigit() else (1, part.lower()) for part in parts)
    return key, stem.lower()


# ======================================================================
# ========================= 主要功能函数 ===============================
# ======================================================================
def main(data_dir: str):
    print(f"开始处理，数据文件夹路径: {data_dir}")

    txts = sorted(glob.glob(os.path.join(data_dir, "*.txt")), key=_drawing_sort_key)
    if not txts:
        print("错误: 目录下没有找到 .txt 数据文件")
        return [], [], []

    two_view_files, single_view_files = [], []
    print("=" * 50)
    print("步骤 1: 开始文件分类...")
    for fp in txts:
        stem = os.path.splitext(os.path.basename(fp))[0]
        if stem.startswith("_"):
            continue
        front_raw, right_raw = load_and_parse_data(fp)
        if front_raw and right_raw:
            two_view_files.append(fp)
            print(f"  - '{stem}.txt' -> [双视图文件]")
        elif front_raw:
            single_view_files.append(fp)
            print(f"  - '{stem}.txt' -> [单视图文件]")
        else:
            print(f"  - '{stem}.txt' -> [跳过] 缺少正面视图数据")
    print("文件分类完成。")

    all_models_data = {}
    if two_view_files:
        print("\n" + "#" * 60)
        print("## 步骤 2: 开始处理双视图文件...")
        print("#" * 60)
        for fp in two_view_files:
            stem = os.path.splitext(os.path.basename(fp))[0]
            print("=" * 46)
            print(f"处理双视图文件: {stem}")
            front_raw, right_raw = load_and_parse_data(fp)

            front_raw = clean_view(front_raw, "正面")
            right_raw = clean_view(right_raw, "右侧面")

            front_raw_for_tiers = dict(front_raw)
            right_raw_for_tiers = dict(right_raw)

            # CAD horizontal members often carry a few drawing-unit Y offset.
            # Scale the tolerance to this drawing while keeping it below the
            # range where diagonal members could be classified as horizontal.
            all_y_values = [
                point[1]
                for view in (front_raw, right_raw)
                for segment in view.values()
                for point in segment
            ]
            vertical_span = max(all_y_values) - min(all_y_values) if all_y_values else 0.0
            H_TOL = min(25.0, max(10.0, vertical_span * 0.005))
            front_support = find_supports(front_raw)
            right_support = find_supports(right_raw)
            if len(front_support) < 2 or len(right_support) < 2:
                print(
                    "  - 错误：主杆识别不足，无法建立双视图基座，跳过。"
                    f"（正面={len(front_support)}, 侧面={len(right_support)}）"
                )
                continue
            # Keep the unmodified CAD legs so secondary endpoints can later
            # be matched to their original host before the legs are refitted.
            front_source_support = dict(front_support)
            right_source_support = dict(right_support)

            front_horizontal = find_horizontals(
                {k: v for k, v in front_raw.items() if k not in front_support},
                tol_y=H_TOL
            )
            right_horizontal = find_horizontals(
                {k: v for k, v in right_raw.items() if k not in right_support},
                tol_y=H_TOL
            )


            front_rest = {k: v for k, v in front_raw.items()
                    if k not in front_support and k not in front_horizontal}
            right_rest = {k: v for k, v in right_raw.items()
                    if k not in right_support and k not in right_horizontal}

            front_support, front_horizontal = enforce_symmetry(front_support, front_horizontal)
            right_support, right_horizontal = enforce_symmetry(right_support, right_horizontal)

            def _support_height_span(supports):
                values = [point[1] for segment in supports.values() for point in segment]
                if len(values) < 2:
                    return None
                low, high = min(values), max(values)
                return (low, high) if high - low >= 1e-9 else None

            # Keep each view's CAD height basis for horizontal pairing.  The
            # following slope-matching step intentionally changes both support
            # extents, so deriving relative levels afterwards is incorrect.
            front_height_span = _support_height_span(front_support)
            right_height_span = _support_height_span(right_support)

            front_support, right_support = match_support_slopes(front_support, right_support)

            f_models = build_support_models(front_support)
            r_models = build_support_models(right_support)

            def _model_height_span(models):
                values = [value for model in models for value in (model.ymin, model.ymax)]
                if len(values) < 2:
                    return None
                low, high = min(values), max(values)
                return (low, high) if high - low >= 1e-9 else None

            model_spans = [span for span in (_model_height_span(f_models), _model_height_span(r_models)) if span]
            canonical_height_span = (
                min(span[0] for span in model_spans),
                max(span[1] for span in model_spans),
            ) if model_spans else None

            plan = plan_top_span(f_models, r_models, front_horizontal, right_horizontal)
            if plan:
                y_top, Lf, Rf, Lr, Rr = plan["y_top"], plan["Lf"], plan["Rf"], plan["Lr"], plan["Rr"]
                if plan["front_top_key"]:
                    front_horizontal[plan["front_top_key"]] = [(Lf, y_top), (Rf, y_top)]
                if plan["right_top_key"]:
                    right_horizontal[plan["right_top_key"]] = [(Lr, y_top), (Rr, y_top)]

                front_support, right_support = expand_to_top_span(
                    front_support, right_support, f_models, r_models, y_top, Lf, Rf, Lr, Rr
                )

            f_models = build_support_models(front_support)
            r_models = build_support_models(right_support)

            # plan_top_span() may extend the support endpoints, so refresh
            # the common target range from the geometry that will actually be
            # used for reconstruction.
            model_spans = [span for span in (_model_height_span(f_models), _model_height_span(r_models)) if span]
            canonical_height_span = (
                min(span[0] for span in model_spans),
                max(span[1] for span in model_spans),
            ) if model_spans else None

            skip_f = {plan["front_top_key"]} if plan and plan.get("front_top_key") else set()
            skip_r = {plan["right_top_key"]} if plan and plan.get("right_top_key") else set()

            front_horizontal, right_horizontal = correct_horizontals(
                f_models,
                r_models,
                front_horizontal,
                right_horizontal,
                skip_f,
                skip_r,
                front_height_span=front_height_span,
                right_height_span=right_height_span,
                target_height_span=canonical_height_span,
            )

            front_support = align_to_top(front_support, front_horizontal)
            right_support = align_to_top(right_support, right_horizontal)

            class1_front = {**front_support, **front_horizontal}
            class1_right = {**right_support, **right_horizontal}

            TOP_XMID_TOL_ABS = None
            TOP_XMID_TOL_RATIO = 0.1

            front_top_key = plan["front_top_key"] if plan and plan.get("front_top_key") else None
            right_top_key = plan["right_top_key"] if plan and plan.get("right_top_key") else None
            y_top_val = plan["y_top"] if plan else None

            front_x_mid, front_x_range, f_top_meta = top_xmid_and_range(
                front_horizontal, preferred_key=front_top_key, y_top=y_top_val,
                tol_abs=TOP_XMID_TOL_ABS, tol_ratio=TOP_XMID_TOL_RATIO
            )
            right_x_mid, right_x_range, r_top_meta = top_xmid_and_range(
                right_horizontal, preferred_key=right_top_key, y_top=y_top_val,
                tol_abs=TOP_XMID_TOL_ABS, tol_ratio=TOP_XMID_TOL_RATIO
            )

            if front_x_mid is not None and front_x_range is not None:
                fx_lo, fx_hi = front_x_range
                f_seg_len = f_top_meta.get('seg_len') if isinstance(f_top_meta, dict) else None
                f_tol = f_top_meta.get('tol') if isinstance(f_top_meta, dict) else None
                f_key = f_top_meta.get('key') if isinstance(f_top_meta, dict) else None
                seg_str = f"len={f_seg_len:.3f}" if isinstance(f_seg_len, (int, float)) else "len=?"
                tol_str = f"tol={f_tol:.3f}" if isinstance(f_tol, (int, float)) else "tol=?"
                key_str = f"key={f_key}" if f_key is not None else "key=?"
                print(
                    f"  - 正面顶横杆 x_mid = {front_x_mid:.3f}，范围 = [{fx_lo:.3f}, {fx_hi:.3f}]"
                    f"（{seg_str}, {tol_str}, {key_str}）"
                )
            else:
                print("  - 正面：未能定位顶端横向杆件")

            if right_x_mid is not None and right_x_range is not None:
                rx_lo, rx_hi = right_x_range
                r_seg_len = r_top_meta.get('seg_len') if isinstance(r_top_meta, dict) else None
                r_tol = r_top_meta.get('tol') if isinstance(r_top_meta, dict) else None
                r_key = r_top_meta.get('key') if isinstance(r_top_meta, dict) else None
                seg_str_r = f"len={r_seg_len:.3f}" if isinstance(r_seg_len, (int, float)) else "len=?"
                tol_str_r = f"tol={r_tol:.3f}" if isinstance(r_tol, (int, float)) else "tol=?"
                key_str_r = f"key={r_key}" if r_key is not None else "key=?"
                print(
                    f"  - 右侧面顶横杆 x_mid = {right_x_mid:.3f}，范围 = [{rx_lo:.3f}, {rx_hi:.3f}]"
                    f"（{seg_str_r}, {tol_str_r}, {key_str_r}）"
                )
            else:
                print("  - 右侧面：未能定位顶端横向杆件")

            front_x_type: CoordDict = {}
            right_x_type: CoordDict = {}
            front_x_dropped_v: CoordDict = {}
            right_x_dropped_v: CoordDict = {}
            span_len_f: Optional[float] = None
            span_len_r: Optional[float] = None
            if front_x_range is not None:
                front_range_tuple = cast(Tuple[float, float], front_x_range)
                front_x_candidates = select_x_type(front_rest, front_range_tuple)
                span_len_f = (f_top_meta['seg_len'] if (f_top_meta and 'seg_len' in f_top_meta) else None)
                front_drop_res = drop_vertical_members(
                    front_x_candidates,
                    span_length=span_len_f,
                    dx_ratio=0.01, dx_abs=4.0, min_len=5.0,
                    return_excluded=True
                )
                front_x_type, front_x_dropped_v = cast(Tuple[CoordDict, CoordDict], front_drop_res)
            else:
                print("  - 正面：没有可用的顶横杆范围，无法筛选X型构件。")

            if right_x_range is not None:
                right_range_tuple = cast(Tuple[float, float], right_x_range)
                right_x_candidates = select_x_type(right_rest, right_range_tuple)
                span_len_r = (r_top_meta['seg_len'] if (r_top_meta and 'seg_len' in r_top_meta) else None)
                right_drop_res = drop_vertical_members(
                    right_x_candidates,
                    span_length=span_len_r,
                    dx_ratio=0.01, dx_abs=4.0, min_len=5.0,
                    return_excluded=True
                )
                right_x_type, right_x_dropped_v = cast(Tuple[CoordDict, CoordDict], right_drop_res)
            else:
                print("  - 右面：没有可用的顶横杆范围，无法筛选X型构件。")

            f_fixed: CoordDict = {}
            r_fixed: CoordDict = {}
            front_sup_final2d: CoordDict = dict(front_support)
            right_sup_final2d: CoordDict = dict(right_support)
            front_horiz_final2d: CoordDict = dict(front_horizontal)
            right_horiz_final2d: CoordDict = dict(right_horizontal)
            front_x_final2d: CoordDict = {}
            right_x_final2d: CoordDict = {}
            front_remaining: CoordDict = {}
            right_remaining: CoordDict = {}
            kept: Dict[str, CoordDict] = {}
            pending3d_2dpack: Dict[str, Dict[str, CoordDict]] = {}
            if front_x_type or right_x_type:
                try:
                    # 1. 调用极简版 X 型处理（只分组、防漏，不扭曲坐标）
                    f_fixed, r_fixed, fpairs, rpairs = _fix_xmembers(
                        front_x_type, right_x_type,
                        front_support, right_support,
                        front_horizontal, right_horizontal,
                        front_x_mid, right_x_mid,
                        span_len_f, span_len_r,
                        params={
                            "overlap_thr": 0.40, # 放宽重叠率，拯救所有的 X/K/V
                        }
                    )
                    
                    # ⚠️ 删除了所有 _snap_adjacent_xpairs 和 median 计算的冗余代码！
                    # 原始坐标直接进入 3D 重建池！
                    
                except Exception as e:
                    print(f"  - 在对X型杆件复位时遇到异常：{e}")
                    f_fixed, r_fixed = {}, {}

                front_x_final2d = dict(f_fixed)
                right_x_final2d = dict(r_fixed)
            # if front_x_type or right_x_type:
            #     try:
            #         f_fixed, r_fixed, fpairs, rpairs = _fix_xmembers(
            #             front_x_type, right_x_type,
            #             front_support, right_support,
            #             front_horizontal, right_horizontal,
            #             front_x_mid, right_x_mid,
            #             span_len_f, span_len_r,
            #             params={
            #                 "overlap_thr": 0.65,
            #                 "band_eps_rel": 0.15,
            #                 "band_eps_abs": 30.0,
            #             }
            #         )

            #         f_models = _models_from_supports(front_support)
            #         r_models = _models_from_supports(right_support)
            #         HmedF = statistics.median([abs(p.y_bot-p.y_top) for p in fpairs]) if fpairs else 100.0
            #         HmedR = statistics.median([abs(p.y_bot-p.y_top) for p in rpairs]) if rpairs else 100.0
            #         thr_y_f = max(50.0, 0.06*HmedF)
            #         thr_y_r = max(50.0, 0.06*HmedR)
            #         fpairs = _snap_adjacent_xpairs(fpairs, f_models, thr_y=thr_y_f, use_projection=True)
            #         rpairs = _snap_adjacent_xpairs(rpairs, r_models, thr_y=thr_y_r, use_projection=True)
            #         f_fixed = _pairs_to_dict(fpairs)
            #         r_fixed = _pairs_to_dict(rpairs)
            #     except Exception as e:
            #         print(f"  - 在对X型杆件复位时遇到异常：{e}")
            #         f_fixed, r_fixed = {}, {}

            #     front_sup_final2d = dict(front_support)
            #     right_sup_final2d = dict(right_support)
            #     front_horiz_final2d = dict(front_horizontal)
            #     right_horiz_final2d = dict(right_horizontal)
            #     front_x_final2d = dict(f_fixed)
            #     right_x_final2d = dict(r_fixed)

            def _remaining(raw_dict, a, b, c):
                keys = set(raw_dict.keys()) - set(a.keys()) - set(b.keys()) - set(c.keys())
                return {k: raw_dict[k] for k in keys if k in raw_dict}

            try:
                front_remaining = _remaining(front_raw, front_sup_final2d, front_horiz_final2d, front_x_final2d)
                right_remaining = _remaining(right_raw, right_sup_final2d, right_horiz_final2d, right_x_final2d)

                kept = {
                    "front_raw": front_remaining,
                    "right_raw": right_remaining,
                    "front_support": front_sup_final2d,
                    "right_support": right_sup_final2d,
                    "front_horizontal": front_horiz_final2d,
                    "right_horizontal": right_horiz_final2d,
                    "front_xmembers": front_x_final2d,
                    "right_xmembers": right_x_final2d,
                }

                pending3d_2dpack = {
                    "front": {
                        "support": front_sup_final2d,
                        "horizontal": front_horiz_final2d,
                        "xmembers": front_x_final2d,
                    },
                    "right": {
                        "support": right_sup_final2d,
                        "horizontal": right_horiz_final2d,
                        "xmembers": right_x_final2d,
                    },
                }

                print("  - 数据保留摘要：")
                print(f"    原始(剩余) 正面/侧面: {len(front_remaining)}/{len(right_remaining)}")
                print(f"    支撑 正面/侧面: {len(front_sup_final2d)}/{len(right_sup_final2d)}")
                print(f"    横向 正面/侧面: {len(front_horiz_final2d)}/{len(right_horiz_final2d)}")
                print(f"    X型  正面/侧面: {len(front_x_final2d)}/{len(right_x_final2d)}")

            except Exception as e:
                print(f"  - [警告] 数据保留结构构建失败：{e}")
                kept = {}
                pending3d_2dpack = {}

            # class2_front = dict(f_fixed)
            # class2_right = dict(r_fixed)
                        # === 核心修复：拯救被丢弃的 K型/V型 辅助斜材 ===
            # 将 f_fixed (标准的交叉X型) 与 front_remaining (未交叉/半宽的辅材) 合并
            # 保证图纸上画了的所有斜材，一根不少地进入 3D 重建池
            # All secondary members must use the same vertical coordinate
            # basis as the corrected main legs/horizontals.  Without this,
            # diagonal endpoints retain their view-local CAD heights and the
            # front/right representations drift onto different Z layers.
            class2_front = remap_vertical_coordinates(
                {**f_fixed, **front_remaining},
                front_height_span,
                canonical_height_span,
                source_support=front_source_support,
                target_support=front_support,
            )
            class2_right = remap_vertical_coordinates(
                {**r_fixed, **right_remaining},
                right_height_span,
                canonical_height_span,
                source_support=right_source_support,
                target_support=right_support,
            )

            front_total = {**front_support, **front_horizontal, **class2_front}
            right_total = {**right_support, **right_horizontal, **class2_right}
            front_reference_tiers = _classify_reference_tiers(
                front_total,
                {**front_support, **front_horizontal},
            )
            right_reference_tiers = _classify_reference_tiers(
                right_total,
                {**right_support, **right_horizontal},
            )

            fbases = extract_bases_front(front_support, front_horizontal)
            sbases = extract_bases_side(right_support, right_horizontal)
            if fbases is None or sbases is None:
                print("  - 错误：无法提取基座点，跳过。")
                continue
                        # ===== 调试输出 =====
#             print("front bases:", fbases)
#             print("side bases:", sbases)
# # ===================
#             print("front_total keys:", list(front_total.keys())[:10])
#             print("right_total keys:", list(right_total.keys())[:10])
            
            f3d = reconstruct3d_front(front_total, front_support, front_horizontal, sbases)
            r3d = reconstruct3d_right(right_total, right_support, right_horizontal, fbases)

            center_xyz, z_minimum = center_props(f3d, r3d)
            translation_target = (center_xyz[0], center_xyz[1], z_minimum)
            f3d, r3d = translate_model(f3d, r3d, translation_target)
            print("  - 已执行模型中心平移。")

            fs_models = build_support_models(front_support)
            leftmost_support_id = fs_models[0].id if fs_models else None

            all_models_data[stem] = {
                'name': stem,
                'f3d': f3d,
                'r3d': r3d,
                'front_support_keys': set(map(str, front_support.keys())),
                'right_support_keys': set(map(str, right_support.keys())),
                'ganjian_args': {
                    'front_support': front_support, 'right_support': right_support,
                    'front_horizontal': front_horizontal, 'right_horizontal': right_horizontal,
                    'front_x_fixed': class2_front,
                    'right_x_fixed': class2_right,
                    'front_reference_tiers': front_reference_tiers,
                    'right_reference_tiers': right_reference_tiers,
                }
            }
            print(f"  - 双视图模型 '{stem}' 已处理并暂存。")

    spliced_f3d, spliced_r3d = {}, {}
    if len(all_models_data) > 1:
        base_idx = AUTO_SPLICE_BASE_INDEX
        if not AUTO_SPLICE_SELECTION:
            print("\n" + "=" * 50)
            print("检测到多个模型，请选择作为拼接基准的模型：")
            model_keys = sorted(all_models_data.keys(), key=_drawing_sort_key)
            for idx, key in enumerate(model_keys):
                print(f"  [{idx}] {key}")
            # 交互式读取用户输入：用于明确哪一段塔身作为整个拼接链的起点
            while True:
                try:
                    sel = input(f"请输入基准模型序号 (0-{len(model_keys)-1}, 默认0): ").strip()
                    if not sel:
                        base_idx = 0
                        break
                    val = int(sel)
                    if 0 <= val < len(model_keys):
                        base_idx = val
                        break
                    else:
                        print("序号超出范围，请重试。")
                except ValueError:
                    print("输入无效，请输入数字。")
            print(f"已选择基准模型: {model_keys[base_idx]}")

        # 将用户选择（或自动推断）的基准索引传递给拼接核心
        spliced_f3d, spliced_r3d = splice_models(
            all_models_data,
            auto_base_index=base_idx,
        )
    elif len(all_models_data) == 1:
        model_data = list(all_models_data.values())[0]
        spliced_f3d, spliced_r3d = model_data['f3d'], model_data['r3d']
    
    # 1. 修复字典覆盖，保留 F_ 和 R_ 前缀
    final_coords_map = {**spliced_f3d, **spliced_r3d}

    if final_coords_map:
        print("\n" + "=" * 50 + "\n双视图模型全局缩放步骤\n" + "=" * 50)
        manual_scale_needed = True
        if AUTO_SCALE_ENABLED:
            # 2. 自动定标查找时兼容前缀
            auto_id = f"F_{AUTO_SCALE_MEMBER_ID}" if f"F_{AUTO_SCALE_MEMBER_ID}" in final_coords_map else f"R_{AUTO_SCALE_MEMBER_ID}"
            if auto_id in final_coords_map:
                try:
                    final_coords_map = scale_by_member(
                        final_coords_map, auto_id, AUTO_SCALE_MEMBER_LENGTH,
                    )
                    print(f"  - 自动定标：杆件 {auto_id} 长度 {AUTO_SCALE_MEMBER_LENGTH}m")
                    manual_scale_needed = False
                except Exception as exc:
                    print(f"  - [警告] 自动定标失败：{exc}")
            else:
                print(f"  - [警告] 自动定标失败：未找到杆件 {AUTO_SCALE_MEMBER_ID}")
                
        if manual_scale_needed:
            while True:
                try:
                    try:
                        target_id = input("请输入用于定标的双视图杆件ID (直接回车可跳过): ").strip()
                    except EOFError:
                        print("  - 非交互运行，跳过手动缩放。")
                        break
                    if not target_id:
                        print("  - 已跳过手动缩放。")
                        break
                    
                    # 3. 手动输入查找兼容前缀
                    search_id = f"F_{target_id}" if f"F_{target_id}" in final_coords_map else f"R_{target_id}"
                    
                    if search_id not in final_coords_map:
                        print(f"  - 错误：杆件ID '{target_id}' 不存在。")
                        continue
                    try:
                        real_length = float(input(f"请输入杆件 '{target_id}' 的真实长度 (单位: 米): "))
                    except EOFError:
                        print("  - 非交互运行，跳过手动缩放。")
                        break
                    final_coords_map = scale_by_member(
                        final_coords_map, search_id, real_length,
                    )
                    print("  - 定标完成。")
                    break
    # final_coords_map = {**{k.replace("F_", ""): v for k, v in spliced_f3d.items()},
    #                 **{k.replace("R_", ""): v for k, v in spliced_r3d.items()}}

    # if final_coords_map:
    #     print("\n" + "=" * 50 + "\n双视图模型全局缩放步骤\n" + "=" * 50)
    #     manual_scale_needed = True
    #     if AUTO_SCALE_ENABLED:
    #         auto_id = AUTO_SCALE_MEMBER_ID
    #         if auto_id in final_coords_map:
    #             try:
    #                 final_coords_map = scale_by_member(
    #                     final_coords_map,
    #                     auto_id,
    #                     AUTO_SCALE_MEMBER_LENGTH,
    #                 )
    #                 print(
    #                     f"  - 自动定标：杆件 {auto_id} 长度 {AUTO_SCALE_MEMBER_LENGTH}m"
    #                 )
    #                 manual_scale_needed = False
    #             except Exception as exc:
    #                 print(f"  - [警告] 自动定标失败：{exc}")
    #         else:
    #             print(
    #                 f"  - [警告] 自动定标失败：未找到杆件 {auto_id}"
    #             )
    #     if manual_scale_needed:
    #         while True:
    #             try:
    #                 target_id = input(
    #                     "请输入用于定标的双视图杆件ID (直接回车可跳过): "
    #                 ).strip()
    #                 if not target_id:
    #                     print("  - 已跳过手动缩放。")
    #                     break
    #                 if target_id not in final_coords_map:
    #                     print(f"  - 错误：杆件ID '{target_id}' 不存在。")
    #                     continue
    #                 real_length = float(
    #                     input(f"请输入杆件 '{target_id}' 的真实长度 (单位: 米): ")
    #                 )
    #                 final_coords_map = scale_by_member(
    #                     final_coords_map,
    #                     target_id,
    #                     real_length,
    #                 )
    #                 print("  - 定标完成。")
                    break
                except ValueError:
                    print("  - 错误：请输入有效的数字长度。")
                except Exception as e:
                    print(f"  - 发生未知错误: {e}")

        ganjian_stage1, jiedian_stage1, pinjie_stage1 = generate_outputs(
            final_coords_map, all_models_data
        )
        # === New: apply vertical stacking alias mapping ===
        id_aliases = build_vertical_reuse_aliases(final_coords_map, all_models_data, eps=1e-6)
        ganjian_stage1, jiedian_stage1, pinjie_stage1 = apply_id_aliases(
            ganjian_stage1, jiedian_stage1, pinjie_stage1, id_aliases
        )
    else:
        print("\n未处理任何双视图文件。本程序只处理双视图数据，单视图将被跳过。")
        ganjian_stage1, jiedian_stage1, pinjie_stage1 = [], [], []

    if single_view_files:
        for fp in single_view_files:
            stem = os.path.splitext(os.path.basename(fp))[0]
            print(f"[跳过] {stem}.txt 仅含单视图。本程序只处理双视图数据。")
    ganjian_final, jiedian_final, pinjie_final = ganjian_stage1, jiedian_stage1, pinjie_stage1

    def _format_final_output_as_strings(ganjian, jiedian, pinjie):
        for item in ganjian:
            item["member_id"] = str(item["member_id"])
            item["node1_id"] = str(item["node1_id"])
            item["node2_id"] = str(item["node2_id"])
        for item in jiedian:
            item["node_id"] = str(item["node_id"])
            if item.get("node_type") == 12:
                item["X"] = str(item.get("X", ""))
                item["Y"] = str(item.get("Y", ""))
        for item in pinjie:
            if isinstance(item, list) and item:
                item[0] = str(item[0])
        return ganjian, jiedian, pinjie

    ganjian_final, jiedian_final, pinjie_final = _format_final_output_as_strings(
        ganjian_final, jiedian_final, pinjie_final
    )

    return ganjian_final, jiedian_final, pinjie_final

if __name__ == "__main__":
    default_data_directory = r"d:/SanWei/project/2d_coordinate"
    print(f"脚本启动，将使用默认路径: '{default_data_directory}'")
    ganjian_result, jiedian_result, pinjie_result = main(data_dir=default_data_directory)
