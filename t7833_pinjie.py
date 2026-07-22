"""
T7833「上字型」担架拼接点识别（截至 2026-07-06）。

与既有图纸不同：T7833 的拼接点这样识别——
1. 担架正视图两个一类杆件（编号升序：小=上杆、大=下杆）各取左端点（x 较小端点）为原始二维坐标；
2. 到塔身正视图的**横杆**里找相同二维坐标的端点，拿到该横杆的塔身 member_id；
3. 取该横杆 ganjian 行的 +x 右侧节点（node2）作为拼接点 node_id，其三维坐标从塔身 jiedian 取
   （node2 直接查；查不到则用其 -x 镜像 node1 的坐标、x 取负）。

只依赖塔身返回的 jiedian + ganjian，不需要塔身重建内部的 final_coords_map。

产出的 pinjie 直接喂给 xintrans.trans 消费：每 4 个一组对应一个担架，
组内顺序 [右上, 右下, 左上, 左下]（右侧 x>=0 进 positive_group，左侧 x<0 进 negative_group 被丢弃）。
左侧为右侧镜像（node_id 复用右侧、坐标 x 取负）。
"""

import os
import glob

from get_first_ganjian_id import detect_main_rods_enhanced
from dual_view_core import _parse_block_dict


def _read_front(txt_path):
    """
    读取一个坐标 txt，返回其正视图杆件字典 {member_id_str: [(x1,y1),(x2,y2)]}。

    复用塔身重建的正则解析器 _parse_block_dict，而非 exec：
    txt 里形如 119_1 的 key 若用 exec 会被 Python 当成数字分隔符解析成整数 1191，
    与塔身 final_coords_map 保留的字符串 key（F_119_1）对不上。正则解析保持字符串 key 一致。
    """
    with open(txt_path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()
    return _parse_block_dict(text, "front")


def _left_end(front, mid):
    """取杆件 mid 的左端点（x 较小者），返回 (二维点, 端点index)。"""
    p1, p2 = front[mid]  # [(x1,y1),(x2,y2)]
    return (p1, 0) if p1[0] <= p2[0] else (p2, 1)


def _right_end(front, mid):
    """取杆件 mid 的右端点（x 较大者），返回 (二维点, 端点index)。与 _left_end 对称。"""
    p1, p2 = front[mid]  # [(x1,y1),(x2,y2)]
    return (p1, 0) if p1[0] >= p2[0] else (p2, 1)


def _pick_end(front, mid, side):
    """按 side（"left"/"right"）取杆件 mid 的连接端点。"""
    return _right_end(front, mid) if side == "right" else _left_end(front, mid)


def _base_id(k):
    """去掉 F_/R_ 前缀与 _1/_2 拼接后缀，得到基础 member_id。例：F_119_1 -> 119。"""
    k = str(k).replace("F_", "").replace("R_", "")
    return k.split("_", 1)[0]


def _is_horizontal(seg, tol_y=20.0):
    """判断塔身杆件是否为横杆（两端 y 近似相等）。担架连接点落在横杆上。"""
    return abs(seg[0][1] - seg[1][1]) <= tol_y


def _match_tashen_horizontal(pt2d, tashen_fronts, tol):
    """
    在塔身正视图的**横杆**里找与 pt2d 重合的端点。
    只匹配横杆，避免斜杆（如 113）在同一二维点上的深度歧义与悬空节点。
    tashen_fronts: [(member_id, [(x1,y1),(x2,y2)]), ...]
    返回 (塔身member_id, 重合端点index)。
    """
    best = None
    for mid, seg in tashen_fronts:
        if not _is_horizontal(seg):
            continue
        for idx, (x, y) in enumerate(seg):
            d = ((x - pt2d[0]) ** 2 + (y - pt2d[1]) ** 2) ** 0.5
            if d <= tol and (best is None or d < best[0]):
                best = (d, mid, idx)
    if best is None:
        raise KeyError(f"T7833: 塔身正视图横杆中未匹配到担架拼接点二维坐标 {pt2d}")
    return best[1], best[2]


def _right_nodes_of_member(mid, ganjian_rows_by_base):
    """
    取横杆 mid 的 (-x 左节点, +x 右节点) 编号。
    横杆 ganjian 行约定 node1↔idx0(-x)、node2↔idx1(+x)；
    多行时优先 symmetry_type==2 的正视图行（拼接/对称拆分会产生 sym1 的镜像行）。
    """
    rows = ganjian_rows_by_base.get(_base_id(mid), [])
    if not rows:
        return None, None
    sym2 = [r for r in rows if int(r.get("symmetry_type", 0)) == 2]
    chosen = sym2[0] if sym2 else rows[0]
    return str(chosen.get("node1_id")), str(chosen.get("node2_id"))


def _node_xyz_from_jiedian(node_id, node_xyz):
    """从 jiedian 坐标表取节点三维坐标；仅接受数值型（node_type 11）坐标。"""
    v = node_xyz.get(str(node_id))
    if v is None:
        return None
    try:
        return [float(v[0]), float(v[1]), float(v[2])]
    except (TypeError, ValueError):
        return None  # node_type 12 的引用式坐标（非数值）不可用


def build_pinjie_for_t7833(danjia_dir, tashen_dir, jiedian_tashen, ganjian_tashen, tol=1.0,
                           end_side_by_index=None, swap_updown=False):
    """
    为 T7833 构建 xintrans 消费格式的 pinjie（长度 = 担架数 × 4）。
    每个元素形如 [node_id_str, [x, y, z]]。

    识别规则（与用户确认的新逻辑一致）：
      担架一类杆件左端点 -> 塔身正视图**横杆**上的重合端点 -> 取该横杆 +x 右侧 ganjian 节点(node2)。
      三维坐标：node2 直接查 jiedian；查不到则用其 -x 镜像 node1 的坐标、x 取负（横杆左右对称）。
      左侧 negative_group 为右侧镜像（node_id 复用、x 取负，会被 xintrans 丢弃）。

    参数:
        danjia_dir: 担架坐标目录（含 0{i}.txt）
        tashen_dir: 塔身坐标目录（含正视图 txt）
        jiedian_tashen: 塔身节点列表，用于取连接点三维坐标
        ganjian_tashen: 塔身杆件列表，用于把横杆映射回 +x 连接节点编号
        tol: 二维坐标匹配容差（原始坐标为整数，默认 1.0 兜底）
        end_side_by_index: 按担架序号（0基）指定取左端还是右端的映射，
            形如 {0: "right", 1: "left"}。为 None 时全部取左端（T7833 默认行为）。
            7837 与 T7833 担架朝向相反：1号担架(01.txt)右端连塔身、2号担架(02.txt)左端连塔身。
        swap_updown: 是否对调上/下杆。detect_main_rods_enhanced 按编号升序返回，
            T7833 编号小=上杆(默认 up=ids[0])；7837 编号小=下杆，需置 True 对调。
    """
    # 汇总塔身所有正视图杆件的二维坐标
    tashen_fronts = []
    for fp in sorted(glob.glob(os.path.join(tashen_dir, "*.txt"))):
        for mid, seg in _read_front(fp).items():
            tashen_fronts.append((str(mid), seg))

    # member_id(base) -> [ganjian 行, ...]（同一 base 可能有对称/拼接多行）
    ganjian_rows_by_base = {}
    for g in ganjian_tashen:
        ganjian_rows_by_base.setdefault(_base_id(g.get("member_id")), []).append(g)

    # node_id -> (X,Y,Z)
    node_xyz = {str(n.get("node_id")): (n.get("X"), n.get("Y"), n.get("Z")) for n in jiedian_tashen}

    def endpoint(pt2d):
        """担架某左端点二维坐标 -> (塔身+x节点编号, +x端三维坐标)。"""
        mid, _idx = _match_tashen_horizontal(pt2d, tashen_fronts, tol)
        left_node, right_node = _right_nodes_of_member(mid, ganjian_rows_by_base)
        if right_node is None:
            raise KeyError(f"T7833: 塔身横杆 {mid} 在 ganjian 中无对应节点")
        # +x 端坐标：优先直接查 node2；查不到用 -x 镜像 node1 坐标、x 取负
        xyz = _node_xyz_from_jiedian(right_node, node_xyz)
        if xyz is None:
            mir = _node_xyz_from_jiedian(left_node, node_xyz)
            if mir is None:
                raise KeyError(f"T7833: 横杆 {mid} 的节点 {right_node}/{left_node} 均无数值坐标")
            xyz = [-mir[0], mir[1], mir[2]]  # 镜像到 +x
        return right_node, [abs(xyz[0]), xyz[1], xyz[2]]

    danjia_txts = sorted(glob.glob(os.path.join(danjia_dir, "*.txt")))
    pinjie = []
    for idx, txt in enumerate(danjia_txts):
        front = _read_front(txt)
        ids = detect_main_rods_enhanced(front)  # 编号升序
        if len(ids) < 2:
            raise ValueError(f"T7833: 担架 {os.path.basename(txt)} 正视图未识别到两个一类杆件")
        # 默认小编号=上杆；swap_updown 时对调（7837 小编号=下杆）
        up_id, down_id = (ids[1], ids[0]) if swap_updown else (ids[0], ids[1])
        side = (end_side_by_index or {}).get(idx, "left")
        up2d, _ = _pick_end(front, up_id, side)
        down2d, _ = _pick_end(front, down_id, side)

        u_node, u_xyz = endpoint(up2d)
        d_node, d_xyz = endpoint(down2d)

        # 右侧（x>=0）真实值，左侧（x<0）为右侧镜像（node_id 复用、x 取负）
        rx_u = [abs(u_xyz[0]), u_xyz[1], u_xyz[2]]
        rx_d = [abs(d_xyz[0]), d_xyz[1], d_xyz[2]]
        lx_u = [-abs(u_xyz[0]), u_xyz[1], u_xyz[2]]
        lx_d = [-abs(d_xyz[0]), d_xyz[1], d_xyz[2]]

        # 组内顺序：右上, 右下, 左上, 左下
        pinjie += [
            [u_node, rx_u],
            [d_node, rx_d],
            [u_node, lx_u],
            [d_node, lx_d],
        ]

    return pinjie
