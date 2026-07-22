import math
import io_utils as rw
from sv_class1_transform import extract_target_members

# ================= 核心工具 =================
def dist_pt_seg(p, a, b):
    """计算点到线段的最短距离，用于容差吸附"""
    x0, y0 = p; x1, y1 = a; x2, y2 = b
    dx = x2 - x1; dy = y2 - y1
    l2 = dx*dx + dy*dy
    if l2 == 0: return math.hypot(x0-x1, y0-y1)
    t = max(0, min(1, ((x0-x1)*dx + (y0-y1)*dy) / l2))
    px = x1 + t * dx; py = y1 + t * dy
    return math.hypot(x0-px, y0-py)

def clean_id(raw_id):
    """去除 CAD 图纸解析时带入的 _1 等后缀"""
    text = str(raw_id).strip()
    if not text:
        return text
    head, sep, tail = text.rpartition("_")
    if sep and tail.isdigit():
        return head
    return text


def member_instance_id(raw_id):
    """Return the original member id, preserving duplicate-instance suffixes."""
    return str(raw_id).strip()


def node_id_base(raw_id):
    """Build the numeric node-ID prefix while preserving duplicate instances."""
    return member_instance_id(raw_id).replace("_", "")

# ================= 1. 先找一类杆件 =================
def extract_lines01(lines_dict):
    return extract_target_members(lines_dict)
# ================= 辅助：算真实坐标的投影仪 =================
def build_projector(lines01):
    lines = list(lines01.values())
    if len(lines) < 2: return None

    centers = [((seg[0][0]+seg[1][0])/2.0, seg) for seg in lines]
    centers.sort(key=lambda x: x[0])

    mid = len(centers)//2
    left_group = [seg for _, seg in centers[:mid]]
    right_group = [seg for _, seg in centers[mid:]]

    left_line = min(left_group, key=lambda s: min(s[0][0], s[1][0]))
    right_line = max(right_group, key=lambda s: max(s[0][0], s[1][0]))
    center_x_avg = (left_line[0][0] + right_line[0][0]) / 2.0

    y_min = min(left_line[0][1], left_line[1][1], right_line[0][1], right_line[1][1])
    y_max = max(left_line[0][1], left_line[1][1], right_line[0][1], right_line[1][1])
    h_cad = abs(y_max - y_min)

    def get_x(y, line):
        (x1, y1), (x2, y2) = line
        if abs(y2 - y1) < 1e-9: return (x1+x2)/2.0
        return x1 + (y - y1)*(x2-x1)/(y2-y1)

    w_top = abs(get_x(y_min, right_line) - get_x(y_min, left_line))
    w_bot = abs(get_x(y_max, right_line) - get_x(y_max, left_line))
    span_sq = max(h_cad**2 - ((w_bot - w_top)/2.0)**2, 0.0)
    z_top_3d = math.sqrt(span_sq) / 1000.0

    def project(x, y):
        t = (y - y_min)/h_cad if h_cad > 1e-9 else 0
        z3d = t * z_top_3d
        xl = get_x(y, left_line)
        xr = get_x(y, right_line)
        cx = (xl + xr)/2.0
        cw = abs(xr - xl)
        rel_x = (x - cx)/cw if cw > 1e-9 else 0
        x3d = rel_x * (w_top + t*(w_bot - w_top)) / 1000.0
        return round(x3d,6), round(x3d,6), round(z3d,6)

    return project, center_x_avg

# ================= 六步拓扑主函数 =================
# Legacy implementation kept for reference; production uses single_view0201 below.
def _legacy_single_view0201(line_coord):
    print("\n" + "="*50)
    print("====== 开始执行严格 6 步引用拓扑法（完全遵照原版指令） ======")

    ganjian = []
    jiedian = []

    # === 1. 一类杆件 ===
    lines01 = extract_lines01(line_coord)
    if len(lines01) < 2: return [], []

    proj_result = build_projector(lines01)
    if not proj_result: return [], []
    projector, center_x_cad = proj_result

    special_bar_id = node_id_base(next(iter(lines01.keys())))

    # 用于防冲突的节点 ID 分配器
    used_nids = set()
    def get_safe_nid(base_id):
        """每一根线，必定生成属于自己的 10/20 节点！绝不跳过！"""
        for suffix in range(10, 100, 10):
            test_nid = f"{base_id}{suffix}"
            if test_nid not in used_nids:
                used_nids.add(test_nid)
                return test_nid
        return f"{base_id}99"

    # 核心字典：记录二类杆件的干爹节点 ID，供三类辅材引用
    tier2_nodes_map = {} 

    # === Step 2: 找一类节点 ===
    for k, seg in lines01.items():
        clean_k = clean_id(k)
        member_k = member_instance_id(k)
        node_ids = []
        for i, pt in enumerate(seg):
            nid = get_safe_nid(node_id_base(k))
            x3d, y3d, z3d = projector(pt[0], pt[1])
            jiedian.append({
                "node_id": str(nid), "node_type": 11, "symmetry_type": 4,
                "X": x3d, "Y": y3d, "Z": z3d
            })
            node_ids.append(str(nid))

        ganjian.append({
            "member_id": clean_k, "node1_id": node_ids[0], "node2_id": node_ids[1], "symmetry_type": 4
        })

    # ================= 拓扑分类准备 =================
    unclassified = {k: v for k, v in line_coord.items() if k not in lines01}
    TOLERANCE = 35.0

    def find_host(pt, host_dict):
        best_d = float("inf")
        best_k = None
        for k, seg in host_dict.items():
            d = dist_pt_seg(pt, seg[0], seg[1])
            if d < best_d:
                best_d = d; best_k = k
        return best_k if best_d < TOLERANCE else None

    # === Step 3 & 4: 找二类节点与杆件 ===
    tier2_members = {}
    for k, seg in list(unclassified.items()):
        h1 = find_host(seg[0], lines01)
        h2 = find_host(seg[1], lines01)

        if h1 and h2:
            tier2_members[k] = seg
            del unclassified[k]
            
            clean_k = clean_id(k)
            member_k = member_instance_id(k)
            node_ids = []
            for i, pt in enumerate(seg):
                # 必定生成 110910 和 110920！
                nid = get_safe_nid(node_id_base(k))
                _, _, z3d = projector(pt[0], pt[1])

                if pt[0] < center_x_cad:
                    ref_x, ref_y = f"{special_bar_id}10", f"{special_bar_id}20"
                else:
                    ref_x, ref_y = f"{special_bar_id}11", f"{special_bar_id}21"

                jiedian.append({
                    "node_id": str(nid), "node_type": 12, "symmetry_type": 4,
                    "X": ref_x, "Y": ref_y, "Z": z3d
                })
                node_ids.append(str(nid))

            # 记录二类杆件真实的 10 和 20 节点
            tier2_nodes_map[member_k] = (node_ids[0], node_ids[1])

            # ================== 完美复刻你的侧面生成逻辑 ==================
            is_horiz = abs(seg[0][1] - seg[1][1]) < 25.0
            if is_horiz:
                ganjian.append({"member_id": clean_k, "node1_id": node_ids[0], "node2_id": f"{node_ids[0][:-1]}1", "symmetry_type": 2})
                ganjian.append({"member_id": clean_k, "node1_id": node_ids[0], "node2_id": f"{node_ids[0][:-1]}2", "symmetry_type": 1})
            else:
                # 生成正面交叉杆
                ganjian.append({"member_id": clean_k, "node1_id": node_ids[0], "node2_id": node_ids[1], "symmetry_type": 4})
                # 生成侧面交叉杆 (依靠 symmetry_type=4 让引擎处理对称)
                ganjian.append({"member_id": clean_k, "node1_id": node_ids[0], "node2_id": f"{node_ids[1][:-1]}3", "symmetry_type": 4})

    # === Step 5 & 6: 找三类节点与杆件 (0202辅材) ===
# ==============================================
    # === Step 5 & 6: 三类杆件（0202辅材）【修复完整版】
    # ==============================================
    for k, seg in list(unclassified.items()):
        pt1, pt2 = seg[0], seg[1]

        # ======================
        # 吸附规则：优先一类，再二类（修复乱吸附）
        # ======================
        h1_t1 = find_host(pt1, lines01)
        h2_t1 = find_host(pt2, lines01)

        h1_t2 = find_host(pt1, tier2_members) if not h1_t1 else None
        h2_t2 = find_host(pt2, tier2_members) if not h2_t1 else None

        # 两端都在一类 → 已经是二类，跳过
        if h1_t1 and h2_t1:
            continue

        # 必须至少一端吸附到有效宿主
        if not (h1_t1 or h1_t2) or not (h2_t1 or h2_t2):
            print(f"[跳过] 杆件 {k} 无有效宿主")
            continue


        # ======================
        # 正式生成三类杆件
        # ======================
        clean_k = clean_id(k)
        node_ids = []
        print(f"\n=== [处理三类杆件] {clean_k} ===")

        for i, pt in enumerate(seg):
            nid = get_safe_nid(node_id_base(k))
            x3d, _, z3d = projector(pt[0], pt[1])

            h_t1 = h1_t1 if i == 0 else h2_t1
            h_t2 = h1_t2 if i == 0 else h2_t2

            # --------------------
            # 情况 A：吸附在【一类主腿】
            # --------------------
            if h_t1:
                real_host_id = node_id_base(h_t1)  # 关键修复：用真实吸附的主腿ID
                ref_x = f"{real_host_id}10"
                ref_y = f"{real_host_id}20"

                jiedian.append({
                    "node_id": str(nid),
                    "node_type": 12,
                    "symmetry_type": 4,
                    "X": ref_x,
                    "Y": ref_y,
                    "Z": z3d
                })
                print(f"节点 {nid} → 吸附一类 {real_host_id} | 引用: {ref_x}, {ref_y} | Z={z3d}")

            # --------------------
            # 情况 B：吸附在【二类杆件】
            # --------------------
            elif h_t2:
                real_host_id = node_id_base(h_t2)
                if real_host_id in tier2_nodes_map:
                    rn1, rn2 = tier2_nodes_map[real_host_id]
                    jiedian.append({
                        "node_id": str(nid),
                        "node_type": 12,
                        "symmetry_type": 4,
                        "X": x3d,
                        "Y": rn1,
                        "Z": rn2
                    })
                    print(f"节点 {nid} → 吸附二类 {real_host_id} | 引用节点: {rn1}, {rn2} | X={x3d}")
                else:
                    jiedian.append({
                        "node_id": str(nid),
                        "node_type": 12,
                        "symmetry_type": 4,
                        "X": x3d,
                        "Y": f"{real_host_id}10",
                        "Z": f"{real_host_id}20"
                    })
                    print(f"节点 {nid} → 吸附二类 {real_host_id}（兜底）")

            node_ids.append(str(nid))

        # ======================
        # 生成正面 + 侧面 杆件（不重复、不乱来）
        # ======================
        if len(node_ids) == 2:
            n1, n2 = node_ids[0], node_ids[1]

            # 正面
            ganjian.append({
                "member_id": clean_k,
                "node1_id": n1,
                "node2_id": n2,
                "symmetry_type": 4
            })
            # 侧面（引擎自动90度）
            ganjian.append({
                "member_id": clean_k,
                "node1_id": n1,
                "node2_id": f"{n2[:-1]}3",
                "symmetry_type": 2
            })
            print(f"[生成三类杆件] {clean_k} 正面 {n1}→{n2} | 侧面 {n1}→{n2[:-1]}3")

    print(f"[完成] 1109 必生成版结束，共生成节点数: {len(jiedian)}")
    print("="*50 + "\n")

    return ganjian, jiedian


def _make_node_record(
    node_id,
    node_type,
    symmetry_type,
    x_value,
    y_value,
    z_value,
    front_xy,
    export=True,
    view_face="front",
):
    return {
        "node_id": str(node_id),
        "node_type": int(node_type),
        "symmetry_type": int(symmetry_type),
        "X": x_value,
        "Y": y_value,
        "Z": z_value,
        "_xyz": (x_value, y_value, z_value),
        "_front_xy": tuple(front_xy) if front_xy is not None else None,
        "_view_face": str(view_face),
        "_member_links": [],
        "_export": bool(export),
    }


def _upsert_node_record(
    node_records,
    node_id,
    node_type,
    symmetry_type,
    x_value,
    y_value,
    z_value,
    front_xy,
    export=True,
    view_face="front",
):
    node_id = str(node_id)
    record = node_records.get(node_id)
    if record is None:
        record = _make_node_record(
            node_id, node_type, symmetry_type, x_value, y_value, z_value, front_xy,
            export=export, view_face=view_face
        )
        node_records[node_id] = record
        return record

    record["node_type"] = int(node_type)
    record["symmetry_type"] = int(symmetry_type)
    record["X"] = x_value
    record["Y"] = y_value
    record["Z"] = z_value
    record["_xyz"] = (x_value, y_value, z_value)
    record["_front_xy"] = tuple(front_xy) if front_xy is not None else None
    record["_view_face"] = str(view_face)
    record["_export"] = record["_export"] or bool(export)
    return record


def _ensure_virtual_node(node_records, node_id, source_id):
    node_id = str(node_id)
    if node_id in node_records:
        return node_records[node_id]

    source = node_records.get(str(source_id))
    if source is None:
        source = _make_node_record(node_id, 11, 4, None, None, None, None, export=False)
    record = _make_node_record(
        node_id,
        source["node_type"],
        source["symmetry_type"],
        source["X"],
        source["Y"],
        source["Z"],
        source["_front_xy"],
        export=False,
        view_face=source.get("_view_face", "front"),
    )
    node_records[node_id] = record
    return record


def _register_member_from_nodes(
    node_records,
    member_specs,
    member_order,
    member_id,
    symmetry_type,
    node_ids,
    variant,
    source_ids=None,
    debug_member_trace=False,
):
    connection_key = f"{member_id}|{symmetry_type}|{variant}"
    if debug_member_trace:
        print(
            f"[TRACE register] init connection_key={connection_key} | "
            f"member_id={member_id} symmetry_type={symmetry_type} variant={variant}"
        )
    if connection_key not in member_specs:
        member_specs[connection_key] = {
            "member_id": str(member_id),
            "symmetry_type": int(symmetry_type),
            "preferred_nodes": tuple(str(node_id) for node_id in node_ids),
        }
        member_order.append(connection_key)
        if debug_member_trace:
            print(f"[TRACE register] new member_spec created for {connection_key}")
    elif debug_member_trace:
        print(f"[TRACE register] member_spec already exists for {connection_key}")

    source_ids = source_ids or node_ids
    pair_list = list(zip(node_ids, source_ids))
    if debug_member_trace:
        print(
            f"[TRACE register] node_ids={tuple(str(n) for n in node_ids)} | "
            f"source_ids={tuple(str(s) for s in source_ids)} | pairs={pair_list}"
        )

    for node_id, source_id in zip(node_ids, source_ids):
        node_id = str(node_id)
        source_id = str(source_id)
        if node_id not in node_records:
            _ensure_virtual_node(node_records, node_id, source_id)
            if debug_member_trace:
                print(
                    f"[TRACE register] virtual node created: node_id={node_id} "
                    f"from source_id={source_id}"
                )
        links = node_records[node_id]["_member_links"]
        if connection_key not in links:
            links.append(connection_key)
            if debug_member_trace:
                print(
                    f"[TRACE register] append link: node_id={node_id} "
                    f"link={connection_key}"
                )
        elif debug_member_trace:
            print(
                f"[TRACE register] link already exists: node_id={node_id} "
                f"link={connection_key}"
            )


def _build_members_from_node_records(node_records, member_specs, member_order, debug_member_trace=False):
    ganjian = []
    for connection_key in member_order:
        spec = member_specs[connection_key]
        endpoints = [
            node_id for node_id, record in node_records.items()
            if connection_key in record["_member_links"]
        ]
        if debug_member_trace:
            print(
                f"[TRACE build] connection_key={connection_key} "
                f"-> endpoints={endpoints} (count={len(endpoints)})"
            )
        if len(endpoints) != 2:
            print(f"[警告] 杆件 {spec['member_id']} 找到 {len(endpoints)} 个端点，已跳过")
            continue

        preferred = list(spec["preferred_nodes"])
        ordered = [node_id for node_id in preferred if node_id in endpoints]
        ordered.extend(node_id for node_id in endpoints if node_id not in ordered)
        node1_id, node2_id = ordered[0], ordered[1]
        ganjian.append({
            "member_id": spec["member_id"],
            "node1_id": node1_id,
            "node2_id": node2_id,
            "symmetry_type": spec["symmetry_type"],
        })
    return ganjian


def _debug_dump_member_links(node_records):
    print("\n" + "-" * 50)
    print("[调试] 节点 _member_links 明细")
    print("-" * 50)
    for node_id in sorted(node_records.keys(), key=lambda x: (len(str(x)), str(x))):
        record = node_records[node_id]
        links = record.get("_member_links", [])
        print(
            f"node_id={node_id} | export={record.get('_export')} | "
            f"node_type={record.get('node_type')} | links_count={len(links)}"
        )
        if links:
            for idx, link in enumerate(links, start=1):
                print(f"  {idx:02d}. {link}")
        else:
            print("  (empty)")
    print("-" * 50 + "\n")


def _export_node_records(node_records, include_view_face=False):
    jiedian = []
    for record in node_records.values():
        if not record["_export"]:
            continue
        node = {
            "node_id": record["node_id"],
            "node_type": record["node_type"],
            "symmetry_type": record["symmetry_type"],
            "X": record["X"],
            "Y": record["Y"],
            "Z": record["Z"],
        }
        if include_view_face:
            node["_view_face"] = record.get("_view_face", "front")
        jiedian.append(node)
    return jiedian


def _export_debug_node_records(node_records):
    debug_nodes = []
    for record in node_records.values():
        debug_nodes.append({
            "node_id": record["node_id"],
            "node_type": record["node_type"],
            "symmetry_type": record["symmetry_type"],
            "X": record["X"],
            "Y": record["Y"],
            "Z": record["Z"],
            "_xyz": record.get("_xyz"),
            "_front_xy": record.get("_front_xy"),
            "_member_links": list(record.get("_member_links", [])),
            "_export": bool(record.get("_export", False)),
        })
    return debug_nodes


def single_view0201(
    line_coord,
    debug_member_links=False,
    return_debug_nodes=False,
    debug_member_trace=False,
    front_only=True,
    keep_view_face=False,
):
    print("\n" + "=" * 50)
    print("====== 开始执行严格 6 步引用拓扑法（节点记录驱动版） ======")

    lines01 = extract_lines01(line_coord)
    if len(lines01) < 2:
        return ([], [], []) if return_debug_nodes else ([], [])

    proj_result = build_projector(lines01)
    if not proj_result:
        return ([], [], []) if return_debug_nodes else ([], [])
    projector, center_x_cad = proj_result

    special_bar_id = node_id_base(next(iter(lines01.keys())))
    used_nids = set()
    node_records = {}
    member_specs = {}
    member_order = []
    tier1_nodes_map = {}

    def get_safe_nid(base_id):
        for suffix in range(10, 100, 10):
            test_nid = f"{base_id}{suffix}"
            if test_nid not in used_nids:
                used_nids.add(test_nid)
                return test_nid
        return f"{base_id}99"

    def add_node(
        node_id,
        node_type,
        symmetry_type,
        x_value,
        y_value,
        z_value,
        front_xy,
        export=True,
        view_face="front",
    ):
        return _upsert_node_record(
            node_records, node_id, node_type, symmetry_type, x_value, y_value, z_value,
            front_xy, export=export, view_face=view_face
        )

    def add_member(member_id, symmetry_type, node1_id, node2_id, variant, source1=None, source2=None):
        _register_member_from_nodes(
            node_records,
            member_specs,
            member_order,
            member_id=str(member_id),
            symmetry_type=int(symmetry_type),
            node_ids=(str(node1_id), str(node2_id)),
            variant=str(variant),
            source_ids=(source1 or node1_id, source2 or node2_id),
            debug_member_trace=debug_member_trace,
        )

    def _replace_node_suffix(node_id, suffix):
        node_id = str(node_id)
        return f"{node_id[:-1]}{suffix}"

    def _front_x(node_id):
        record = node_records.get(str(node_id), {})
        front_xy = record.get("_front_xy")
        if not front_xy:
            return None
        return front_xy[0]

    def _front_y(node_id):
        record = node_records.get(str(node_id), {})
        front_xy = record.get("_front_xy")
        if not front_xy:
            return None
        return front_xy[1]

    def _side_face_endpoint_pair(node1_id, node2_id):
        """
        Build an explicit side-face diagonal from a front-view member.
        Cross-body members use the opposite-corner (+3) endpoint; same-side
        members use the depth mirror (+2). This avoids depending on CAD input
        order, which can flip X-brace side members.
        """
        node1_id = str(node1_id)
        node2_id = str(node2_id)
        x1 = _front_x(node1_id)
        x2 = _front_x(node2_id)

        if x1 is None or x2 is None:
            return node1_id, _replace_node_suffix(node2_id, "3"), node2_id

        side1 = -1 if x1 < center_x_cad else 1
        side2 = -1 if x2 < center_x_cad else 1

        if side1 != side2:
            left_id, right_id = (node1_id, node2_id) if x1 < x2 else (node2_id, node1_id)
            return left_id, _replace_node_suffix(right_id, "3"), right_id

        y1 = _front_y(node1_id)
        y2 = _front_y(node2_id)
        if y1 is not None and y2 is not None and y2 < y1:
            node1_id, node2_id = node2_id, node1_id
        return node1_id, _replace_node_suffix(node2_id, "2"), node2_id

    def _side_face_projection(node1_id, node2_id):
        side_node1, side_node2, source2 = _side_face_endpoint_pair(node1_id, node2_id)
        source1 = str(node1_id) if side_node1 == str(node1_id) else str(node2_id)
        return side_node1, side_node2, {
            source1: str(side_node1),
            str(source2): str(side_node2),
        }

    def _resolve_node_xyz(node_id, resolving=None):
        """Resolve a local type-11/type-12 record to a real 3D point."""
        node_id = str(node_id)
        resolving = set() if resolving is None else resolving
        if node_id in resolving:
            return None
        record = node_records.get(node_id)
        if record is None:
            return None

        values = record.get("_xyz", (record["X"], record["Y"], record["Z"]))
        reference_indexes = [index for index, value in enumerate(values) if isinstance(value, str)]
        if not reference_indexes:
            return tuple(float(value) for value in values)
        if len(reference_indexes) != 2:
            return None

        real_index = next(index for index in range(3) if index not in reference_indexes)
        point_a = _resolve_node_xyz(values[reference_indexes[0]], resolving | {node_id})
        point_b = _resolve_node_xyz(values[reference_indexes[1]], resolving | {node_id})
        if point_a is None or point_b is None:
            return None

        span = point_b[real_index] - point_a[real_index]
        if abs(span) < 1e-9:
            return None
        ratio = (float(values[real_index]) - point_a[real_index]) / span
        return tuple(
            float(values[real_index]) if index == real_index
            else point_a[index] + ratio * (point_b[index] - point_a[index])
            for index in range(3)
        )

    def _add_rotated_side_member(member_id, node_base, node1_id, node2_id, variant):
        """Create one true side-face member by rotating both front endpoints."""
        point1 = _resolve_node_xyz(node1_id)
        point2 = _resolve_node_xyz(node2_id)
        if point1 is None or point2 is None:
            print(f"[跳过侧面] 杆件 {member_id} 的正面端点无法解析")
            return None

        side_node1 = get_safe_nid(node_base)
        side_node2 = get_safe_nid(node_base)
        x1, y1, z1 = point1
        x2, y2, z2 = point2
        add_node(side_node1, 11, 4, y1, -x1, z1, None, export=True, view_face="side")
        add_node(side_node2, 11, 4, y2, -x2, z2, None, export=True, view_face="side")
        add_member(member_id, 4, side_node1, side_node2, variant=variant)
        return str(side_node1), str(side_node2)

    tier2_nodes_map = {}
    tier2_side_nodes_map = {}

    for k, seg in lines01.items():
        clean_k = clean_id(k)
        member_k = member_instance_id(k)
        node_ids = []
        # Keep the main-leg endpoint IDs consistent with single_view01:
        # ..10 is the lower endpoint and ..20 is the upper endpoint.  CAD
        # segments are not consistently ordered, while later seam correction
        # uses these suffixes as physical bottom/top identifiers.
        ordered_points = sorted(seg, key=lambda pt: projector(pt[0], pt[1])[2])
        for pt in ordered_points:
            nid = get_safe_nid(node_id_base(k))
            x3d, y3d, z3d = projector(pt[0], pt[1])
            # Keep the two tier1 main legs on the same reference side plane
            # while preserving their left/right X symmetry.
            tier1_y3d = -abs(x3d)
            add_node(nid, 11, 4, x3d, tier1_y3d, z3d, pt, export=True)
            node_ids.append(str(nid))
        tier1_nodes_map[member_k] = (node_ids[0], node_ids[1])
        add_member(member_k, 4, node_ids[0], node_ids[1], variant="tier1-main")

    unclassified = {k: v for k, v in line_coord.items() if k not in lines01}
    tolerance = 35.0

    def find_host(pt, host_dict):
        best_d = float("inf")
        best_k = None
        for host_key, seg in host_dict.items():
            d = dist_pt_seg(pt, seg[0], seg[1])
            if d < best_d:
                best_d = d
                best_k = host_key
        return best_k if best_d < tolerance else None

    tier2_members = {}
    for k, seg in list(unclassified.items()):
        h1 = find_host(seg[0], lines01)
        h2 = find_host(seg[1], lines01)

        if h1 and h2:
            tier2_members[k] = seg
            del unclassified[k]

            clean_k = clean_id(k)
            member_k = member_instance_id(k)
            node_ids = []
            endpoint_hosts = {}
            for idx, pt in enumerate(seg):
                nid = get_safe_nid(node_id_base(k))
                _, _, z3d = projector(pt[0], pt[1])
                host_key = member_instance_id(h1 if idx == 0 else h2)
                if host_key in tier1_nodes_map:
                    ref_x, ref_y = tier1_nodes_map[host_key]
                else:
                    real_host_id = node_id_base(host_key)
                    ref_x, ref_y = f"{real_host_id}10", f"{real_host_id}20"
                add_node(nid, 12, 4, ref_x, ref_y, z3d, pt, export=True)
                node_ids.append(str(nid))
                endpoint_hosts[str(nid)] = host_key

            tier2_nodes_map[member_k] = (node_ids[0], node_ids[1])

            is_horiz = abs(seg[0][1] - seg[1][1]) < 25.0
            side_nodes = None
            side_by_source = {}
            if is_horiz:
                if not front_only:
                    virtual_y = f"{node_ids[0][:-1]}1"
                    virtual_x = f"{node_ids[0][:-1]}2"
                    add_member(member_k, 2, node_ids[0], virtual_y, variant="tier2-horizontal-y", source2=node_ids[0])
                    add_member(member_k, 1, node_ids[0], virtual_x, variant="tier2-horizontal-x", source2=node_ids[0])
            else:
                add_member(member_k, 4, node_ids[0], node_ids[1], variant="tier2-main")
                if not front_only:
                    side_nodes = _add_rotated_side_member(
                        member_k, node_id_base(k), node_ids[0], node_ids[1], "tier2-side"
                    )
                    if side_nodes:
                        tier2_side_nodes_map[member_k] = {
                            "endpoints": side_nodes,
                            "by_source": {},
                            "endpoint_hosts": endpoint_hosts,
                        }
    def _tier3_side_endpoint(node_base, endpoint_info, paired_info):
        node_id = str(endpoint_info["node_id"])
        host_kind = endpoint_info.get("host_kind")
        host_key = endpoint_info.get("host_key")

        if host_kind in {"tier2", "tier3"}:
            side_maps = {
                "tier2": tier2_side_nodes_map,
                "tier3": tier3_side_nodes_map,
            }
            side_info = side_maps[host_kind].get(str(host_key))
            if side_info:
                side_node_id = get_safe_nid(node_base)
                side_ref1, side_ref2 = side_info["endpoints"]
                add_node(
                    side_node_id,
                    12,
                    4,
                    side_ref1,
                    side_ref2,
                    endpoint_info["z"],
                    endpoint_info["front_xy"],
                    export=True,
                    view_face="side",
                )
                return str(side_node_id), str(side_node_id)
            return _replace_node_suffix(node_id, "2"), node_id

        if host_kind == "tier1":
            suffix = "2"
            if paired_info and paired_info.get("host_kind") in {"tier2", "tier3"}:
                paired_kind = paired_info["host_kind"]
                paired_maps = {
                    "tier2": tier2_side_nodes_map,
                    "tier3": tier3_side_nodes_map,
                }
                side_info = paired_maps[paired_kind].get(str(paired_info.get("host_key")))
                if side_info:
                    for source_id, source_host in side_info.get("endpoint_hosts", {}).items():
                        if source_host == host_key:
                            side_id = side_info.get("by_source", {}).get(source_id)
                            if side_id:
                                suffix = str(side_id)[-1]
                                break
            return _replace_node_suffix(node_id, suffix), node_id

        return _replace_node_suffix(node_id, "2"), node_id

    def _tier3_host_node_values(host_key, host_nodes_map, host_members, x3d, z3d):
        """Choose a resolvable type-12 representation for the host direction."""
        if host_key in host_nodes_map:
            ref1, ref2 = host_nodes_map[host_key]
        else:
            real_host_id = node_id_base(host_key)
            ref1, ref2 = f"{real_host_id}10", f"{real_host_id}20"

        host_seg = host_members.get(host_key)
        if host_seg and abs(host_seg[0][1] - host_seg[1][1]) < 25.0:
            # A horizontal host has no Z span.  Keep X real so the renderer
            # can interpolate from the two referenced endpoints.
            return x3d, ref1, ref2
        return ref1, ref2, z3d

    # Tier-3 braces may depend on another tier-3 brace.  Keep propagating
    # recognized hosts until no remaining member can be resolved.
    tier3_members = {}
    tier3_nodes_map = {}
    tier3_side_nodes_map = {}
    pending = dict(unclassified)

    def resolve_host(pt):
        host_key = find_host(pt, lines01)
        if host_key:
            return "tier1", member_instance_id(host_key)
        host_key = find_host(pt, tier2_members)
        if host_key:
            return "tier2", member_instance_id(host_key)
        host_key = find_host(pt, tier3_members)
        if host_key:
            return "tier3", member_instance_id(host_key)
        return None, None

    while pending:
        resolved_count = 0
        for k, seg in list(pending.items()):
            endpoint_hosts = [resolve_host(pt) for pt in seg]
            if not all(host_key for _, host_key in endpoint_hosts):
                continue
            if all(host_kind == "tier1" for host_kind, _ in endpoint_hosts):
                continue

            clean_k = clean_id(k)
            member_k = member_instance_id(k)
            node_ids = []
            endpoint_infos = []
            print(f"\n=== [处理三类杆件] {clean_k} ===")

            for pt, (host_kind, host_key) in zip(seg, endpoint_hosts):
                nid = get_safe_nid(node_id_base(k))
                _, _, z3d = projector(pt[0], pt[1])
                if host_kind == "tier1":
                    host_nodes_map = tier1_nodes_map
                    host_members = lines01
                elif host_kind == "tier2":
                    host_nodes_map = tier2_nodes_map
                    host_members = tier2_members
                else:
                    host_nodes_map = tier3_nodes_map
                    host_members = tier3_members
                x3d, _, z3d = projector(pt[0], pt[1])
                node_x, node_y, node_z = _tier3_host_node_values(
                    host_key, host_nodes_map, host_members, x3d, z3d
                )
                add_node(nid, 12, 4, node_x, node_y, node_z, pt, export=True)
                endpoint_infos.append({
                    "node_id": str(nid),
                    "host_kind": host_kind,
                    "host_key": host_key,
                    "z": z3d,
                    "front_xy": pt,
                })
                node_ids.append(str(nid))

            tier3_members[k] = seg
            tier3_nodes_map[member_k] = (node_ids[0], node_ids[1])
            add_member(member_k, 4, node_ids[0], node_ids[1], variant="tier3-main")
            if not front_only:
                side_nodes = _add_rotated_side_member(
                    member_k, node_id_base(k), node_ids[0], node_ids[1], "tier3-side"
                )
                if side_nodes:
                    tier3_side_nodes_map[member_k] = {
                        "endpoints": side_nodes,
                        "by_source": {},
                        "endpoint_hosts": {
                            endpoint_infos[0]["node_id"]: endpoint_infos[0]["host_key"],
                            endpoint_infos[1]["node_id"]: endpoint_infos[1]["host_key"],
                        },
                    }
            del pending[k]
            resolved_count += 1

        if not resolved_count:
            for k in pending:
                print(f"[跳过] 杆件 {k} 无有效宿主")
            break

    if debug_member_links:
        _debug_dump_member_links(node_records)

    ganjian = _build_members_from_node_records(
        node_records, member_specs, member_order, debug_member_trace=debug_member_trace
    )
    jiedian = _export_node_records(node_records, include_view_face=keep_view_face)

    print(f"[完成] 单视图二/三类节点数: {len(jiedian)} | 杆件数: {len(ganjian)}")
    print("=" * 50 + "\n")

    if return_debug_nodes:
        debug_nodes = _export_debug_node_records(node_records)
        return ganjian, jiedian, debug_nodes
    return ganjian, jiedian
