import math
import io_utils as rw

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
    return str(raw_id).split("_")[0]

# ================= 1. 先找一类杆件 =================
def extract_lines01(lines_dict):
    target = {}
    for mid, coords in lines_dict.items():
        s = clean_id(mid)
        if s[-2:] in ["01", "02"]:
            target[mid] = coords
    return target

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
def single_view0201(line_coord):
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

    special_bar_id = clean_id(next(iter(lines01.keys())))

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
        node_ids = []
        for i, pt in enumerate(seg):
            nid = get_safe_nid(clean_k)
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
            node_ids = []
            for i, pt in enumerate(seg):
                # 必定生成 110910 和 110920！
                nid = get_safe_nid(clean_k)
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
            tier2_nodes_map[clean_k] = (node_ids[0], node_ids[1])

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
            nid = get_safe_nid(clean_k)
            x3d, _, z3d = projector(pt[0], pt[1])

            h_t1 = h1_t1 if i == 0 else h2_t1
            h_t2 = h1_t2 if i == 0 else h2_t2

            # --------------------
            # 情况 A：吸附在【一类主腿】
            # --------------------
            if h_t1:
                real_host_id = clean_id(h_t1)  # 关键修复：用真实吸附的主腿ID
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
                real_host_id = clean_id(h_t2)
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