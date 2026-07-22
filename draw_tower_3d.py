import matplotlib
import pandas as pd

import matplotlib.pyplot as plt
from ipywidgets.widgets import widget
from mpl_toolkits.mplot3d import Axes3D





def load_nodes_from_excel(excel_path):
    """
    从 Excel 文件中读取三维节点信息
    """
    # 1. 读取 Excel
    df = pd.read_excel(excel_path)

    # 2. 基本检查（可按需要删掉）
    required_cols = ["节点编号", "对称性", "X坐标", "Y坐标", "Z坐标"]
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"Excel 缺少必要列: {col}")

    # 3. 提取节点数据
    nodes_3d = []
    for _, row in df.iterrows():
        node = {
            "node_id": int(row["节点编号"]),
            "symmetry": int(row["对称性"]),
            "x": float(row["X坐标"]),
            "y": float(row["Y坐标"]),
            "z": float(row["Z坐标"]),
        }
        nodes_3d.append(node)

    return nodes_3d

def classify_nodes(nodes_3d, id_threshold=10000):
    """
    将节点分为一类节点和二类节点

    判定规则：
    - 一类节点：xyz 三个坐标都是真实值
    - 二类节点：xyz 中只有一个是真实值，其余两个是节点编号
    """

    def is_node_id_value(v):
        return float(v).is_integer() and abs(v) >= id_threshold

    first_class_nodes = []
    second_class_nodes = []

    for node in nodes_3d:
        coords = [node["x"], node["y"], node["z"]]

        # 判断每个坐标是否像“节点编号”
        id_like_count = sum(is_node_id_value(c) for c in coords)

        if id_like_count == 0:
            # xyz 都是真实值
            first_class_nodes.append(node)
        elif id_like_count == 2:
            # 两个是编号，一个是真实值
            second_class_nodes.append(node)
        else:
            # 其他情况：暂不归类（可按需要打印或处理）
            pass

    return first_class_nodes, second_class_nodes

def expand_first_class_nodes_by_symmetry(first_class_nodes):
    """
    根据一类节点的对称性，生成完整节点信息（无对称性字段）
    """

    expanded_nodes = {}

    for node in first_class_nodes:
        nid = node["node_id"]
        sym = node["symmetry"]
        x, y, z = node["x"], node["y"], node["z"]

        # 1️⃣ 原节点（一定保留）
        expanded_nodes[nid] = {
            "node_id": nid,
            "x": x,
            "y": y,
            "z": z
        }

        # 2️⃣ 对称节点生成
        if sym == 1 or sym == 4:
            # 关于 y 轴对称
            expanded_nodes[nid + 1] = {
                "node_id": nid + 1,
                "x": -x,
                "y": y,
                "z": z
            }

        if sym == 2 or sym == 4:
            # 关于 x 轴对称
            expanded_nodes[nid + 2] = {
                "node_id": nid + 2,
                "x": x,
                "y": -y,
                "z": z
            }

        if sym == 3 or sym == 4:
            # 关于 z 轴对称（X、Y 互为正负）
            expanded_nodes[nid + 3] = {
                "node_id": nid + 3,
                "x": -x,
                "y": -y,
                "z": z
            }

    return expanded_nodes

def parse_second_node(node):

    coords = {"x": node["x"], "y": node["y"], "z": node["z"]}
    id_dims = []

    for dim, v in coords.items():
        if float(v).is_integer() and abs(v) >= 10000:
            id_dims.append(dim)

    # 剩下那个就是真实维度
    real_dim = ({"x", "y", "z"} - set(id_dims)).pop()
    real_value = coords[real_dim]

    ref_id1 = int(coords[id_dims[0]] - 10000)
    ref_id2 = int(coords[id_dims[1]] - 10000)

    return real_dim, real_value, ref_id1, ref_id2



# def compute_second_node_xyz(node, expanded_nodes):
#     real_dim, real_value, id1, id2 = parse_second_node(node)
#
#     A = expanded_nodes[id1]
#     B = expanded_nodes[id2]
#
#     x1, y1, z1 = A["x"], A["y"], A["z"]
#     x2, y2, z2 = B["x"], B["y"], B["z"]
#
#     if real_dim == "x":
#         t = (real_value - x1) / (x2 - x1)
#         x = real_value
#         y = y1 + t * (y2 - y1)
#         z = z1 + t * (z2 - z1)
#
#     elif real_dim == "y":
#         t = (real_value - y1) / (y2 - y1)
#         y = real_value
#         x = x1 + t * (x2 - x1)
#         z = z1 + t * (z2 - z1)
#
#     else:  # z
#         t = (real_value - z1) / (z2 - z1)
#         z = real_value
#         x = x1 + t * (x2 - x1)
#         y = y1 + t * (y2 - y1)
#
#     return x, y, z
#

def compute_second_node_xyz(node, expanded_nodes):
    real_dim, real_value, id1, id2 = parse_second_node(node)

    A = expanded_nodes[id1]
    B = expanded_nodes[id2]

    x1, y1, z1 = A["x"], A["y"], A["z"]
    x2, y2, z2 = B["x"], B["y"], B["z"]

    EPS = 1e-8  # 浮点容差

    if real_dim == "x":
        dx = x2 - x1
        if abs(dx) < EPS:
            # 杆件在x方向没有变化，无法用于x插值
            return None
        t = (real_value - x1) / dx
        x = real_value
        y = y1 + t * (y2 - y1)
        z = z1 + t * (z2 - z1)

    elif real_dim == "y":
        dy = y2 - y1
        if abs(dy) < EPS:
            return None
        t = (real_value - y1) / dy
        y = real_value
        x = x1 + t * (x2 - x1)
        z = z1 + t * (z2 - z1)

    else:  # real_dim == "z"
        dz = z2 - z1
        if abs(dz) < EPS:
            return None
        t = (real_value - z1) / dz
        z = real_value
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)

    # 👉 可选：限制 t 在合理范围内（说明节点在线段附近）
    if t < -0.2 or t > 1.2:
        # 偏离太远，说明选错参考杆
        return None

    return x, y, z


def resolve_second_class_nodes(
    second_class_nodes,
    expanded_nodes
):
    unresolved = second_class_nodes.copy()
    round_idx = 0

    while unresolved:
        round_idx += 1
        print(f"\n开始第 {round_idx} 轮二类节点求解，剩余 {len(unresolved)} 个")

        newly_resolved = []

        for node in unresolved:
            _, _, id1, id2 = parse_second_node(node)

            if id1 in expanded_nodes and id2 in expanded_nodes:
                # 可以计算
                #x, y, z = compute_second_node_xyz(node, expanded_nodes)
                result = compute_second_node_xyz(node, expanded_nodes)
                if result is None:
                    continue  # 换下一根参考杆再试
                x, y, z = result

                base_node = {
                    "node_id": node["node_id"],
                    "x": x,
                    "y": y,
                    "z": z
                }

                # 加入原节点
                expanded_nodes[node["node_id"]] = base_node

                # 根据对称性生成对称节点（复用你之前的一类逻辑）
                sym = node["symmetry"]
                nid = node["node_id"]

                if sym == 1 or sym == 4:
                    expanded_nodes[nid + 1] = {
                        "node_id": nid + 1, "x": -x, "y": y, "z": z
                    }
                if sym == 2 or sym == 4:
                    expanded_nodes[nid + 2] = {
                        "node_id": nid + 2, "x": x, "y": -y, "z": z
                    }
                if sym == 3 or sym == 4:
                    expanded_nodes[nid + 3] = {
                        "node_id": nid + 3, "x": -x, "y": -y, "z": z
                    }

                newly_resolved.append(node)

        if not newly_resolved:
            print("⚠️ 本轮未能解析任何新二类节点，终止")
            break

        for n in newly_resolved:
            unresolved.remove(n)

    return expanded_nodes


def plot_3d_nodes(expanded_nodes):
    """
    在三维空间中绘制所有节点
    """
    xs, ys, zs = [], [], []

    for node in expanded_nodes.values():
        xs.append(node["x"])
        ys.append(node["y"])
        zs.append(node["z"])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(xs, ys, zs, s=10)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_title("3D Node Distribution")

    plt.show()




def load_rods_from_excel(excel_path, sheet_index=1):
    """
    从 Excel 第二个 sheet 读取杆件信息
    返回：杆件列表，每个元素是 dict
    """
    df = pd.read_excel(excel_path, sheet_name=sheet_index)

    rods = []
    for _, row in df.iterrows():
        # 安全处理带有 F_ 或 R_ 前缀的杆件编号
        raw_id = str(row.iloc[0])
        clean_id = raw_id.replace("F_", "").replace("R_", "")
        # 如果还有下划线后缀（如 503_1），只取前面纯数字部分
        clean_id = clean_id.split('_')[0] 
        
        try:
            final_rod_id = int(clean_id)
        except ValueError:
            final_rod_id = 0 # 兜底防崩溃

        rod = {
            "rod_id": final_rod_id,
            "start": int(row.iloc[1]),
            "end": int(row.iloc[2]),
            "symmetry": int(row.iloc[3])
        }
        rods.append(rod)

    return rods

def plot_3d_rods(expanded_nodes, rods, show_rod_ids=False, rod_label_filter=None):
    """
    在三维空间中绘制杆件（直线）
    同时统计并输出丢失杆件及缺失端点信息
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    def _rod_in_drawing(rod_id, drawing_no):
        try:
            rid = int(rod_id)
        except (TypeError, ValueError):
            return False
        base = int(drawing_no) * 100
        return base <= rid < base + 100

    def _print_rod_height_summary(drawings=(7, 11)):
        summary = {}
        for drawing_no in drawings:
            matched = [
                rod for rod in rods
                if _rod_in_drawing(rod.get("rod_id"), drawing_no)
            ]
            z_values = []
            missing = 0
            for rod in matched:
                for endpoint in ("start", "end"):
                    node = expanded_nodes.get(rod[endpoint])
                    if node is None:
                        missing += 1
                    else:
                        z_values.append(node["z"])
            if z_values:
                summary[drawing_no] = (min(z_values), max(z_values), len(matched), missing)
                print(
                    f"[height-check] 图纸{drawing_no}: 杆件={len(matched)}, "
                    f"Z={min(z_values):.3f}~{max(z_values):.3f}, 缺失端点={missing}"
                )
            else:
                print(f"[height-check] 图纸{drawing_no}: 未找到可绘制杆件")
        return summary

    height_summary = _print_rod_height_summary()

    # ===============================
    # 1. 绘制节点（淡色）
    # ===============================
    xs, ys, zs = [], [], []
    for node in expanded_nodes.values():
        xs.append(node["x"])
        ys.append(node["y"])
        zs.append(node["z"])

    ax.scatter(xs, ys, zs, s=5, alpha=0.3)

    # ===============================
    # 2. 绘制杆件 + 统计缺失
    # ===============================
    missing_rods = []   # 存详细信息

    for rod in rods:
        s_id = rod["start"]
        e_id = rod["end"]

        s_exist = s_id in expanded_nodes
        e_exist = e_id in expanded_nodes

        # ❌ 有缺失
        if not s_exist or not e_exist:
            missing_info = {
                "rod_id": rod.get("rod_id"),
                "start": s_id,
                "end": e_id,
                "missing_start": not s_exist,
                "missing_end": not e_exist
            }
            missing_rods.append(missing_info)
            continue

        # ✅ 正常绘制
        A = expanded_nodes[s_id]
        B = expanded_nodes[e_id]

        ax.plot(
            [A["x"], B["x"]],
            [A["y"], B["y"]],
            [A["z"], B["z"]],
            linewidth=1
        )

        if show_rod_ids:
            rod_id = rod.get("rod_id")
            if rod_label_filter is None or rod_id in rod_label_filter:
                mid_x = (A["x"] + B["x"]) / 2
                mid_y = (A["y"] + B["y"]) / 2
                mid_z = (A["z"] + B["z"]) / 2
                ax.text(
                    mid_x,
                    mid_y,
                    mid_z,
                    str(rod_id),
                    fontsize=7,
                    color="red",
                )

    # ===============================
    # 3. 坐标 & 显示
    # ===============================
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass
    ax.view_init(elev=0, azim=-90)

    title = "3D Rod Structure"
    if 7 in height_summary and 11 in height_summary:
        z7 = height_summary[7]
        z11 = height_summary[11]
        title = (
            f"3D Rod Structure | 7: {z7[0]:.1f}-{z7[1]:.1f} "
            f"| 11: {z11[0]:.1f}-{z11[1]:.1f}"
        )
    ax.set_title(title)

    # ===============================
    # ✅ 自动根据数据扩大三维显示范围（关键）
    # ===============================
    all_x, all_y, all_z = [], [], []

    # 所有节点坐标
    for node in expanded_nodes.values():
        all_x.append(node["x"])
        all_y.append(node["y"])
        all_z.append(node["z"])

    xmin, xmax = min(all_x), max(all_x)
    ymin, ymax = min(all_y), max(all_y)
    zmin, zmax = min(all_z), max(all_z)

    max_range = max(xmax - xmin, ymax - ymin, zmax - zmin)

    x_mid = (xmax + xmin) / 2
    y_mid = (ymax + ymin) / 2
    z_mid = (zmax + zmin) / 2

    scale = 1.2  # 🔥 调大这个数，缩放时就更不容易被裁剪

    ax.set_xlim(x_mid - max_range / 2 * scale, x_mid + max_range / 2 * scale)
    ax.set_ylim(y_mid - max_range / 2 * scale, y_mid + max_range / 2 * scale)
    ax.set_zlim(z_mid - max_range / 2 * scale, z_mid + max_range / 2 * scale)

    # 保证XYZ比例一致，不会被拉扁
    ax.set_box_aspect([1, 1, 1])
    plt.show()

    # ===============================
    # 4. 输出丢失杆件信息
    # ===============================
    print(f"\n⚠️ 丢失杆件总数：{len(missing_rods)}")

    for info in missing_rods:
        miss = []
        if info["missing_start"]:
            miss.append("start")
        if info["missing_end"]:
            miss.append("end")

        print(
            f"杆件 rod_id={info['rod_id']} "
            f"(start={info['start']}, end={info['end']}) "
            f"缺少端点：{', '.join(miss)}"
        )

    # 如果你后续还想程序化处理，直接返回
    #return missing_rods


def expand_rods_by_symmetry(rods, expanded_nodes):
    """
    根据杆件 symmetry 生成对称杆件
    返回：新的杆件列表（包含原始杆件）
    """
    expanded_rods = []
    used = set()  # 防止重复

    for rod in rods:
        rid = rod["rod_id"]
        s = rod["start"]
        e = rod["end"]
        sym = rod["symmetry"]

        # 原始杆件
        key = tuple(sorted((s, e)))
        if key not in used:
            expanded_rods.append({
                "rod_id": rid,
                "start": s,
                "end": e
            })
            used.add(key)

        # 对称映射规则
        sym_map = []

        if sym == 1:
            sym_map = [1]
        elif sym == 2:
            sym_map = [2]
        elif sym == 3:
            sym_map = [3]
        elif sym == 4:
            sym_map = [1, 2, 3]

        for sym_type in sym_map:
            ns = map_node(s, sym_type)
            ne = map_node(e, sym_type)

            if ns is None or ne is None:
                continue

            if ns not in expanded_nodes or ne not in expanded_nodes:
                continue

            key = tuple(sorted((ns, ne)))
            if key in used:
                continue

            expanded_rods.append({
                "rod_id": rid * 10 + sym_type,
                "start": ns,
                "end": ne
            })
            used.add(key)

    return expanded_rods


def map_node(node, sym_type):
    base = (node // 10) * 10
    tail = node % 10

    if sym_type not in SYMMETRY_MAP:
        return None

    mapping = SYMMETRY_MAP[sym_type]

    if tail not in mapping:
        return None

    return base + mapping[tail]

SYMMETRY_MAP = {
    1: {0:1, 1:0, 2:3, 3:2},  # 左右
    2: {0:2, 2:0, 1:3, 3:1},  # 前后
    3: {0:3, 3:0, 1:2, 2:1},  # Z轴
}

if __name__ == "__main__":
    excel_path = r"D:\Sanwei\project_path\3d_result\zhenghe_data.xlsx"

    # excel_path = r"D:\Sanwei\output_path\draw_3d\1E2-SDJ\zhenghe_data.xlsx"
    # excel_path = r"D:\Sanwei\output_path\draw_3d\J1\zhenghe_data.xlsx"

    # excel_path = r"D:\Sanwei\output_path\draw_3d\J1\tashen_zhenghe_data.xlsx"
    #excel_path = r"D:\Sanwei\output_path\draw_3d\J1\5_zhenghe_data.xlsx"
    #excel_path = r"D:\Sanwei\output_path\draw_3d\J1\6_zhenghe_data.xlsx"
    # excel_path = r"D:\Sanwei\output_path\draw_3d\J1\7_zhenghe_data.xlsx"
    #excel_path = r"D:\Sanwei\output_path\draw_3d\J1\11_zhenghe_data.xlsx"
    # excel_path = r"D:\Sanwei\output_path\draw_3d\J1\danjia_zhenghe_data.xlsx"
    # excel_path = r"D:\Sanwei\output_path\draw_3d\J1\danjia_zhenghe_data_half.xlsx"

    nodes_3d = load_nodes_from_excel(excel_path)

    print("共读取节点数量：", len(nodes_3d))
    print("前 3 个节点示例：")
    for n in nodes_3d[:3]:
        print(n)

    first_class_nodes, second_class_nodes = classify_nodes(nodes_3d)

    print("一类节点数量：", len(first_class_nodes))
    print("二类节点数量：", len(second_class_nodes))

    print("\n一类节点示例：")
    for n in first_class_nodes[:3]:
        print(n)

    print("\n二类节点示例：")
    for n in second_class_nodes[:3]:
        print(n)


    # 对一类节点做对称展开
    expanded_nodes = expand_first_class_nodes_by_symmetry(first_class_nodes)

    print("原始一类节点数量：", len(first_class_nodes))
    print("展开后节点总数：", len(expanded_nodes))

    print("\n展开后的节点示例：")
    for k in list(expanded_nodes.keys())[:5]:
        print(expanded_nodes[k])

    # ===============================
    # 求解所有二类节点真实坐标
    # ===============================
    expanded_nodes = resolve_second_class_nodes(
        second_class_nodes,
        expanded_nodes
    )

    print("\n=== 二类节点解析完成 ===")
    print("最终节点总数（含一类 + 二类 + 对称）：", len(expanded_nodes))

    print("\n最终节点示例：")
    for k in list(expanded_nodes.keys())[:10]:
        print(expanded_nodes[k])

    # ===============================
    # 三维可视化
    # ===============================
    #plot_3d_nodes(expanded_nodes)


    # ===============================
    # 新增：读取杆件 & 绘制
    # ===============================
    rods = load_rods_from_excel(excel_path)

    print("读取杆件数量：", len(rods))
    print("前 3 根杆件示例：", rods[:3])

   # plot_3d_rods(expanded_nodes, rods)

    # 原始杆件
    rods = load_rods_from_excel(excel_path)

    # 对称展开杆件
    expanded_rods = expand_rods_by_symmetry(rods, expanded_nodes)

    print("原始杆件数量：", len(rods))
    print("对称展开后杆件数量：", len(expanded_rods))


    plot_3d_rods(
        expanded_nodes,
        expanded_rods,
        show_rod_ids=False,
        # rod_label_filter={701, 702, 703,101, 102, 103, 108,117,113,114,201, 203, 202, 301, 302, 303, 401, 402,406,501, 502, 503, 601, 602,607,801,802,901,902},
    )

