import math
import io_utils as rw

# ================= 三维转换相关子函数 =================

# 提取一类杆件（仅保留后缀 01 / 02）
def extract_target_members(lines_dict):
    target_members = {}
    for member_id, coordinates in lines_dict.items():
        member_str = str(member_id)
        if "_" in member_str:
            int_part = member_str.split("_", 1)[0]
        else:
            int_part = member_str
        end_suffix = int_part[-2:] if len(int_part) >= 2 else ""
        if end_suffix in ["01", "02"]:
            target_members[member_id] = coordinates
    return target_members


# 一类杆件所需参数读取
def parameter_extract(lines_dict):
    lines = list(lines_dict.values())
    if len(lines) < 2:
        raise ValueError("提取一类杆件失败：需要至少两条杆件用于参数计算。")

    kb_list = []
    for line in lines:
        (x1, y1), (x2, y2) = line
        if abs(x2 - x1) < 1e-9:
            raise ValueError("杆件端点 X 相同，无法计算斜率。")
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        kb_list.append([k, b])

    if kb_list[0][0] < 0:
        left_line = 0
        right_line = 1
    else:
        left_line = 1
        right_line = 0

    (x1_l, y1_l), (x2_l, y2_l) = lines[left_line]
    y_left_min = min(y1_l, y2_l)
    y_left_max = max(y1_l, y2_l)

    (x1_r, y1_r), (x2_r, y2_r) = lines[right_line]
    y_right_min = min(y1_r, y2_r)
    y_right_max = max(y1_r, y2_r)

    y_avg_min = (y_left_min + y_right_min) / 2.0
    y_avg_max = (y_left_max + y_right_max) / 2.0

    k_avg = (abs(kb_list[0][0]) + abs(kb_list[1][0])) / 2.0

    x1_left = (y_avg_min - kb_list[left_line][1]) / (-k_avg)
    x2_left = (y_avg_max - kb_list[left_line][1]) / (-k_avg)
    x1_right = (y_avg_min - kb_list[right_line][1]) / k_avg
    x2_right = (y_avg_max - kb_list[right_line][1]) / k_avg

    h = round(abs(y_avg_max - y_avg_min))
    a = round(abs(x1_right - x1_left))
    b = round(abs(x2_right - x2_left))
    return h, a, b


# 一类杆件坐标转换（生成单根基准杆的 3D 坐标）
def single_view_trans01(lines_dict, h, a, b):
    res_key = None
    for key in lines_dict:
        if str(key).endswith("01"):
            res_key = key
            break
    if res_key is None and lines_dict:
        res_key = next(iter(lines_dict.keys()))
    if res_key is None:
        return {}

    half_a = round(a / 2.0 / 1000.0, 3)
    half_b = round(b / 2.0 / 1000.0, 3)
    span_sq = max(h ** 2 - ((b - a) / 2.0) ** 2, 0.0)
    z_top = round(math.sqrt(span_sq) / 1000.0, 3)

    res_line = {
        res_key: [
            (-half_a, -half_a, 0.0),
            (-half_b, -half_b, z_top),
        ]
    }

    return res_line


# 一类杆件坐标总转换
def translate_single_front_3d_coordinates01(lines_dict):
    line_extracted = extract_target_members(lines_dict)
    if not line_extracted:
        return {}
    h, a, b = parameter_extract(line_extracted)
    return single_view_trans01(line_extracted, h, a, b)


# ================= 格式转换相关子函数 =================

def convert_member_format(original_members):
    member_collection = []
    for member_id, coordinates in original_members.items():
        if len(coordinates) != 2:
            print(f"警告：杆件 {member_id} 的端点数量异常（{len(coordinates)}）已跳过转换")
            continue
        node1_id = f"{member_id}10"
        node2_id = f"{member_id}20"
        member_collection.append({
            "member_id": str(member_id),
            "node1_id": str(node1_id),
            "node2_id": str(node2_id),
            "symmetry_type": 4,
        })
    return member_collection


def convert_node_format(original_data):
    result = []
    for rod_id, endpoints in original_data.items():
        for index, coords in enumerate(endpoints, start=1):
            node_id = f"{rod_id}{10 if index == 1 else 20}"
            node = {
                "node_id": str(node_id),
                "node_type": 11,
                "symmetry_type": 4,
                "X": coords[0],
                "Y": coords[1],
                "Z": coords[2],
            }
            result.append(node)
    return result


# ================= 总函数 =================

def single_view01(line_coord):
    res_lines = translate_single_front_3d_coordinates01(line_coord)
    ganjian = convert_member_format(res_lines)
    jiedian = convert_node_format(res_lines)
    return ganjian, jiedian


# ====== 辅助接口：提供一类 3D 字典与顶端杆件两端点 ======

def build_final_map_single_view(line_coord):
    lines3d = translate_single_front_3d_coordinates01(line_coord)
    if not lines3d:
        return {}, None
    special_key = None
    for key in lines3d.keys():
        if str(key).endswith("01"):
            special_key = key
            break
    if special_key is None:
        special_key = next(iter(lines3d.keys()))
    special_str = str(special_key)
    return {special_str: lines3d[special_key]}, special_str


def extract_top_span_points_single(line_coord):
    line_extracted = extract_target_members(line_coord)
    if not line_extracted:
        return None, None
    h, a, b = parameter_extract(line_extracted)
    span_sq = max(h * h - ((b - a) / 2.0) * ((b - a) / 2.0), 0.0)
    z_top = round(math.sqrt(span_sq) / 1000.0, 6)
    half = round(b / 2.0 / 1000.0, 6)
    UL = (-half, -half, z_top)
    UR = (half, half, z_top)
    return UL, UR