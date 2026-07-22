import math
import io_utils as rw

def segment_length(p1, p2):
    return math.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1]))


def _numeric_id(member_id):
    try:
        return int(str(member_id))
    except (TypeError, ValueError):
        return float("inf")


def _member_sort_key(member_id):
    num_id = _numeric_id(member_id)
    if num_id == float("inf"):
        return (1, str(member_id))
    return (0, num_id)


def _is_main_rod_candidate(member_id):
    """Return whether the base drawing ID is eligible to be a class-1 rod."""
    raw_id = str(member_id).strip()
    base_id, separator, instance = raw_id.rpartition("_")
    if separator and instance.isdigit():
        raw_id = base_id
    return raw_id.endswith(("01", "02", "03"))


def detect_main_rods_enhanced(coordinates_data, top_k=2):
    """
    Detect class-1/main rods among IDs ending in 01, 02, or 03.

    The eligible candidates are ranked by length first, then fall back to the
    smallest eligible IDs. Duplicate-instance suffixes such as ``_1`` are
    ignored only when evaluating the drawing ID suffix.
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
        num_id = _numeric_id(rod_id)
        all_ids.append((rod_id, num_id))
        p1, p2 = endpoints
        rod_items.append((rod_id, num_id, segment_length(p1, p2)))

    if len(rod_items) < top_k:
        return []

    rod_items.sort(key=lambda item: item[2], reverse=True)
    candidates = [item[0] for item in rod_items[:top_k]]

    all_ids.sort(key=lambda item: (item[1], str(item[0])))
    min_two_ids = [item[0] for item in all_ids[:top_k]]
    if len(min_two_ids) < top_k:
        return []

    candidates_set = set(candidates)
    min_two_set = set(min_two_ids)
    min_one = min_two_ids[0]

    if candidates_set == min_two_set:
        result = min_two_ids
    elif min_one in candidates_set:
        result = candidates
    else:
        result = min_two_ids

    return sorted(result, key=_member_sort_key)


def extract_target_members(lines_dict, main_rod_ids=None):
    target_ids = main_rod_ids or detect_main_rods_enhanced(lines_dict)
    return {member_id: lines_dict[member_id] for member_id in target_ids if member_id in lines_dict}

# 一类杆件所需参数读取
def parameter_extract(lines_dict):
    lines = list(lines_dict.values())
    # if len(lines) < 2:
    #     raise ValueError("提取一类杆件失败：需要至少两条杆件用于参数计算。")
    if len(lines) < 1:
        raise ValueError("提取一类杆件失败：需要至少一条杆件用于参数计算。")

    # 如果只有一条杆件，对称它以获得完整的参数
    if len(lines) == 1:
        line = lines[0]
        (x1, y1), (x2, y2) = line
        # 添加关于Y轴对称的杆件
        symmetric_line = ((-x1, y1), (-x2, y2))
        lines = [line, symmetric_line]

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
def translate_single_front_3d_coordinates01(lines_dict, main_rod_ids=None):
    line_extracted = extract_target_members(lines_dict, main_rod_ids)
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

def single_view01(line_coord, main_rod_ids=None):
    res_lines = translate_single_front_3d_coordinates01(line_coord, main_rod_ids)
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
