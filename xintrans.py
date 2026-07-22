import math
import os
from collections import defaultdict
from get_first_ganjian_id import detect_main_rods_enhanced


def count_txt_files(folder_path):
    """
    统计文件夹内 .txt 文件数量（不递归）
    """
    try:
        entries = os.listdir(folder_path)
    except FileNotFoundError:
        return 0

    return sum(1 for name in entries if name.lower().endswith(".txt"))

def dist_points(p1, p2):
    """计算点之间距离"""
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_between_lines(line1_pt1, line1_pt2, line2_pt1, line2_pt2):
    """
    计算两条直线的锐角夹角（0-90度）的简化版本

    参数:
        line1_pt1: 第一条直线的第一个端点 (x1, y1)
        line1_pt2: 第一条直线的第二个端点 (x2, y2)
        line2_pt1: 第二条直线的第一个端点 (x3, y3)
        line2_pt2: 第二条直线的第二个端点 (x4, y4)

    返回:
        两条直线的锐角夹角（单位：度，范围0-90度）
    """
    # 计算方向向量
    v1 = (line1_pt2[0] - line1_pt1[0], line1_pt2[1] - line1_pt1[1])
    v2 = (line2_pt2[0] - line2_pt1[0], line2_pt2[1] - line2_pt1[1])

    # 计算点积和模
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    # 检查零向量
    if mag1 == 0 or mag2 == 0:
        return 0

    # 计算夹角
    cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    angle_deg = math.degrees(math.acos(cos_angle))

    # 返回锐角
    return min(angle_deg, 180 - angle_deg)

def dist_point_to_line(point, line_point1, line_point2):
    """
    计算点到直线的垂直距离（垂线段长度）

    参数:
        point: 点的坐标 (x, y)
        line_point1: 直线上的第一个点 (x1, y1)
        line_point2: 直线上的第二个点 (x2, y2)

    返回:
        点到直线的垂直距离
    """
    x, y = point
    x1, y1 = line_point1
    x2, y2 = line_point2

    # 如果直线是垂直的
    if x1 == x2:
        return abs(x - x1)

    # 如果直线是水平的
    if y1 == y2:
        return abs(y - y1)

    # 计算直线的一般式方程 Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2

    # 计算点到直线的距离
    distance = abs(A * x + B * y + C) / math.sqrt(A ** 2 + B ** 2)

    return distance

def line_intersection(line1, line2):
    """
    计算两条直线的交点，返回交点坐标（x,y）

    参数:
        line1: 第一条直线，格式为 [(x1, y1), (x2, y2)]
        line2: 第二条直线，格式为 [(x3, y3), (x4, y4)]

    返回:
        交点坐标 (x, y)，如果直线平行则返回 None
    """
    # 提取坐标
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # 计算分母
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    # 如果分母为0，则直线平行或重合
    if denominator == 0:
        return None

    # 计算交点坐标
    x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

    return (x, y)

def dist_z(p1, p2):
    """
    计算三维点之间的欧氏距离
    """
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def transform_data(input_data):
    """
    将输入的节点数据按每4个一组进行分组

    示例:
        输入: [1, 2, 3, 4, 5, 6, 7, 8]
        输出: [[1, 2, 3, 4], [5, 6, 7, 8]]
    """
    # 将数据分成4组，每组4个节点
    groups = []
    for i in range(0, len(input_data), 4):
        group = input_data[i:i + 4]
        groups.append(group)

    return groups

def cluster_points(points, threshold):
    """
    对点进行聚类，如果当前点与前一个点距离小于阈值，则合并到前一个聚类

    示例:
        输入: points=[(0, 0), (1, 1), (10, 10)], threshold=3
        输出: [(0.5, 0.5), (10, 10)]
    """
    clusters = []

    for point in points:
        if not clusters:
            # 第一个点，直接作为第一个聚类
            clusters.append({
                "points": [point],
                "centroid": point
            })
        else:
            # 计算与最后一个聚类中心的距离
            last_centroid = clusters[-1]["centroid"]
            distance = dist_points(point, last_centroid)

            if distance < threshold:
                # 合并到最后一个聚类
                clusters[-1]["points"].append(point)
                # 更新聚类中心为所有点的平均值
                all_points = clusters[-1]["points"]
                avg_x = sum(p[0] for p in all_points) / len(all_points)
                avg_y = sum(p[1] for p in all_points) / len(all_points)
                clusters[-1]["centroid"] = (avg_x, avg_y)
            else:
                # 创建新聚类
                clusters.append({
                    "points": [point],
                    "centroid": point
                })

    # 返回聚类中心点
    return [cluster["centroid"] for cluster in clusters]

def mark_endpoint_for_real_points(real_points,coordinates_data,rod_id,left_endpoint_3d_id,right_endpoint_3d_id,threshold):
    """
    判断交点是否为杆件端点之一，若是则用端点的三维节点编号标记
    """

    # 杆件的两个二维端点
    rod_p1, rod_p2 = coordinates_data[rod_id]

    for item in real_points:
        px, py = item["point_2d"]

        d1 = dist_points((px, py), rod_p1)
        d2 = dist_points((px, py), rod_p2)

        if d1 < threshold:
            item["endpoint_3d_id"] = left_endpoint_3d_id
        elif d2 < threshold:
            item["endpoint_3d_id"] = right_endpoint_3d_id
        else:
            item["endpoint_3d_id"] = -1

    return real_points

def find_ganjian_by_nodes(node_list, coordinates_data, threshold):
    """
    根据节点列表，通过节点的二维坐标查找对应的杆件编号
    """
    node_to_members = {}

    for node in node_list:
        node_id = node["node_id"]
        x0, y0 = node["point_2d"]

        matched_members = []

        for member_id, endpoints in coordinates_data.items():
            for (x, y) in endpoints:
                dist = math.hypot(x - x0, y - y0)
                if dist < threshold:
                    matched_members.append(member_id)
                    break  # 一个端点命中即可

        node_to_members[node_id] = matched_members

    return node_to_members

def calc_jiandian_xyz(coordinates_data, drawing_id, pj,pj_view_index, rod_a_id, rod_b_id):
    """
    pj_view_index：0代表担架的上端点，1代表担架的下端点
    rod_a_id, rod_b_id：当前视图中两根一类杆件的实际编号
    生成尖点的真实XYZ的值
    输出：10020：（x,y,z）
    """
    start_01x = coordinates_data[rod_a_id][0][0]
    start_01y = coordinates_data[rod_a_id][0][1]
    start_02x = coordinates_data[rod_b_id][0][0]
    start_02y = coordinates_data[rod_b_id][0][1]
    end_01x = coordinates_data[rod_a_id][1][0]
    end_01y = coordinates_data[rod_a_id][1][1]
    end_02x = coordinates_data[rod_b_id][1][0]
    end_02y = coordinates_data[rod_b_id][1][1]
    midr = ((end_01x + end_02x) / 2, (end_01y + end_02y) / 2)  # 计算301和302两个右端点的中点
    midl = ((start_01x + start_02x) / 2, (start_01y + start_02y) / 2)  # 计算301和302两个左端点的中点
    h = dist_points(midl, midr)  # 计算左中点到右中点的距离
    p_01l = (start_01x, start_01y)
    a1 = dist_points(p_01l, midl)  # 起点侧一类杆件到起点中点的距离（起点侧像素半宽）
    p_01r = (end_01x, end_01y)
    a2 = dist_points(p_01r, midr)  # 终点侧一类杆件到终点中点的距离（终点侧像素半宽）
    shiji = abs(pj[drawing_id - 1][pj_view_index][1][1])  # 计算担架3与塔身相交的下端点的三维x坐标的值，也就是实际值
    # 常规图纸：基座在起点侧，用起点侧半宽 a1 做像素→真实比例、终点侧半宽 a2 算 newy。
    # 特例（如 7837/01）：两根一类杆件在起点处收拢，a1==0 会除零；此时基座实为终点侧，
    # 回退用 a2 做比例、a1 算 newy。只有 a1==0 时才切换，其余图纸行为完全不变。
    if a1 != 0:
        base_half, apex_half = a1, a2
    else:
        base_half, apex_half = a2, a1
    bili = shiji / base_half  # 计算实际值/像素值的比例
    # 生成尖点
    if (pj[drawing_id - 1][pj_view_index][1][0] > 0):  # if里面生成右边担架的尖点
        newx = pj[drawing_id - 1][pj_view_index][1][0] + h * bili
        newy = -apex_half * bili
        newz = pj[drawing_id - 1][pj_view_index][1][2]
    else:  # else生成左边担架的尖点
        newx = pj[drawing_id - 1][pj_view_index][1][0] - h * bili
        newy = -apex_half * bili
        newz = pj[drawing_id - 1][pj_view_index][1][2]
    return newx, newy, newz

def get_jiaodian_on_ganjian(coordinates_data, drawing_id, pj, rod_101_id, rod_103_id, yuzhi):
    """
    找二类杆件与一类杆件在一类杆件上的交点坐标

    输入：
    rod_101_id：101杆件的编号
    rod_103_id：103杆件的编号

    输出：
    node_101: [(10,20),(50,70),(90,100)] 表示101一类杆件上有三个交点（二类节点）
    node_103同理
    """
    rod_101_points = coordinates_data[rod_101_id]
    rod_103_points = coordinates_data[rod_103_id]
    intersections_101 = []
    intersections_103 = []

    # 遍历所有杆件
    # rod_id：杆件编号（如 301、302、303 …），points：该杆件的两个端点（二维）
    for rod_id, points in coordinates_data.items():
        if rod_id in [rod_101_id, rod_103_id]:  # 如果是301、303杆件就跳过
            continue
        flag1 = 0  # flag1 用来统计该杆件有多少端点靠近参考杆件
        for i, point in enumerate(points):
            distance_to_101 = dist_point_to_line(point, rod_101_points[0], rod_101_points[1])
            distance_to_103 = dist_point_to_line(point, rod_103_points[0], rod_103_points[1])
            if (distance_to_103 < yuzhi or distance_to_101 < yuzhi):  # 如果端点只要靠近任意一根参考杆，flag1就加1
                flag1 += 1
        if flag1 == 2:  # 该杆件的两个端点都靠近参考杆件
            intersection_101 = line_intersection(points, rod_101_points)
            if intersection_101 is not None:
                intersections_101.append(intersection_101)  # 求杆件与 301 的交点坐标

            # 计算目标杆件与303杆件的交点坐标
            intersection_103 = line_intersection(points, rod_103_points)
            if intersection_103 is not None:
                intersections_103.append(intersection_103)

    # 对在301杆件上的交点这个数组的元素进行排序，按照二维坐标的x轴从小到大进行排序
    if (pj[drawing_id - 1][1][1][0] > 0):
        intersections_101.sort(key=lambda point: point[0])
    else:
        intersections_101.sort(key=lambda point: point[0], reverse=True)
    node_101 = cluster_points(intersections_101, threshold=yuzhi)  # 对杆件上的交点进行聚类得到node_101[]这个聚类后的交点数组
    if (pj[drawing_id - 1][1][1][0] > 0):
        intersections_103.sort(key=lambda point: point[0])
    else:
        intersections_103.sort(key=lambda point: point[0], reverse=True)
    node_103 = cluster_points(intersections_103, threshold=yuzhi)

    return node_101,node_103

def get_jiaodian_on_ganjian_by_missing_members(
    coordinates_data,
    missing_members,
    existing_member_ids,
    drawing_id,
    rod_101_id, 
    rod_103_id,
    pj,
    yuzhi,
):
    """
    找三类杆件与一类杆件、二类杆件上的交点坐标

    """

    intersections_existing = defaultdict(list)

    def is_point_on_segment(point, seg_p1, seg_p2, threshold):
        if dist_point_to_line(point, seg_p1, seg_p2) >= threshold:
            return False
        min_x = min(seg_p1[0], seg_p2[0]) - threshold
        max_x = max(seg_p1[0], seg_p2[0]) + threshold
        min_y = min(seg_p1[1], seg_p2[1]) - threshold
        max_y = max(seg_p1[1], seg_p2[1]) + threshold
        return min_x <= point[0] <= max_x and min_y <= point[1] <= max_y

    # 预先整理二类杆件编号，并建立字符串->原始键的映射，避免类型不一致
    second_class_ids = {
        str(m_id)
        for m_id in (existing_member_ids or [])
    }
    second_class_ids.update({str(rod_101_id), str(rod_103_id)})
    second_class_key_map = {str(m_id): m_id for m_id in (coordinates_data or {}).keys()}

    # 遍历三类杆件的二维端点，若端点落在二类杆件上则记录
    for rod_id, points in (missing_members or {}).items():
        for point in points:
            for second_id in second_class_ids:
                key = second_class_key_map.get(second_id)
                if key is None:
                    continue
                second_points = coordinates_data.get(key)
                if not second_points:
                    continue
                if is_point_on_segment(point, second_points[0], second_points[1], yuzhi):
                    intersections_existing[str(second_id)].append(point)


    node_existing = {}
    # 对每个二类杆件的交点排序+聚类
    for second_id, points in intersections_existing.items():
        # 保持排序方向与当前担架方向一致
        if pj[drawing_id - 1][1][1][0] > 0:
            points.sort(key=lambda point: point[0])
        else:
            points.sort(key=lambda point: point[0], reverse=True)
        # 聚类以去除近邻重复交点
        node_existing[second_id] = cluster_points(points, threshold=yuzhi)

    # 去除与二类杆件端点距离过近的交点
    endpoint_points = []
    endpoint_ids = set(second_class_ids)
    for member_id in endpoint_ids:
        key = second_class_key_map.get(member_id)
        if key is None:
            continue
        endpoints = coordinates_data.get(key)
        if not endpoints:
            continue
        endpoint_points.append(endpoints[0])
        endpoint_points.append(endpoints[1])

    def is_near_any_endpoint(point, endpoints, threshold):
        return any(dist_points(point, endpoint) < threshold for endpoint in endpoints)

    for member_id, points in list(node_existing.items()):
        node_existing[member_id] = [
            point for point in points
            if not is_near_any_endpoint(point, endpoint_points, yuzhi)
        ]

    return node_existing


def get_real_x_of_jiaodian(coordinates_data, drawing_id, filtered, newx, pj, rod_id, judge_pj_index,value_pj_index):
    """
    计算一类杆件上交点的真实x坐标
    输出：[{"point_2d": (10,20), "x_3d": 123.456}, {...}, ...] 表示每个交点的二维坐标和对应的真实x坐标
    """

    # 得到301杆件的两个端点的二维X坐标
    min_2d_x = coordinates_data[rod_id][0][0]
    max_2d_x = coordinates_data[rod_id][1][0]

    # 得到301 杆件在三维中的两端的 X 坐标
    if (pj[drawing_id - 1][judge_pj_index][1][0] > 0):
        min_3d_x = pj[drawing_id - 1][value_pj_index][1][0]  # 这个是301杆件与塔身相交的端点的三维X坐标
        max_3d_x = newx  # 这个是301杆件尖点的三维X坐标
    else:
        min_3d_x = newx
        max_3d_x = pj[drawing_id - 1][value_pj_index][1][0]

    real = []  # 计算比例并转换为真实x坐标

    for point in filtered:
        # 计算点在二维杆件上的x坐标比例
        x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
        # 根据比例计算真实x坐标
        real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
        real.append({
            "point_2d": point,  # (x2d, y2d)
            "x_3d": real_x  # 对应的真实x
        })

    return real

def get_real_x_of_missing_nodes(
    coordinates_data,
    drawing_id,
    node_existing,
    newx,
    pj,
    rod_id,
    judge_pj_index,
    value_pj_index,
):
    """
    计算三类节点的真实x坐标
    输出：{member_id: [{"point_2d": (x,y), "x_3d": v}, ...], ...}
    """

    def calc_real_points(points):
        # 统一使用 rod_id 对应的二维端点范围
        if rod_id not in coordinates_data:
            return []

        min_2d_x = coordinates_data[rod_id][0][0]
        max_2d_x = coordinates_data[rod_id][1][0]

        # 根据担架方向确定对应的三维 x 取值范围
        if pj[drawing_id - 1][judge_pj_index][1][0] > 0:
            min_3d_x = pj[drawing_id - 1][value_pj_index][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][value_pj_index][1][0]

        real_points = []
        for point in points:
            # 按二维 x 比例映射到三维 x
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_points.append({
                "point_2d": point,
                "x_3d": real_x,
            })
        return real_points

    # 输入为 dict 时，统一按 rod_id 的范围计算
    if isinstance(node_existing, dict):
        real_existing = {}
        for member_id, points in node_existing.items():
            real_existing[member_id] = calc_real_points(points)
        return real_existing

    # 否则按单杆件的点列表计算
    return calc_real_points(node_existing)

def generate_ganjian(coordinates_data, node_101_nodes, node_103_nodes,yuzhi):
    """
    根据节点从二维坐标信息中找到杆件编号
    """
    ganjian_nodes_table_101 = find_ganjian_by_nodes(node_101_nodes, coordinates_data,yuzhi)
    ganjian_nodes_table_103 = find_ganjian_by_nodes(node_103_nodes, coordinates_data,yuzhi)
    all_node_member_map = {}
    all_node_member_map.update(ganjian_nodes_table_101)
    all_node_member_map.update(ganjian_nodes_table_103)
    member_to_nodes = defaultdict(list)
    for node_id, member_list in all_node_member_map.items():
        for member_id in member_list:
            member_to_nodes[member_id].append(node_id)
    return member_to_nodes


def find_missing_members(ganjian, coordinates_data, main_rod_ids):
    """
    找出还未生成的三类杆件的杆件编号ID与其对应的二维坐标数据
    通过对比 ganjian 和 当前视图的坐标数据，找出 ganjian 中缺失的 member_id 及其二维坐标

    参数:
        ganjian: 已生成的杆件列表，每个字典包含 'member_id'
        coordinates_data: 当前视图的坐标数据，键为 member_id，值为两端点的二维坐标数据
        main_rod_ids: 一类杆件的 ID 列表（可选），如包含则从缺失结果中剔除

    返回:
        missing_members: 字典格式，键为缺失的 member_id，值为其对应的二维坐标点列表
        existing_member_ids: 已生成且存在于 coordinates_data 的 member_id 列表（字符串）
    """
    # 收集 ganjian 中已有的 member_id（统一转换为字符串以方便比对）
    existing_members = {str(item.get("member_id")) for item in ganjian if "member_id" in item}
    coordinates_member_ids = {str(m_id) for m_id in (coordinates_data or {}).keys()}
    existing_member_ids = sorted(existing_members & coordinates_member_ids)

    # 对比查找并收集缺失的 member_id 及其二维坐标
    main_rod_id_set = {str(m_id) for m_id in (main_rod_ids or [])}
    missing_members = {}
    if coordinates_data:
        for m_id, points in coordinates_data.items():
            m_id_str = str(m_id)
            if m_id_str not in existing_members and m_id_str not in main_rod_id_set:
                missing_members[m_id_str] = points

    return missing_members, existing_member_ids


def detect_main_rods_for_781(coordinates_data, top_k=2):
    """
    781 的一类杆件编号含下划线，exec 后会变成 1051/1071 这类整数。
    现有检测会因“编号最小”规则回退到短杆；781 只按长度取最长两根。
    """
    rods = []
    for rod_id, points in (coordinates_data or {}).items():
        if len(points) != 2:
            continue
        rods.append((rod_id, dist_points(points[0], points[1])))
    rods.sort(key=lambda item: item[1], reverse=True)
    result = [rod_id for rod_id, _length in rods[:top_k]]
    return sorted(result, key=lambda x: int(x) if str(x).isdigit() else str(x))


def detect_main_rods_by_type(coordinates_data, drawing_type):
    if drawing_type == "781":
        return detect_main_rods_for_781(coordinates_data)
    return detect_main_rods_enhanced(coordinates_data)





jiedian = []
ganjian = []

def trans(file_path, drawing_id, data1, drawing_type, id_offset=False):
    """
    参数:
        file_path: 包含三视图坐标数据的文件路径
        drawing_id: 图纸序号
    """
    # 记录本次担架处理前全局列表的长度，用于只对本担架新增的节点/杆件做后处理
    jiedian_start = len(jiedian)
    ganjian_start = len(ganjian)

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    namespace = {}
    exec(content, namespace)
    coordinatesFront_data = namespace.get('coordinatesFront_data', {})
    coordinatesBottom_data = namespace.get('coordinatesBottom_data', {})
    coordinatesOverhead_data = namespace.get('coordinatesOverhead_data', {})

    yuzhi = 150   #阈值
    data = transform_data(data1) # 将担架和塔身连接点每四个分为一组

    # 将16个端点按照x轴的正负进行分组，【【【担架1的上端点】，【担架1的下端点】】，【【担架2的上端点】，【担架2的下端点】】，【【担架3的上端点】，【担架3的下端点】】，【】，【】...】
    pj = []
    for array in data:
        positive_group = []
        negative_group = []
        for point in array:
            x_coord = point[1][0]
            if x_coord >= 0:
                positive_group.append(point)
            else:
                negative_group.append(point)
        pj.append(positive_group)
        pj.append(negative_group)

    # ===== J1 担架索引修正 =====
    if drawing_type in ("J1", "Z1"):
        # 原 pj 是 8 个担架，这里只取 0,2,4,6 对应的
        pj = [pj[i] for i in (0, 2, 4, 6)]
    elif drawing_type in ("J3", "J4"):
        pj = [pj[i] for i in (1, 0, 3, 2, 5, 4, 7, 6)]
    elif drawing_type in ("T7833", "781"):
        pj = [pj[i] for i in (0,2)]
    elif drawing_type in "7837":
        pj = [pj[i] for i in (0,2)]

    if drawing_type == "7837" and drawing_id == 2 and len(pj) > 1 and len(pj[1]) >= 2:
        pj[1][0], pj[1][1] = pj[1][1], pj[1][0]


    print(pj)
    # 担架编号偏移：担架和塔身合并的图纸（如 T7833/7837，塔身目录下存在 01.txt）担架从 3 号起，
    # 需 +2（1→3, 2→4），保证担架和塔身的节点编号生成不会重复。是否偏移由调用方 work() 传入。
    id_prefix = drawing_id + 2 if id_offset else drawing_id
    jiandian_id = (id_prefix * 100 + 1) * 100

    if drawing_type == "7837":
        jiandian_id = (id_prefix * 100 + 1) * 100 + 70

    # 特例：drawing_type 为 7837 且为 01.txt（drawing_id==1）时，强制走 else 分支（按最上层担架处理）
    if (drawing_type == "7837" and drawing_id == 2) or (
        not (drawing_type == "7837" and drawing_id == 1)
        and (drawing_type in ("T7833", "781") or (drawing_id * 100 + 1 in coordinatesBottom_data))
    ): # 处理下面的担架（非最上面的两个担架）

        rod_front_a, rod_front_b = detect_main_rods_by_type(coordinatesFront_data, drawing_type)
        rod_bottom_a, rod_bottom_b = detect_main_rods_by_type(coordinatesBottom_data, drawing_type)
        rod_overhead_a, rod_overhead_b = detect_main_rods_by_type(coordinatesOverhead_data, drawing_type)

        # T7833 正视图一类杆件方向修正：
        # 本分支约定 rod_front_a 对应 pj 下端点(index1)、rod_front_b 对应 pj 上端点(index0)。
        # 但 T7833 中编号小的(如105)是上杆、编号大的(如107)是下杆，与约定相反，
        # 故对调 a/b，使 section2 的二类节点Y引用与 section5 的一类杆件接续都落到正确的上下端点。
        if drawing_type == "T7833" or (drawing_type == "7837" and drawing_id == 2):
            rod_front_a, rod_front_b = rod_front_b, rod_front_a

        rod_101_id, rod_102_id = rod_bottom_a, rod_bottom_b
        rod_103_id, rod_104_id = rod_overhead_a, rod_overhead_b
        main_rod_ids = [rod_101_id, rod_102_id, rod_103_id, rod_104_id, rod_front_a, rod_front_b]


        ############################################################################################################
        # 1. 计算尖点的三维信息
        ############################################################################################################

        newx, newy, newz = calc_jiandian_xyz(coordinatesBottom_data, drawing_id, pj,1, rod_101_id, rod_102_id)
        new_node = {
            "node_id": f"{jiandian_id + 20}",
            "node_type": 11,  # 根据实际情况设置节点类型
            "symmetry_type": 2,  # 根据实际情况设置对称类型
            "X": round(newx,3),
            "Y": round(newy,3),
            "Z": round(newz,3),
        }
        jiedian.append(new_node)


        ############################################################################################################
        # 2. 正视图
        ############################################################################################################

        # 得到两个一类杆件上的交点
        jiaodian_101,jiaodian_103 = get_jiaodian_on_ganjian(coordinatesFront_data,drawing_id, pj, rod_front_a,rod_front_b, yuzhi)

        # 得到这些交点的真实x的值
        real_101 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_101, newx, pj, rod_front_a, 1, 1)
        real_103 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_103, newx, pj, rod_front_b, 0, 0)

        if (pj[drawing_id - 1][1][1][0] > 0): # 第 drawing_id 号担架下连接点的 x 坐标
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点的节点编号
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点

        # 标记交点中的端点
        real_101 = mark_endpoint_for_real_points(real_101,coordinatesFront_data,rod_front_a,left_3d_id,right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 303 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
        real_103 = mark_endpoint_for_real_points(real_103, coordinatesFront_data, rod_front_b, left_3d_id, right_3d_id,yuzhi)

        # ----------------生成节点---------------------------------------------------------------------------------------#
        # 为交点创建节点
        node_101_nodes = []
        node_103_nodes = []

        new_node_cnt = 0  # 只统计“真正新建的节点”

        # 为101杆件上的交点创建节点
        for i, item in enumerate(real_101):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_101_nodes.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}191{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_101_nodes.append(node_info)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{pj[drawing_id - 1][1][0]}",
                "Z": f"1{jiandian_id + 20}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        # 为103杆件上的交点创建节点
        for i, item in enumerate(real_103):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_103_nodes.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}193{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_103_nodes.append(node_info)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{jiandian_id + 20}"
            })

        # # ---------------生成杆件-----------------------------------------------------------------------------------#

        member_to_nodes = generate_ganjian(coordinatesFront_data, node_101_nodes, node_103_nodes,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 2
                })

        ############################################################################################################
        # 3. 底视图
        ############################################################################################################

        jiaodian_101,jiaodian_102 = get_jiaodian_on_ganjian(coordinatesBottom_data,drawing_id, pj, rod_101_id,rod_102_id, yuzhi)


        real_101 = get_real_x_of_jiaodian(coordinatesBottom_data, drawing_id, jiaodian_101, newx, pj, rod_101_id,1,1)
        real_102 = get_real_x_of_jiaodian(coordinatesBottom_data, drawing_id, jiaodian_102, newx, pj, rod_102_id, 0,0)


        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
        real_101 = mark_endpoint_for_real_points(real_101,coordinatesBottom_data,rod_101_id,left_3d_id,right_3d_id,yuzhi)


        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = str(int(pj[drawing_id - 1][1][0]) + 2)
            right_3d_id = f"{jiandian_id + 22}"
        else:
            left_3d_id = f"{jiandian_id + 22}"
            right_3d_id = str(int(pj[drawing_id - 1][1][0]) + 2)
        real_102 = mark_endpoint_for_real_points(real_102, coordinatesBottom_data, rod_102_id, left_3d_id, right_3d_id,yuzhi)

        # ----------------生成节点---------------------------------------------------------------------------------------#
        node_101_ids = []
        node_102_ids = []

        new_node_cnt = 0  # 只统计“真正新建的节点”

        for i, item in enumerate(real_102):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_101_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}291{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_101_ids.append(node_info)
            duichenzuo = int(pj[drawing_id - 1][1][0]) + 2
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{duichenzuo}",
                "Z": f"1{jiandian_id + 22}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_101):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_102_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}291{new_node_cnt}2"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_102_ids.append(node_info)

        # ----------------生成杆件---------------------------------------------------------------------------------------#

        member_to_nodes = generate_ganjian(coordinatesBottom_data, node_101_ids, node_102_ids,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 0
                })

        #-----------------------------找到三类节点--------------------------------------------------------------------------#

        # 找出底视图中尚未被二类处理覆盖的剩余杆件（三类杆件）及其二维坐标
        missing_members, existing_member_ids = find_missing_members(ganjian, coordinatesBottom_data, main_rod_ids)

        # 计算三类杆件端点落在已有杆件（一类+二类）上的交点二维坐标
        node_existing = get_jiaodian_on_ganjian_by_missing_members(
            coordinatesBottom_data,
            missing_members,
            existing_member_ids,
            drawing_id,
            rod_101_id,
            rod_102_id,
            pj,
            yuzhi,
        )

        # 将交点的二维坐标按比例映射为真实三维X坐标
        real_node_existing = get_real_x_of_missing_nodes(
            coordinatesBottom_data,
            drawing_id,
            node_existing,
            newx,
            pj,
            rod_101_id,
            1,
            1,
            )

        #-----------------------------生成三类节点--------------------------------------------------------------------------#

        # 构建杆件编号->两端节点编号的映射表，用于确定三类节点的Y/Z引用值
        member_endpoints = {}
        for member in ganjian:
            member_id = str(member.get("member_id"))
            if member_id not in member_endpoints:
                member_endpoints[member_id] = (member.get("node1_id"), member.get("node2_id"))
        # 一类杆件的端点需要单独指定（因为前面删除过再重建）
        member_endpoints[str(rod_102_id)] = (
            str(int(pj[drawing_id - 1][1][0]) + 2),
            f"{jiandian_id + 22}",
        )
        member_endpoints[str(rod_101_id)] = (
            str(int(pj[drawing_id - 1][1][0])),
            f"{jiandian_id + 20}",
        )

        new_node_cnt = 0
        real_node_existing_2d_nodes = []

        for member_id, points in (real_node_existing or {}).items():
            # 获取该交点所在杆件的两端节点编号，作为三类节点的Y/Z引用
            endpoints = member_endpoints.get(str(member_id))
            if not endpoints:
                continue
            node1_id, node2_id = endpoints
            for item in points:
                new_node_cnt += 1
                # 编号规则：前9个用 {drawing_id}491X0，超过9个用 {drawing_id}591X0
                if new_node_cnt > 9:
                    node_seq = new_node_cnt - 9
                    node_id = f"{id_prefix}591{node_seq}0"
                else:
                    node_id = f"{id_prefix}491{new_node_cnt}0"
                jiedian.append({
                    "node_id": node_id,
                    "node_type": 12,
                    "symmetry_type": 0,
                    "X": item["x_3d"],
                    "Y": f"1{node1_id}",
                    "Z": f"1{node2_id}",
                })
                # 同时记录二维坐标，供后续生成三类杆件时匹配使用
                real_node_existing_2d_nodes.append({
                    "node_id": node_id,
                    "point_2d": item.get("point_2d"),
                })


        #-----------------------------生成三类杆件--------------------------------------------------------------------------#

        # 通过三类节点的二维坐标反查其属于哪些杆件
        node_to_members = find_ganjian_by_nodes(
            real_node_existing_2d_nodes,
            coordinatesBottom_data,
            yuzhi,
        )
        # 反转映射：杆件编号 -> 该杆件上的节点列表
        member_to_nodes = defaultdict(list)
        for node_id, member_list in node_to_members.items():
            for member_id in member_list:
                member_to_nodes[member_id].append(node_id)

        # 只有恰好两个端点的杆件才是合法的三类杆件
        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 0,
                })
       






        ############################################################################################################
        # 4. 顶视图
        ############################################################################################################

        jiaodian_103,jiaodian_104 = get_jiaodian_on_ganjian(coordinatesOverhead_data,drawing_id, pj, rod_103_id,rod_104_id, yuzhi)

        real_103 = get_real_x_of_jiaodian(coordinatesOverhead_data, drawing_id, jiaodian_103, newx, pj, rod_103_id,1,0)
        real_104 = get_real_x_of_jiaodian(coordinatesOverhead_data, drawing_id, jiaodian_104, newx, pj, rod_104_id, 1,0)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点

        real_103 = mark_endpoint_for_real_points(real_103, coordinatesOverhead_data, rod_103_id, left_3d_id, right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = str(int(pj[drawing_id - 1][0][0]) + 2)
            right_3d_id = f"{jiandian_id + 22}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 22}"
            right_3d_id = str(int(pj[drawing_id - 1][0][0]) + 2)

        real_104 = mark_endpoint_for_real_points(real_104, coordinatesOverhead_data, rod_104_id, left_3d_id, right_3d_id,yuzhi)

        # ----------------生成节点---------------------------------------------------------------------------------------#
        node_103_ids = []
        node_104_ids = []

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_103):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_103_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}391{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_103_ids.append(node_info)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{jiandian_id + 20}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_104):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_104_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}391{new_node_cnt}2"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_104_ids.append(node_info)

        # ----------------生成杆件---------------------------------------------------------------------------------------#
        member_to_nodes = generate_ganjian(coordinatesOverhead_data, node_103_ids, node_104_ids,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 0
                })

        ############################################################################################################
        # 5. 添加一类杆件
        ############################################################################################################

        # 删除已经生成的一类杆件信息
        remove_ids = {
            str(rod_101_id),
            str(rod_102_id),
            str(rod_103_id),
            str(rod_104_id),
            str(rod_front_a),
            str(rod_front_b),
        }

        ganjian[:] = [
            g for g in ganjian
            if g.get("member_id") not in remove_ids
        ]

        new_ganjian = {
            "member_id": f"{rod_front_a}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)

        new_ganjian = {
            "member_id": f"{rod_front_b}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)


    else:

        rod_front_a, rod_front_b = detect_main_rods_by_type(coordinatesFront_data, drawing_type)
        rod_bottom_a, rod_bottom_b = detect_main_rods_by_type(coordinatesBottom_data, drawing_type)
        rod_overhead_a, rod_overhead_b = detect_main_rods_by_type(coordinatesOverhead_data, drawing_type)

        if drawing_type == "7837" and drawing_id == 1:
            rod_front_a, rod_front_b = rod_front_b, rod_front_a

        rod_101_id, rod_102_id = rod_overhead_a, rod_overhead_b
        rod_103_id, rod_104_id = rod_bottom_a, rod_bottom_b
        main_rod_ids = [rod_101_id, rod_102_id, rod_103_id, rod_104_id, rod_front_a, rod_front_b]


        ############################################################################################################
        # 1. 计算尖点的三维信息
        ############################################################################################################

        newx, newy, newz = calc_jiandian_xyz(coordinatesOverhead_data, drawing_id, pj, 0, rod_101_id, rod_102_id)
        new_node = {
            "node_id": f"{jiandian_id + 20}",
            "node_type": 11,  # 根据实际情况设置节点类型
            "symmetry_type": 2,  # 根据实际情况设置对称类型
            "X": round(newx,3),
            "Y": round(newy,3),
            "Z": round(newz,3),
        }
        jiedian.append(new_node)

        ############################################################################################################
        # 2. 正视图
        ############################################################################################################


        jiaodian_101,jiaodian_103 = get_jiaodian_on_ganjian(coordinatesFront_data,drawing_id, pj, rod_front_a,rod_front_b, yuzhi)

        real_101 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_101, newx, pj, rod_front_a, 1, 0)
        real_103 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_103, newx, pj, rod_front_b, 0, 1)

        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 20}"  # 7837/01 正视图尖点在左端收拢点
            right_3d_id = pj[drawing_id - 1][0][0]  # 与塔身相交端点
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点

        real_101 = mark_endpoint_for_real_points(real_101, coordinatesFront_data, rod_front_a, left_3d_id, right_3d_id,yuzhi)

        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 20}"  # 7837/01 正视图尖点在左端收拢点
            right_3d_id = pj[drawing_id - 1][1][0]  # 与塔身相交端点
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点

        real_103 = mark_endpoint_for_real_points(real_103, coordinatesFront_data, rod_front_b, left_3d_id, right_3d_id,yuzhi)

        # ---------------生成节点-----------------------------------------------------------------------------------#
       # 为交点创建节点
        node_101_nodes = []
        node_103_nodes = []

        new_node_cnt = 0  # 只统计“真正新建的节点”
        # 为101杆件上的交点创建节点
        for i, item in enumerate(real_101):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_101_nodes.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}191{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_101_nodes.append(node_info)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{jiandian_id + 20}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        # 为103杆件上的交点创建节点
        for i, item in enumerate(real_103):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_103_nodes.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}193{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_103_nodes.append(node_info)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{pj[drawing_id - 1][1][0]}",
                "Z": f"1{jiandian_id + 20}"
            })

        # ---------------生成杆件-----------------------------------------------------------------------------------#

        member_to_nodes = generate_ganjian(coordinatesFront_data, node_101_nodes, node_103_nodes,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 2
                })

        ############################################################################################################
        # 3. 顶视图
        ############################################################################################################

        jiaodian_101,jiaodian_102 = get_jiaodian_on_ganjian(coordinatesOverhead_data,drawing_id, pj, rod_101_id,rod_102_id, yuzhi)

        real_101 = get_real_x_of_jiaodian(coordinatesOverhead_data, drawing_id, jiaodian_101, newx, pj, rod_101_id,1,1)
        real_102 = get_real_x_of_jiaodian(coordinatesOverhead_data, drawing_id, jiaodian_102, newx, pj, rod_102_id, 1,0)


        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 20}"  # 7837/01 顶视图尖点在左端收拢点
            right_3d_id = pj[drawing_id - 1][0][0]  # 与塔身相交端点
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点

        real_101 = mark_endpoint_for_real_points(real_101, coordinatesOverhead_data, rod_101_id, left_3d_id, right_3d_id,yuzhi)

        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 22}"  # 7837/01 顶视图尖点在左端收拢点
            right_3d_id = str(int(pj[drawing_id - 1][0][0]) + 2)
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = str(int(pj[drawing_id - 1][0][0]) + 2)
            right_3d_id = f"{jiandian_id + 22}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 22}"
            right_3d_id = str(int(pj[drawing_id - 1][0][0]) + 2)

        real_102 = mark_endpoint_for_real_points(real_102, coordinatesOverhead_data, rod_102_id, left_3d_id, right_3d_id,yuzhi)

        # ----------------生成节点---------------------------------------------------------------------------------------#
        node_101_ids = []
        node_102_ids = []

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_102):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_102_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}291{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_102_ids.append(node_info)
            duichenzuo = int(pj[drawing_id - 1][0][0]) + 2
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{duichenzuo}",
                "Z": f"1{jiandian_id + 22}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_101):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_101_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}291{new_node_cnt}2"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_101_ids.append(node_info)

        # ----------------生成杆件---------------------------------------------------------------------------------------#

        member_to_nodes = generate_ganjian(coordinatesOverhead_data, node_101_ids, node_102_ids,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 0
                })

        ############################################################################################################
        # 4. 底视图
        ############################################################################################################

        jiaodian_103,jiaodian_104 = get_jiaodian_on_ganjian(coordinatesBottom_data,drawing_id, pj, rod_103_id,rod_104_id, yuzhi)

        real_103 = get_real_x_of_jiaodian(coordinatesBottom_data, drawing_id, jiaodian_103, newx, pj, rod_103_id,1,0)
        real_104 = get_real_x_of_jiaodian(coordinatesBottom_data, drawing_id, jiaodian_104, newx, pj, rod_104_id, 1,0)

        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 20}"  # 7837/01 底视图尖点在左端收拢点
            right_3d_id = pj[drawing_id - 1][1][0]  # 与塔身相交端点
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
        real_103 = mark_endpoint_for_real_points(real_103,coordinatesBottom_data,rod_103_id,left_3d_id,right_3d_id,yuzhi)

        if drawing_type == "7837" and drawing_id == 1:
            left_3d_id = f"{jiandian_id + 22}"  # 7837/01 底视图尖点在左端收拢点
            right_3d_id = str(int(pj[drawing_id - 1][1][0]) + 2)
        elif (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = str(int(pj[drawing_id - 1][1][0]) + 2)
            right_3d_id = f"{jiandian_id + 22}"
        else:
            left_3d_id = f"{jiandian_id + 22}"
            right_3d_id = str(int(pj[drawing_id - 1][1][0]) + 2)
        real_104 = mark_endpoint_for_real_points(real_104, coordinatesBottom_data, rod_104_id, left_3d_id, right_3d_id,yuzhi)


        # ----------------生成节点---------------------------------------------------------------------------------------#
        node_103_ids = []
        node_104_ids = []

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_103):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_103_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}391{new_node_cnt}0"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_103_ids.append(node_info)
            duichenzuo = int(pj[drawing_id - 1][1][0])
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": item["x_3d"],
                "Y": f"1{duichenzuo}",
                "Z": f"1{jiandian_id + 20}"
            })

        new_node_cnt = 0  # 只统计“真正新建的节点”
        for i, item in enumerate(real_104):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_104_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{id_prefix}391{new_node_cnt}2"
            node_info = {
                "node_id": node_id,
                "point_2d": item["point_2d"]
            }
            node_104_ids.append(node_info)
        # ----------------生成杆件---------------------------------------------------------------------------------------#

        member_to_nodes = generate_ganjian(coordinatesBottom_data, node_103_ids, node_104_ids,yuzhi)

        for member_id, node_list in member_to_nodes.items():
            if len(node_list) == 2:  # 只有两个端点的杆件才是合法的
                ganjian.append({
                    "member_id": str(member_id),
                    "node1_id": node_list[0],
                    "node2_id": node_list[1],
                    "symmetry_type": 0
                })
        #
        #     # -----------------------------找到三类节点--------------------------------------------------------------------------#
        #
        # missing_members, existing_member_ids = find_missing_members(ganjian, coordinatesBottom_data, main_rod_ids)
        #
        # node_existing = get_jiaodian_on_ganjian_by_missing_members(
        #     coordinatesBottom_data,
        #     missing_members,
        #     existing_member_ids,
        #     drawing_id,
        #     rod_103_id,
        #     rod_104_id,
        #     pj,
        #     yuzhi,
        # )
        #
        # real_node_existing = get_real_x_of_missing_nodes(
        #     coordinatesBottom_data,
        #     drawing_id,
        #     node_existing,
        #     newx,
        #     pj,
        #     rod_103_id,
        #     1,
        #     0,
        # )
        #
        # # -----------------------------生成三类节点--------------------------------------------------------------------------#
        #
        # member_endpoints = {}
        # for member in ganjian:
        #     member_id = str(member.get("member_id"))
        #     if member_id not in member_endpoints:
        #         member_endpoints[member_id] = (member.get("node1_id"), member.get("node2_id"))
        # member_endpoints[str(rod_104_id)] = (
        #     str(int(pj[drawing_id - 1][1][0]) + 2),
        #     f"{jiandian_id + 22}",
        # )
        # member_endpoints[str(rod_103_id)] = (
        #     str(int(pj[drawing_id - 1][1][0])),
        #     f"{jiandian_id + 20}",
        # )
        #
        # new_node_cnt = 0
        # real_node_existing_2d_nodes = []
        #
        # for member_id, points in (real_node_existing or {}).items():
        #     endpoints = member_endpoints.get(str(member_id))
        #     if not endpoints:
        #         continue
        #     node1_id, node2_id = endpoints
        #     for item in points:
        #         new_node_cnt += 1
        #         if new_node_cnt > 9:
        #             node_seq = new_node_cnt - 9
        #             node_id = f"{id_prefix}591{node_seq}0"
        #         else:
        #             node_id = f"{id_prefix}491{new_node_cnt}0"
        #         jiedian.append({
        #             "node_id": node_id,
        #             "node_type": 12,
        #             "symmetry_type": 0,
        #             "X": item["x_3d"],
        #             "Y": f"1{node1_id}",
        #             "Z": f"1{node2_id}",
        #         })
        #         real_node_existing_2d_nodes.append({
        #             "node_id": node_id,
        #             "point_2d": item.get("point_2d"),
        #         })
        #
        # # -----------------------------生成三类杆件--------------------------------------------------------------------------#
        #
        # node_to_members = find_ganjian_by_nodes(
        #     real_node_existing_2d_nodes,
        #     coordinatesBottom_data,
        #     yuzhi,
        # )
        # member_to_nodes = defaultdict(list)
        # for node_id, member_list in node_to_members.items():
        #     for member_id in member_list:
        #         member_to_nodes[member_id].append(node_id)
        #
        # for member_id, node_list in member_to_nodes.items():
        #     if len(node_list) == 2:
        #         ganjian.append({
        #             "member_id": str(member_id),
        #             "node1_id": node_list[0],
        #             "node2_id": node_list[1],
        #             "symmetry_type": 0,
        #         })

        ############################################################################################################
        # 5. 添加一类杆件
        ############################################################################################################

        remove_ids = {
            str(rod_101_id),
            str(rod_102_id),
            str(rod_103_id),
            str(rod_104_id),
            str(rod_front_a),
            str(rod_front_b),
        }

        ganjian[:] = [
            g for g in ganjian
            if g.get("member_id") not in remove_ids
        ]

        new_ganjian = {
            "member_id": f"{rod_front_a}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)

        new_ganjian = {
            "member_id": f"{rod_front_b}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)




   #===== J1 担架对称性生成 =====
    if drawing_type in ("J1", "Z1"):
        for g in ganjian:
            if g.get("symmetry_type") == 2:
                g["symmetry_type"] = 4
            elif g.get("symmetry_type") == 0:
                g["symmetry_type"] = 1
        for j in jiedian:
            if j.get("symmetry_type") == 2:
                j["symmetry_type"] = 4

 #===== T7833 担架对称性生成 =====
    if drawing_type in ("T7833", "781") and drawing_id == 2:
        for g in ganjian[ganjian_start:]:
            if g.get("symmetry_type") == 2:
                g["symmetry_type"] = 4
            elif g.get("symmetry_type") == 0:
                g["symmetry_type"] = 1
        for j in jiedian[jiedian_start:]:
            if j.get("symmetry_type") == 2:
                j["symmetry_type"] = 4



#===== 7837 担架对称性生成 =====
    if drawing_type == "7837":
        for g in ganjian:
            if g.get("symmetry_type") == 2:
                g["symmetry_type"] = 4
            elif g.get("symmetry_type") == 0:
                g["symmetry_type"] = 1
        for j in jiedian:
            if j.get("symmetry_type") == 2:
                j["symmetry_type"] = 4

def work(file_path, data, drawing_type, tashen_dir=None):
    txt_count = count_txt_files(file_path)

    # 担架和塔身合并的图纸（塔身目录下存在 01.txt，如 T7833/7837）担架编号需偏移 +2，
    # 避免担架与塔身的节点编号生成重复。
    id_offset = bool(tashen_dir) and os.path.exists(os.path.join(tashen_dir, "01.txt"))

    for i in range(1, txt_count + 1):
        #specific_file_path = f"{file_path}\\0{i}.txt"   # windows
        specific_file_path = f"{file_path}/0{i}.txt"    # mac
        trans(specific_file_path, i, data, drawing_type, id_offset)

    # trans(f"{file_path}\\0{4}.txt", 4, data, drawing_type)


    return jiedian, ganjian
