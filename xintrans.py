import math
import json
from collections import defaultdict

from get_first_ganjian_id import detect_main_rods_enhanced


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
    计算两条直线的交点

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
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def transform_data(input_data):
    """
    将输入的节点数据按每4个一组进行分组

    参数:
        input_data: 输入的节点数据列表

    返回:
        分组后的数据
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

    参数:
        points: 点列表，每个点为 (x, y) 格式
        threshold: 聚类阈值，默认10.0

    返回:
        聚类后的点列表
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
    判断这个交点是不是这个杆件上的两个端点中的其中一个，如果是的话，用端点的节点编号标记它
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

def calc_jiandian_xyz(coordinates_data, drawing_id, pj,pj_view_index):
    """
    生成尖点的真实XYZ的值
    """
    start_01x = coordinates_data[drawing_id * 100 + 1][0][0]
    start_01y = coordinates_data[drawing_id * 100 + 1][0][1]
    start_02x = coordinates_data[drawing_id * 100 + 2][0][0]
    start_02y = coordinates_data[drawing_id * 100 + 2][0][1]
    end_01x = coordinates_data[drawing_id * 100 + 1][1][0]
    end_01y = coordinates_data[drawing_id * 100 + 1][1][1]
    end_02x = coordinates_data[drawing_id * 100 + 2][1][0]
    end_02y = coordinates_data[drawing_id * 100 + 2][1][1]
    midr = ((end_01x + end_02x) / 2, (end_01y + end_02y) / 2)  # 计算301和302两个右端点的中点
    midl = ((start_01x + start_02x) / 2, (start_01y + start_02y) / 2)  # 计算301和302两个左端点的中点
    h = dist_points(midl, midr)  # 计算左中点到右中点的距离
    p_01l = (start_01x, start_01y)
    a1 = dist_points(p_01l, midl)  # 计算301左端点到左中点的距离
    shiji = abs(pj[drawing_id - 1][pj_view_index][1][1])  # 计算担架3与塔身相交的下端点的三维x坐标的值，也就是实际值
    bili = shiji / a1  # 计算实际值/像素值的比例
    p_01r = (end_01x, end_01y)
    a2 = dist_points(p_01r, midr)
    # 生成尖点
    if (pj[drawing_id - 1][pj_view_index][1][0] > 0):  # if里面生成右边担架的尖点
        newx = pj[drawing_id - 1][pj_view_index][1][0] + h * bili
        newy = -a2 * bili
        newz = pj[drawing_id - 1][pj_view_index][1][2]
    else:  # else生成左边担架的尖点
        newx = pj[drawing_id - 1][pj_view_index][1][0] - h * bili
        newy = -a2 * bili
        newz = pj[drawing_id - 1][pj_view_index][1][2]
    return newx, newy, newz

def get_jiaodian_on_ganjian(coordinates_data, drawing_id, pj, rod_101_id, rod_103_id, yuzhi):
    """
    从一类杆件上找到交点
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

def get_real_x_of_jiaodian(coordinates_data, drawing_id, filtered_101, newx, pj, rod_101_id, judge_pj_index,value_pj_index):
    """
    计算交点的真实x值
    """
    # 得到301杆件的两个端点的二维X坐标
    min_2d_x = coordinates_data[rod_101_id][0][0]
    max_2d_x = coordinates_data[rod_101_id][1][0]
    # 得到301 杆件在三维中的两端的 X 坐标
    if (pj[drawing_id - 1][judge_pj_index][1][0] > 0):
        min_3d_x = pj[drawing_id - 1][value_pj_index][1][0]  # 这个是301杆件与塔身相交的端点的三维X坐标
        max_3d_x = newx  # 这个是301杆件尖点的三维X坐标
    else:
        min_3d_x = newx
        max_3d_x = pj[drawing_id - 1][value_pj_index][1][0]
    # 计算比例并转换为真实x坐标
    real_101 = []
    for point in filtered_101:
        # 计算点在二维杆件上的x坐标比例
        x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
        # 根据比例计算真实x坐标
        real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
        real_101.append({
            "point_2d": point,  # (x2d, y2d)
            "x_3d": real_x  # 对应的真实x
        })
    return real_101

def generate_ganjian(coordinatesFront_data, node_101_nodes, node_103_nodes,yuzhi):
    """
    根据节点从二维坐标信息中找到杆件编号
    """
    ganjian_nodes_table_101 = find_ganjian_by_nodes(node_101_nodes, coordinatesFront_data,yuzhi)
    ganjian_nodes_table_103 = find_ganjian_by_nodes(node_103_nodes, coordinatesFront_data,yuzhi)
    all_node_member_map = {}
    all_node_member_map.update(ganjian_nodes_table_101)
    all_node_member_map.update(ganjian_nodes_table_103)
    member_to_nodes = defaultdict(list)
    for node_id, member_list in all_node_member_map.items():
        for member_id in member_list:
            member_to_nodes[member_id].append(node_id)
    return member_to_nodes





jiedian = []
ganjian = []

def trans(file_path, drawing_id, data1, drawing_type):
    """
    参数:
        file_path: 包含三视图坐标数据的文件路径
        drawing_id: 图纸序号
        l1, l2: 对称轴附近的杆件编号
    """

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    namespace = {}
    exec(content, namespace)
    coordinatesFront_data = namespace.get('coordinatesFront_data', {})
    coordinatesBottom_data = namespace.get('coordinatesBottom_data', {})
    coordinatesOverhead_data = namespace.get('coordinatesOverhead_data', {})



    yuzhi = 150   #阈值
    data = transform_data(data1)

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
    if drawing_type == "J1":
        # 原 pj 是 8 个担架，这里只取 0,2,4,6 对应的
        pj = [pj[i] for i in (0, 2, 4, 6)]


    jiandian_id = (drawing_id * 100 + 1) * 100

    if(drawing_id * 100 + 1 in coordinatesBottom_data):

        ############################################################################################################
        # 1. 计算尖点的三维信息
        ############################################################################################################

        newx, newy, newz = calc_jiandian_xyz(coordinatesBottom_data, drawing_id, pj,1)
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

        rod_101_id, rod_103_id = detect_main_rods_enhanced(coordinatesFront_data)
        rod_101_id, rod_102_id = detect_main_rods_enhanced(coordinatesBottom_data)
        rod_103_id, rod_104_id = detect_main_rods_enhanced(coordinatesOverhead_data)

        # 得到两个一类杆件上的交点
        jiaodian_101,jiaodian_103 = get_jiaodian_on_ganjian(coordinatesFront_data,drawing_id, pj, rod_101_id,rod_103_id, yuzhi)

        # 得到这些交点的真实x的值
        real_101 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_101, newx, pj, rod_101_id, 1, 1)
        real_103 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_103, newx, pj, rod_103_id, 0, 0)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点

        # 标记节点中的端点
        real_101 = mark_endpoint_for_real_points(real_101,coordinatesFront_data,rod_101_id,left_3d_id,right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 303 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
        real_103 = mark_endpoint_for_real_points(real_103, coordinatesFront_data, rod_103_id, left_3d_id, right_3d_id,yuzhi)

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
            node_id = f"{drawing_id}191{new_node_cnt}0"
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
            node_id = f"{drawing_id}193{new_node_cnt}0"
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

        for i, item in enumerate(real_101):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_101_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{drawing_id}291{new_node_cnt}0"
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
        for i, item in enumerate(real_102):
            if item.get("endpoint_3d_id", -1) != -1:
                # 复用已有端点节点
                node_102_ids.append({
                    "node_id": item["endpoint_3d_id"],
                    "point_2d": item["point_2d"]
                })
                continue

            new_node_cnt += 1
            node_id = f"{drawing_id}291{new_node_cnt}2"
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
            node_id = f"{drawing_id}391{new_node_cnt}0"
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
            node_id = f"{drawing_id}391{new_node_cnt}2"
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
            str(rod_104_id)
        }

        ganjian[:] = [
            g for g in ganjian
            if g.get("member_id") not in remove_ids
        ]

        new_ganjian = {
            "member_id": f"{rod_101_id}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)

        new_ganjian = {
            "member_id": f"{rod_103_id}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)


    else:

        ############################################################################################################
        # 1. 计算尖点的三维信息
        ############################################################################################################

        newx, newy, newz = calc_jiandian_xyz(coordinatesOverhead_data, drawing_id, pj, 0)
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


        rod_101_id, rod_103_id = detect_main_rods_enhanced(coordinatesFront_data)
        rod_103_id, rod_104_id = detect_main_rods_enhanced(coordinatesBottom_data)
        rod_101_id, rod_102_id = detect_main_rods_enhanced(coordinatesOverhead_data)


        jiaodian_101,jiaodian_103 = get_jiaodian_on_ganjian(coordinatesFront_data,drawing_id, pj, rod_101_id,rod_103_id, yuzhi)

        real_101 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_101, newx, pj, rod_101_id, 1, 0)
        real_103 = get_real_x_of_jiaodian(coordinatesFront_data, drawing_id, jiaodian_103, newx, pj, rod_103_id, 0, 1)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点

        real_101 = mark_endpoint_for_real_points(real_101, coordinatesFront_data, rod_101_id, left_3d_id, right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点

        real_103 = mark_endpoint_for_real_points(real_103, coordinatesFront_data, rod_103_id, left_3d_id, right_3d_id,yuzhi)

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
            node_id = f"{drawing_id}191{new_node_cnt}0"
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
            node_id = f"{drawing_id}193{new_node_cnt}0"
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


        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][0][0]  # 301 与塔身相交端点

        real_101 = mark_endpoint_for_real_points(real_101, coordinatesOverhead_data, rod_101_id, left_3d_id, right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
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
            node_id = f"{drawing_id}291{new_node_cnt}0"
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
            node_id = f"{drawing_id}291{new_node_cnt}2"
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

        if (pj[drawing_id - 1][1][1][0] > 0):
            left_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
            right_3d_id = f"{jiandian_id + 20}"  # 尖点
        else:
            left_3d_id = f"{jiandian_id + 20}"
            right_3d_id = pj[drawing_id - 1][1][0]  # 301 与塔身相交端点
        real_103 = mark_endpoint_for_real_points(real_103,coordinatesBottom_data,rod_103_id,left_3d_id,right_3d_id,yuzhi)

        if (pj[drawing_id - 1][1][1][0] > 0):
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
            node_id = f"{drawing_id}391{new_node_cnt}0"
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
            node_id = f"{drawing_id}391{new_node_cnt}2"
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


        ############################################################################################################
        # 5. 添加一类杆件
        ############################################################################################################

        remove_ids = {
            str(rod_101_id),
            str(rod_102_id),
            str(rod_103_id),
            str(rod_104_id)
        }

        ganjian[:] = [
            g for g in ganjian
            if g.get("member_id") not in remove_ids
        ]

        new_ganjian = {
            "member_id": f"{rod_101_id}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)

        new_ganjian = {
            "member_id": f"{rod_103_id}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{jiandian_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)

   #===== J1 担架对称性生成 =====
    if drawing_type == "J1":
        for g in ganjian:
            if g.get("symmetry_type") == 2:
                g["symmetry_type"] = 4
            elif g.get("symmetry_type") == 0:
                g["symmetry_type"] = 1
        for j in jiedian:
            if j.get("symmetry_type") == 2:
                j["symmetry_type"] = 4

def work(file_path, data, drawing_type):
    if drawing_type=="1E2-SDJ":
        for i in range(1,9):
            specific_file_path = f"{file_path}\\0{i}.txt"
            trans(specific_file_path, i, data,drawing_type)
    elif drawing_type == "J1":
        for i in range(1, 5):
            specific_file_path = f"{file_path}\\0{i}.txt"
            trans(specific_file_path, i, data,drawing_type)

    return jiedian, ganjian
