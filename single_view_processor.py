"""
单正面坐标转换处理器

本模块实现单正面视图的三维坐标转换功能，包括：
1. 调用一类和二类杆件转换模块 (sv_class1_transform, sv_class2_transform)
2. 一二类杆件间的修正逻辑
3. 单正面内部多图幅的拼接逻辑
4. 节点格式修正

主函数：single_view(filelist, filepath)
"""

import os
import io_utils as rw
import sv_class1_transform as t1
import sv_class2_transform as t21


# # =============== 一二类间修正相关子函数 ===============
# def correct_single_lines(lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian):
#     """
#     修正一类和二类杆件之间的关系
    
#     主要逻辑：
#     1. 找到一类杆件节点（lines01_jiedian）中Z值最小和最大的节点
#     2. 在二类杆件节点（lines0201_jiedian）中查找与最小/最大Z值相同的节点
#     3. 删除这些重复节点，并将其node_id映射更新到二类杆件的连接关系中
    
#     参数：
#         lines01_ganjian: 一类杆件列表
#         lines01_jiedian: 一类节点列表
#         lines0201_ganjian: 二类杆件列表
#         lines0201_jiedian: 二类节点列表
    
#     返回：
#         修正后的 (lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian)
#     """
#     # 如果一类节点为空，直接返回
#     if not lines01_jiedian:
#         return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
    
#     def _z_equal(z1, z2, tol=0.02):
#         try:
#             return abs(float(z1) - float(z2)) < tol
#         except (TypeError, ValueError):
#             return z1 == z2

#     # 找到一类节点中Z值最小的节点，记录其node_id为min_id，Z为min_z
#     min_z_node = min(lines01_jiedian, key=lambda x: x["Z"])
#     min_id = min_z_node["node_id"]
#     min_z = min_z_node["Z"]
    
#     # 找到一类节点中Z值最大的节点，记录其node_id为max_id，Z为max_z
#     max_z_node = max(lines01_jiedian, key=lambda x: x["Z"])
#     max_id = max_z_node["node_id"]
#     max_z = max_z_node["Z"]
    
#     # 遍历二类节点列表
#     i = 0
#     while i < len(lines0201_jiedian):
#         node = lines0201_jiedian[i]
        
#         # 如果二类节点的Z值等于一类最小Z值
#         if _z_equal(node["Z"], min_z):
#             # 记录该节点的node_id、X、Y值
#             temp_id = node["node_id"]
#             temp_X = node["X"]
#             temp_Y = node["Y"]
            
#             # 删除该二类节点（因为已经在一类中存在）
#             del lines0201_jiedian[i]
            
#             # 遍历二类杆件列表，更新引用该节点的杆件连接关系
#             for member in lines0201_ganjian:
#                 # 处理node1_id：如果除最后一位外与temp_id相同
#                 n1_str = str(member["node1_id"])
#                 temp_id_str = str(temp_id)
#                 if len(n1_str) >= 1 and len(temp_id_str) >= 1 and n1_str[:-1] == temp_id_str[:-1]:
#                     # 替换为temp_X，保留原ID最后一位
#                     new_id = f"{temp_X[:-1]}{n1_str[-1]}"
#                     member["node1_id"] = new_id
                
#                 # 处理node2_id：同理
#                 n2_str = str(member["node2_id"])
#                 if len(n2_str) >= 1 and len(temp_id_str) >= 1 and n2_str[:-1] == temp_id_str[:-1]:
#                     new_id = f"{temp_X[:-1]}{n2_str[-1]}"
#                     member["node2_id"] = new_id
        
#         # 如果二类节点的Z值等于一类最大Z值
#         elif _z_equal(node["Z"], max_z):
#             # 记录该节点的node_id、X、Y值
#             temp_id = node["node_id"]
#             temp_X = node["X"]
#             temp_Y = node["Y"]
            
#             # 删除该二类节点（因为已经在一类中存在）
#             del lines0201_jiedian[i]
            
#             # 遍历二类杆件列表，更新引用该节点的杆件连接关系
#             for member in lines0201_ganjian:
#                 # 处理node1_id：如果除最后一位外与temp_id相同
#                 n1_str = str(member["node1_id"])
#                 temp_id_str = str(temp_id)
#                 if len(n1_str) >= 1 and len(temp_id_str) >= 1 and n1_str[:-1] == temp_id_str[:-1]:
#                     # 替换为temp_Y，保留原ID最后一位
#                     new_id = f"{temp_Y[:-1]}{n1_str[-1]}"
#                     member["node1_id"] = new_id
                
#                 # 处理node2_id：根据倒数第二位的值进行不同的处理
#                 n2_str = str(member["node2_id"])
#                 if len(n2_str) >= 1 and len(temp_id_str) >= 1 and n2_str[:-1] == temp_id_str[:-1]:
#                     if len(n2_str) >= 2 and n2_str[-2] == '1':
#                         # 倒数第二位为1：替换为temp_Y，保留原ID最后一位
#                         new_id = f"{temp_Y[:-1]}{n2_str[-1]}"
#                         member["node2_id"] = new_id
#                     elif len(n2_str) >= 2 and n2_str[-2] == '2':
#                         # 倒数第二位为2：最后一位改为 (原最后一位/3 + 1)
#                         last_digit = int(n2_str[-1])
#                         new_last = int(last_digit / 3 + 1)
#                         new_id = f"{temp_Y[:-1]}{new_last}"
#                         member["node2_id"] = new_id
        
#         else:
#             i += 1  # 只有在未删除元素时才递增索引
    
#     return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
def correct_single_lines(lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian):
    """
    修正一类和二类杆件之间的关系（安全遍历版）
    """
    if not lines01_jiedian:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
    
    def _z_equal(z1, z2, tol=0.02):
        try:
            return abs(float(z1) - float(z2)) < tol
        except (TypeError, ValueError):
            return z1 == z2

    min_z_node = min(lines01_jiedian, key=lambda x: x["Z"])
    min_id = str(min_z_node["node_id"])
    min_z = min_z_node["Z"]
    
    max_z_node = max(lines01_jiedian, key=lambda x: x["Z"])
    max_id = str(max_z_node["node_id"])
    max_z = max_z_node["Z"]
    
    # 🚀 既然不再删除元素，直接用 for 循环遍历，彻底告别死循环！
    for node in lines0201_jiedian:
        
        if _z_equal(node["Z"], min_z):
            temp_id = str(node["node_id"])
            for member in lines0201_ganjian:
                n1_str = str(member["node1_id"])
                if len(n1_str) >= 1 and len(temp_id) >= 1 and n1_str[:-1] == temp_id[:-1]:
                    member["node1_id"] = f"{min_id[:-1]}{n1_str[-1]}"
                n2_str = str(member["node2_id"])
                if len(n2_str) >= 1 and len(temp_id) >= 1 and n2_str[:-1] == temp_id[:-1]:
                    member["node2_id"] = f"{min_id[:-1]}{n2_str[-1]}"
        
        elif _z_equal(node["Z"], max_z):
            temp_id = str(node["node_id"])
            for member in lines0201_ganjian:
                n1_str = str(member["node1_id"])
                if len(n1_str) >= 1 and len(temp_id) >= 1 and n1_str[:-1] == temp_id[:-1]:
                    member["node1_id"] = f"{max_id[:-1]}{n1_str[-1]}"
                n2_str = str(member["node2_id"])
                if len(n2_str) >= 1 and len(temp_id) >= 1 and n2_str[:-1] == temp_id[:-1]:
                    if len(n2_str) >= 2 and n2_str[-2] == '1':
                        member["node2_id"] = f"{max_id[:-1]}{n2_str[-1]}"
                    elif len(n2_str) >= 2 and n2_str[-2] == '2':
                        # 保持原有的整除逻辑
                        member["node2_id"] = f"{max_id[:-1]}{int(int(n2_str[-1]) / 3 + 1)}"
                        
    return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
# =============== 杆件拼接相关子函数 ===============
def correct_single_lines(lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian):
    """
    修正一类和二类杆件之间的关系。

    当二类节点与一类主腿节点在高度上重合时：
    1. 将相关杆件端点并到一类节点上；
    2. 记录旧二类节点到保留节点的别名关系；
    3. 用别名关系统一修正当前图幅内所有坐标引用；
    4. 删除已经并入主腿的重复二类节点。
    """
    if not lines01_jiedian:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian

    def _z_equal(z1, z2, tol=0.02):
        try:
            return abs(float(z1) - float(z2)) < tol
        except (TypeError, ValueError):
            return z1 == z2

    def _remap_token(value, alias_prefix_map):
        if not isinstance(value, str) or len(value) < 2:
            return value

        prefix = value[:-1]
        suffix = value[-1]
        if prefix in alias_prefix_map:
            return f"{alias_prefix_map[prefix]}{suffix}"

        if value.startswith("1") and len(value) >= 3:
            raw_value = value[1:]
            raw_prefix = raw_value[:-1]
            raw_suffix = raw_value[-1]
            if raw_prefix in alias_prefix_map:
                return f"1{alias_prefix_map[raw_prefix]}{raw_suffix}"

        return value

    min_z_node = min(lines01_jiedian, key=lambda x: x["Z"])
    min_id = str(min_z_node["node_id"])
    min_z = min_z_node["Z"]

    max_z_node = max(lines01_jiedian, key=lambda x: x["Z"])
    max_id = str(max_z_node["node_id"])
    max_z = max_z_node["Z"]

    alias_prefix_map = {}
    merged_node_ids = set()

    for node in lines0201_jiedian:
        temp_id = str(node["node_id"])
        temp_prefix = temp_id[:-1]

        if _z_equal(node["Z"], min_z):
            alias_prefix_map[temp_prefix] = min_id[:-1]
            merged_node_ids.add(temp_id)
            for member in lines0201_ganjian:
                n1_str = str(member["node1_id"])
                if len(n1_str) >= 1 and n1_str[:-1] == temp_prefix:
                    member["node1_id"] = f"{min_id[:-1]}{n1_str[-1]}"
                n2_str = str(member["node2_id"])
                if len(n2_str) >= 1 and n2_str[:-1] == temp_prefix:
                    member["node2_id"] = f"{min_id[:-1]}{n2_str[-1]}"

        elif _z_equal(node["Z"], max_z):
            alias_prefix_map[temp_prefix] = max_id[:-1]
            merged_node_ids.add(temp_id)
            for member in lines0201_ganjian:
                n1_str = str(member["node1_id"])
                if len(n1_str) >= 1 and n1_str[:-1] == temp_prefix:
                    member["node1_id"] = f"{max_id[:-1]}{n1_str[-1]}"
                n2_str = str(member["node2_id"])
                if len(n2_str) >= 1 and n2_str[:-1] == temp_prefix:
                    if len(n2_str) >= 2 and n2_str[-2] == '1':
                        member["node2_id"] = f"{max_id[:-1]}{n2_str[-1]}"
                    elif len(n2_str) >= 2 and n2_str[-2] == '2':
                        member["node2_id"] = f"{max_id[:-1]}{int(int(n2_str[-1]) / 3 + 1)}"

    if alias_prefix_map:
        for member in lines0201_ganjian:
            member["node1_id"] = _remap_token(str(member["node1_id"]), alias_prefix_map)
            member["node2_id"] = _remap_token(str(member["node2_id"]), alias_prefix_map)

        for node in lines0201_jiedian:
            for key in ("X", "Y", "Z"):
                node[key] = _remap_token(node.get(key), alias_prefix_map)

        lines0201_jiedian = [
            node for node in lines0201_jiedian
            if str(node.get("node_id")) not in merged_node_ids
        ]

    return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian


def connect_single_inner(prev_jiedian01, lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian):
    """
    单正面内部多图幅拼接函数
    
    主要逻辑：
    1. 找到前一张图中一类节点的Z最大值节点作为基准
    2. 找到当前图中一类节点的Z最小值节点作为对齐点
    3. 计算缩放比例k和Z轴平移量b
    4. 对当前图的所有节点和杆件进行缩放和平移变换
    5. 更新节点ID映射关系
    
    参数：
        prev_jiedian01: 前一张图的一类节点列表
        lines01_ganjian: 当前图的一类杆件列表
        lines01_jiedian: 当前图的一类节点列表
        lines0201_ganjian: 当前图的二类杆件列表
        lines0201_jiedian: 当前图的二类节点列表
    
    返回：
        变换后的 (lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian)
    """
    # 找到前一张图中一类节点的Z最大值节点
    if not prev_jiedian01:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
    
    prev_max_node = max(prev_jiedian01, key=lambda x: x["Z"])
    prev_key = prev_max_node["node_id"]
    prev_x = prev_max_node["X"]
    prev_z = prev_max_node["Z"]
    
    # 找到当前图中一类节点的Z最小值节点
    if not lines01_jiedian:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
    
    now_min_node = min(lines01_jiedian, key=lambda x: x["Z"])
    now_key = now_min_node["node_id"]
    now_x = now_min_node["X"]
    now_z = now_min_node["Z"]
    
    # 计算缩放比例k和Z轴平移量b
    # 处理now_x为0的情况，避免除零错误
    if abs(now_x) < 1e-9:
        k = 1.0
    else:
        k = prev_x / now_x
    
    b = prev_z - now_z
    
    # 处理一类节点：缩放X、Y，平移Z
    for node in lines01_jiedian:
        node["X"] = round(node["X"] * k, 3)
        node["Y"] = round(node["Y"] * k, 3)
        node["Z"] = round(node["Z"] + b, 3)
    
    # 删除一类节点中与now_key相同的节点（避免重复）
    lines01_jiedian = [node for node in lines01_jiedian if node["node_id"] != now_key]
    
    # 处理二类节点：仅对数值型Z进行平移，引用型字符串保持不动
    for node in lines0201_jiedian:
        z_value = node.get("Z")
        if isinstance(z_value, (int, float)):
            node["Z"] = round(z_value + b, 3)
    
    # 处理二类节点的X和Y（这里是字符串ID，需要替换）
    for node in lines0201_jiedian:
        x_val = node.get("X")
        y_val = node.get("Y")
        if isinstance(x_val, str) and len(x_val) >= 1 and len(now_key) >= 1 and x_val[:-1] == now_key[:-1]:
            node["X"] = f"{prev_key[:-1]}{x_val[-1]}"
        if isinstance(y_val, str) and len(y_val) >= 1 and len(now_key) >= 1 and y_val[:-1] == now_key[:-1]:
            node["Y"] = f"{prev_key[:-1]}{y_val[-1]}"
    
    # 处理一类杆件的node1_id和node2_id
    for member in lines01_ganjian:
        # 处理node1_id：如果除最后一位外与now_key相同，则替换为prev_key
        if len(member["node1_id"]) >= 1 and len(now_key) >= 1 and member["node1_id"][:-1] == now_key[:-1]:
            member["node1_id"] = f"{prev_key[:-1]}{member['node1_id'][-1]}"
        # 处理node2_id：同理
        if len(member["node2_id"]) >= 1 and len(now_key) >= 1 and member["node2_id"][:-1] == now_key[:-1]:
            member["node2_id"] = f"{prev_key[:-1]}{member['node2_id'][-1]}"
    
    # 处理二类杆件的node1_id和node2_id
    for member in lines0201_ganjian:
        # 处理node1_id：如果除最后一位外与now_key相同，则替换为prev_key
        if len(member["node1_id"]) >= 1 and len(now_key) >= 1 and member["node1_id"][:-1] == now_key[:-1]:
            member["node1_id"] = f"{prev_key[:-1]}{member['node1_id'][-1]}"
        # 处理node2_id：同理
        if len(member["node2_id"]) >= 1 and len(now_key) >= 1 and member["node2_id"][:-1] == now_key[:-1]:
            member["node2_id"] = f"{prev_key[:-1]}{member['node2_id'][-1]}"
    
    return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian


# =============== 节点格式修正相关子函数 ===============
def correct_format_jiedian(res_jiedian):
    """仅对字符串坐标补齐前缀'1'，保持数值类型不变。"""
    target_keys = {'X', 'Y', 'Z'}
    for node in res_jiedian:
        for key in target_keys:
            value = node.get(key)
            if isinstance(value, str):
                node[key] = '1' + value
    return res_jiedian


# =============== 单正面转换总函数 ===============
def single_view(filelist, filepath):
    """
    单正面坐标转换主函数
    
    功能：
    1. 对每张图幅分别调用一类和二类杆件转换函数
    2. 进行一二类间修正
    3. 进行图幅间拼接
    4. 进行节点格式修正
    
    参数：
        filelist: 文件编号列表，例如 [1, 2, 3]
        filepath: 文件所在目录路径
    
    返回：
        (res_ganjian, res_jiedian): 杆件列表和节点列表
    
    流程：
        - 第一张图：直接将结果添加到res中
        - 后续图幅：先进行拼接变换，再添加到res中
        - 最后对所有节点进行格式修正
    """
    # 初始化结果列表
    res_ganjian = []        # 杆件结果列表
    res_jiedian = []        # 节点结果列表
    prev_jiedian01 = []     # 存储前一张图的一类杆件节点

    for file_num in filelist:
        # 支持两位数格式：先尝试两位数（07.txt），再尝试一位数（7.txt）
        file_path = None
        
        # 确保 file_num 是整数类型
        if isinstance(file_num, int):
            # 尝试两种格式：07.txt, 7.txt
            for fmt in [f'{file_num:02d}.txt', f'{file_num}.txt']:
                test_path = f'{filepath}/{fmt}'
                if os.path.exists(test_path):
                    file_path = test_path
                    break
        else:
            # 如果是字符串，直接使用
            test_path = f'{filepath}/{file_num}.txt'
            if os.path.exists(test_path):
                file_path = test_path
        
        # 文件读取
        if file_path is None:
            if isinstance(file_num, int):
                print(f"警告：文件 {file_num:02d}.txt 或 {file_num}.txt 不存在，已跳过\n")
            else:
                print(f"警告：文件 {file_num}.txt 不存在，已跳过\n")
            continue
            
        try:
            line_coord = rw.read_coords(file_path)
        except Exception as e:
            print(f"读取文件 {os.path.basename(file_path)} 时出错：{str(e)}\n")
            continue

        if not isinstance(line_coord, dict) or not line_coord:
            print(f"警告：文件 {os.path.basename(file_path)} 未找到正面数据，已跳过\n")
            continue

        # 调用一类和二类杆件转换函数
        lines01_ganjian, lines01_jiedian = t1.single_view01(line_coord)
        lines0201_ganjian, lines0201_jiedian = t21.single_view0201(line_coord)
        
        # 进行一二类间修正
        lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian = correct_single_lines(
            lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
        )

        # 单正面间第一张图纸：直接添加到结果中
        if not prev_jiedian01:
            res_ganjian.extend(lines01_ganjian)
            res_ganjian.extend(lines0201_ganjian)

            res_jiedian.extend(lines01_jiedian)
            res_jiedian.extend(lines0201_jiedian)
        
        # 单正面其他图纸：先进行拼接变换，再添加到结果中
        else:
            lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian = connect_single_inner(
                prev_jiedian01, lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
            )

            res_ganjian.extend(lines01_ganjian)
            res_ganjian.extend(lines0201_ganjian)

            res_jiedian.extend(lines01_jiedian)
            res_jiedian.extend(lines0201_jiedian)

        # 保存当前一类节点，用于下一张图的拼接
        prev_jiedian01.extend(lines01_jiedian)
    
    # 节点格式修正
    res_jiedian = correct_format_jiedian(res_jiedian)
    
    return res_ganjian, res_jiedian
