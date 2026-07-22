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
def _numeric_value(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _get_numeric_xyz(node):
    x = _numeric_value(node.get("X"))
    y = _numeric_value(node.get("Y"))
    z = _numeric_value(node.get("Z"))
    if x is None or y is None or z is None:
        return None
    return x, y, z


def _pick_interface_layer(nodes, mode):
    numeric_nodes = []
    for node in nodes or []:
        xyz = _get_numeric_xyz(node)
        if xyz is not None:
            numeric_nodes.append((node, xyz))

    if not numeric_nodes:
        return None

    if mode == "top":
        key_node, (_, _, z_ref) = max(numeric_nodes, key=lambda item: item[1][2])
    else:
        key_node, (_, _, z_ref) = min(numeric_nodes, key=lambda item: item[1][2])

    z_tol = 0.02
    layer = [
        (node, xyz)
        for node, xyz in numeric_nodes
        if abs(xyz[2] - z_ref) <= z_tol
    ]
    if not layer:
        layer = [(key_node, _get_numeric_xyz(key_node))]

    xs = [xyz[0] for _, xyz in layer]
    ys = [xyz[1] for _, xyz in layer]
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0

    if max(xs) - min(xs) > 1e-9:
        half_width = (max(xs) - min(xs)) / 2.0
    else:
        # single_view01 keeps one symmetric main leg only; abs(X) is the
        # half-width represented by that reference leg.
        half_width = max(abs(x) for x in xs)

    return {
        "node": key_node,
        "center_x": center_x,
        "center_y": center_y,
        "z": z_ref,
        "half_width": half_width,
    }


def _apply_single_view_transform(nodes, xy_scale, x_shift, y_shift, z_shift):
    for node in nodes:
        x_value = _numeric_value(node.get("X"))
        if x_value is not None:
            node["X"] = round(x_value * xy_scale + x_shift, 6)

        y_value = _numeric_value(node.get("Y"))
        if y_value is not None:
            node["Y"] = round(y_value * xy_scale + y_shift, 6)

        z_value = _numeric_value(node.get("Z"))
        if z_value is not None:
            node["Z"] = round(z_value + z_shift, 6)


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

    def _remap_token(value, alias_prefix_map, alias_node_map):
        if not isinstance(value, str) or len(value) < 2:
            return value

        if value in alias_node_map:
            return alias_node_map[value]

        if value.startswith("1") and value[1:] in alias_node_map:
            return f"1{alias_node_map[value[1:]]}"

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

    node_lookup = {
        str(node.get("node_id", "")): node
        for node in lines0201_jiedian
    }

    def _resolved_xyz(node_id, resolving=None):
        """Resolve an in-sheet type-12 node before selecting a seam alias."""
        node_id = str(node_id)
        resolving = set() if resolving is None else resolving
        if node_id in resolving:
            return None

        node = node_lookup.get(node_id)
        if node is None:
            return None

        values = [node.get(key) for key in ("X", "Y", "Z")]
        ref_indexes = [index for index, value in enumerate(values) if isinstance(value, str)]
        if not ref_indexes:
            try:
                return tuple(float(value) for value in values)
            except (TypeError, ValueError):
                return None
        if len(ref_indexes) != 2:
            return None

        real_index = next(index for index in range(3) if index not in ref_indexes)
        try:
            real_value = float(values[real_index])
        except (TypeError, ValueError):
            return None

        point_a = _resolved_xyz(values[ref_indexes[0]], resolving | {node_id})
        point_b = _resolved_xyz(values[ref_indexes[1]], resolving | {node_id})
        if point_a is None or point_b is None:
            return None

        span = point_b[real_index] - point_a[real_index]
        if abs(span) < 1e-9:
            return None
        ratio = (real_value - point_a[real_index]) / span
        return tuple(
            real_value if index == real_index
            else point_a[index] + ratio * (point_b[index] - point_a[index])
            for index in range(3)
        )

    def _matching_symmetry_id(source_node, target_node):
        """Return the target-family node whose symmetry image matches source."""
        target_id = str(target_node["node_id"])
        try:
            sx, sy = float(source_node["X"]), float(source_node["Y"])
        except (TypeError, ValueError):
            resolved = _resolved_xyz(source_node.get("node_id", ""))
            if resolved is None:
                return target_id
            sx, sy = resolved[0], resolved[1]
        try:
            tx, ty = float(target_node["X"]), float(target_node["Y"])
        except (TypeError, ValueError):
            return target_id

        candidates = {
            "0": (tx, ty),
            "1": (-tx, ty),
            "2": (tx, -ty),
            "3": (-tx, -ty),
        }
        suffix = min(
            candidates,
            key=lambda key: (sx - candidates[key][0]) ** 2 + (sy - candidates[key][1]) ** 2,
        )
        return f"{target_id[:-1]}{suffix}"

    def _replace_member_endpoint(old_node_id, new_node_id):
        for member in lines0201_ganjian:
            if str(member["node1_id"]) == old_node_id:
                member["node1_id"] = new_node_id
            if str(member["node2_id"]) == old_node_id:
                member["node2_id"] = new_node_id

    def _member_base(member_id):
        text = str(member_id)
        if "_" in text:
            text = text.split("_", 1)[0]
        return text

    min_z_node = min(lines01_jiedian, key=lambda x: x["Z"])
    min_id = str(min_z_node["node_id"])
    min_z = min_z_node["Z"]

    max_z_node = max(lines01_jiedian, key=lambda x: x["Z"])
    max_id = str(max_z_node["node_id"])
    max_z = max_z_node["Z"]

    alias_prefix_map = {}
    alias_node_map = {}
    merged_node_ids = set()
    protected_prefixes = set()
    node_id_set = {str(node.get("node_id", "")) for node in lines0201_jiedian}

    # Protect rods that are themselves class-1 trunks (..01 / ..02) and
    # selected tier2 X-members from being collapsed onto the single global
    # min/max reference pair. Those members should keep their own generated
    # endpoint nodes; only their internal type12 references are expected to
    # point at the host trunk family.
    protected_member_suffixes = {"01", "02", "05", "06", "09", "10"}
    for member in lines0201_ganjian:
        member_base = _member_base(member.get("member_id", ""))
        if len(member_base) >= 2 and member_base[-2:] in protected_member_suffixes:
            protected_prefixes.add(str(member.get("node1_id", ""))[:-1])
            protected_prefixes.add(str(member.get("node2_id", ""))[:-1])

    for node in lines0201_jiedian:
        for coord_key in ("X", "Y", "Z"):
            ref_id = node.get(coord_key)
            if not isinstance(ref_id, str):
                continue
            if ref_id in node_id_set:
                protected_prefixes.add(ref_id[:-1])
            elif ref_id.startswith("1") and ref_id[1:] in node_id_set:
                protected_prefixes.add(ref_id[1:-1])

    for node in lines0201_jiedian:
        temp_id = str(node["node_id"])
        temp_prefix = temp_id[:-1]

        # Side-face nodes are physical rotations.  Sharing a height with a
        # main-leg endpoint does not make them the same node.
        if node.get("_view_face") == "side":
            continue

        # Type-12 endpoints already lie on their host member by reference.
        # Replacing them solely because they share a support-end height can
        # collapse distinct left/right attachments into one node family.
        if node.get("node_type") == 12:
            continue

        if temp_prefix in protected_prefixes:
            continue

        if _z_equal(node["Z"], min_z):
            target_id = _matching_symmetry_id(node, min_z_node)
            alias_node_map[temp_id] = target_id
            if target_id[-1] == temp_id[-1]:
                alias_prefix_map[temp_prefix] = min_id[:-1]
            merged_node_ids.add(temp_id)
            _replace_member_endpoint(temp_id, target_id)

        elif _z_equal(node["Z"], max_z):
            target_id = _matching_symmetry_id(node, max_z_node)
            alias_node_map[temp_id] = target_id
            if target_id[-1] == temp_id[-1]:
                alias_prefix_map[temp_prefix] = max_id[:-1]
            merged_node_ids.add(temp_id)
            _replace_member_endpoint(temp_id, target_id)

    if alias_prefix_map or alias_node_map:
        for member in lines0201_ganjian:
            member["node1_id"] = _remap_token(
                str(member["node1_id"]), alias_prefix_map, alias_node_map
            )
            member["node2_id"] = _remap_token(
                str(member["node2_id"]), alias_prefix_map, alias_node_map
            )

        for node in lines0201_jiedian:
            for key in ("X", "Y", "Z"):
                node[key] = _remap_token(node.get(key), alias_prefix_map, alias_node_map)

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
    
    prev_interface = _pick_interface_layer(prev_jiedian01, "top")
    if prev_interface is None:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian

    prev_max_node = prev_interface["node"]
    prev_key = str(prev_max_node["node_id"])
    
    # 找到当前图中一类节点的Z最小值节点
    if not lines01_jiedian:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
    
    now_interface = _pick_interface_layer(lines01_jiedian, "bottom")
    if now_interface is None:
        return lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian

    now_min_node = now_interface["node"]
    now_key = str(now_min_node["node_id"])
    
    # Keep tier1 main-leg families (..01/..02) local to the current sheet.
    # They act as the current file's concrete output and should not be renamed
    # onto the previous sheet's reference IDs during inner stitching.
    protected_prefixes = set()
    for member in lines0201_ganjian:
        member_id = str(member.get("member_id", "")).split("_")[0]
        if not member_id.endswith(("01", "02")):
            continue
        for node_key in ("node1_id", "node2_id"):
            node_id = member.get(node_key)
            if isinstance(node_id, str) and len(node_id) >= 1:
                protected_prefixes.add(node_id[:-1])

    # Align the current sheet to the previous sheet by the interface half-width
    # and center. Z keeps its own scale and is only shifted onto the seam.
    if abs(now_interface["half_width"]) < 1e-9:
        xy_scale = 1.0
    else:
        xy_scale = abs(prev_interface["half_width"]) / abs(now_interface["half_width"])

    x_shift = prev_interface["center_x"] - now_interface["center_x"] * xy_scale
    y_shift = prev_interface["center_y"] - now_interface["center_y"] * xy_scale
    z_shift = prev_interface["z"] - now_interface["z"]
    
    # Apply the same sheet transform to both the reference skeleton and the
    # exported nodes. String references are kept as node references.
    _apply_single_view_transform(lines01_jiedian, xy_scale, x_shift, y_shift, z_shift)
    
    # 删除一类节点中与now_key相同的节点（避免重复）
    lines01_jiedian = [node for node in lines01_jiedian if node["node_id"] != now_key]
    
    _apply_single_view_transform(lines0201_jiedian, xy_scale, x_shift, y_shift, z_shift)
    
    # 处理二类节点的X和Y（这里是字符串ID，需要替换）
    for node in lines0201_jiedian:
        x_val = node.get("X")
        y_val = node.get("Y")
        if (
            isinstance(x_val, str)
            and len(x_val) >= 1
            and len(now_key) >= 1
            and x_val[:-1] == now_key[:-1]
            and x_val[:-1] not in protected_prefixes
        ):
            node["X"] = f"{prev_key[:-1]}{x_val[-1]}"
        if (
            isinstance(y_val, str)
            and len(y_val) >= 1
            and len(now_key) >= 1
            and y_val[:-1] == now_key[:-1]
            and y_val[:-1] not in protected_prefixes
        ):
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
        if (
            len(member["node1_id"]) >= 1
            and len(now_key) >= 1
            and member["node1_id"][:-1] == now_key[:-1]
            and member["node1_id"][:-1] not in protected_prefixes
        ):
            member["node1_id"] = f"{prev_key[:-1]}{member['node1_id'][-1]}"
        # 处理node2_id：同理
        if (
            len(member["node2_id"]) >= 1
            and len(now_key) >= 1
            and member["node2_id"][:-1] == now_key[:-1]
            and member["node2_id"][:-1] not in protected_prefixes
        ):
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
    prev_jiedian01 = []     # 仅保存上一张图的一类节点，按图纸编号链式拼接

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
        lines0201_ganjian, lines0201_jiedian = t21.single_view0201(
            line_coord,
            front_only=False,
            keep_view_face=True,
        )
        
        # 进行一二类间修正
        lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian = correct_single_lines(
            lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
        )
        # lines01_* is only the reference skeleton used for correction/alignment.
        # Exported single-view results should come from lines0201_* only.
        # 单正面间第一张图纸：直接添加到结果中
        if not prev_jiedian01:
            res_ganjian.extend(lines0201_ganjian)

            res_jiedian.extend(lines0201_jiedian)
        
        # 单正面其他图纸：先进行拼接变换，再添加到结果中
        else:
            lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian = connect_single_inner(
                prev_jiedian01, lines01_ganjian, lines01_jiedian, lines0201_ganjian, lines0201_jiedian
            )

            res_ganjian.extend(lines0201_ganjian)

            res_jiedian.extend(lines0201_jiedian)

        # 只保留当前图，下一张图只和上一张图对接，避免跨图号回连
        prev_jiedian01 = list(lines01_jiedian)
    
    # 节点格式修正
    res_jiedian = correct_format_jiedian(res_jiedian)
    for node in res_jiedian:
        node.pop("_view_face", None)
    
    return res_ganjian, res_jiedian
