import math


# ===============================
# 1. 线段长度计算
# ===============================

def segment_length(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


# ===============================
# 2. 增强版一类杆件检测（长度 + 编号验证）
# ===============================

def detect_main_rods_enhanced(coordinates_data, top_k=2):
    """
    智能检测一类杆件：
    1. 先按长度选最长的 top_k 根
    2. 检查这些杆件编号是否包含最小编号或就是最小两个
    3. 如果不满足，则回退到编号最小的两个杆件
    """
    if len(coordinates_data) < 2:
        return []

    # 计算所有杆件的长度，并收集编号
    rod_items = []
    all_ids = []

    for rod_id in coordinates_data.keys():
        # 统一处理编号（字符串或数字）
        try:
            num_id = int(rod_id)
        except (ValueError, TypeError):
            num_id = float('inf')  # 非数字编号排最后
        all_ids.append((rod_id, num_id))

        p1, p2 = coordinates_data[rod_id]
        length = segment_length(p1, p2)
        rod_items.append((rod_id, num_id, length))

    # 按长度降序排序，取出最长的 top_k 根
    rod_items.sort(key=lambda x: x[2], reverse=True)
    candidates = [item[0] for item in rod_items[:top_k]]  # 候选杆件ID（字符串）

    # 获取所有杆件中编号最小的两个（按数字值排序）
    all_ids.sort(key=lambda x: x[1])
    min_two_ids = [item[0] for item in all_ids[:2]]

    # 转换为集合方便判断包含关系
    candidates_set = set(candidates)
    min_two_set = set(min_two_ids)
    min_one = min_two_ids[0]  # 最小的那个编号

    # 判断规则
    if candidates_set == min_two_set:
        # 候选正好是最小两个 → 完美，直接返回
        result = min_two_ids
    elif min_one in candidates_set:
        # 候选中包含最小的那个编号 → 也认为合理（常见于一类杆件包含最小编号）
        result = sorted(candidates, key=lambda x: int(x) if str(x).isdigit() else x)
    else:
        # 候选两个编号都较大（如813、814），不包含最小编号 → 回退到最小两个
       # print(f"警告：长度选出的 {candidates} 不包含最小编号，回退到编号最小的 {min_two_ids}")
        result = min_two_ids

    # 最终返回按编号升序排序的结果
    return sorted(result, key=lambda x: int(x) if str(x).isdigit() else x)


# ===============================
# 主程序（替换原函数即可）
# ===============================

if __name__ == "__main__":
    #dir_path = r"D:\Sanwei\zuobiao\DanJia\1E2-SDJ"  # 保留目录路径
    dir_path = r"D:\Sanwei\zuobiao\DanJia\J1"
    for i in range(1, 5):
        file_path = f"{dir_path}\\0{i}.txt"  # 每次生成新的完整路径
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        namespace = {}
        exec(content, namespace)

        coordinatesFront_data = namespace.get('coordinatesFront_data', {})
        coordinatesBottom_data = namespace.get('coordinatesBottom_data', {})
        coordinatesOverhead_data = namespace.get('coordinatesOverhead_data', {})

        print(f"文件: 0{i}.txt")

        print("正视图一类杆件:", detect_main_rods_enhanced(coordinatesFront_data))
        print("底视图一类杆件:", detect_main_rods_enhanced(coordinatesBottom_data))
        print("顶视图一类杆件:", detect_main_rods_enhanced(coordinatesOverhead_data))

        print("----------------------------------------------")