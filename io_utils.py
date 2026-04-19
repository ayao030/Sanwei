import json
import re


def read_coords(file_path):
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # 仅提取 coordinatesFront_data 内部的字典部分
    block_match = re.search(
        r"coordinatesFront_data\s*=\s*\{(?P<body>.*?)\}",
        content,
        re.IGNORECASE | re.DOTALL,
    )
    if not block_match:
        print("解析数据时出错: 未找到 coordinatesFront_data 块")
        return None

    coord_str = "{" + block_match.group("body") + "}"

    # 处理键，添加引号以保留原始格式（包括下划线）
    # 匹配类似 "1907_1:" 这样的键
    pattern = r"(\d+_\d+):|(\d+):"
    processed_str = re.sub(pattern, lambda m: f"'{m.group(0)[:-1]}':", coord_str)

    # 解析处理后的字符串
    try:
        original_data = eval(processed_str)
    except Exception as e:
        print(f"解析数据时出错: {e}")
        return None

    # 转换数据格式
    line_coord = {}
    for key, value in original_data.items():
        line_coord[key] = [(x, y) for x, y in value]

    return line_coord

def write_data_to_txt(ganjian, jiedian, filename="data.txt"):
    """
    将杆件和节点数据格式化写入TXT文件
    
    参数:
        ganjian: 杆件数据列表
        jiedian: 节点数据列表
        filename: 输出文件名
    """
    with open(filename, 'w', encoding='utf-8') as f:
        # 写入杆件数据
        f.write("杆件数据:\n")
        # 使用json.dumps进行格式化，indent=4表示缩进4个空格
        f.write(json.dumps(ganjian, ensure_ascii=False, indent=4))
        f.write("\n\n")  # 空行分隔
        
        # 写入节点数据
        f.write("节点数据:\n")
        f.write(json.dumps(jiedian, ensure_ascii=False, indent=4))
        f.write("\n")