import xintrans
import tower_body_reconstruction
import pandas as pd
import os

def normalize_node_ids(nodes, members):
    # 步骤1：收集所有现有节点标识（去掉最后一位）
    used_ids = set()
    for node in nodes:
        node_id = node['node_id']
        prefix = node_id[:-1]
        used_ids.add(prefix)
    
    # 步骤2：找出所有需要修改的节点
    nodes_to_modify = []
    id_mapping = {}  # 旧标识 -> 新标识的映射
    
    # 找出不在1-500范围内或重复的标识
    for node in nodes:
        node_id = node['node_id']
        prefix = node_id[:-1]
        suffix = node_id[-1]
        
        # 检查是否是数字且在1-500范围内
        try:
            num = int(prefix)
            if 1 <= num <= 500 and prefix not in used_ids:
                used_ids.add(prefix)  # 标记为已使用
                continue
        except ValueError:
            pass
        
        # 需要修改的节点
        nodes_to_modify.append((node, prefix, suffix))
    
    # 步骤3：生成新的标识（1-500范围内未使用的）
    available_ids = set(str(i) for i in range(1, 501)) - used_ids
    available_ids = sorted(available_ids, key=lambda x: int(x))
    
    # 步骤4：创建映射关系
    for node, old_prefix, suffix in nodes_to_modify:
        if old_prefix in id_mapping:
            new_prefix = id_mapping[old_prefix]
        else:
            if not available_ids:
                raise ValueError("1-500范围内的可用ID已用完")
            new_prefix = available_ids.pop(0)
            id_mapping[old_prefix] = new_prefix
        
        # 更新节点ID
        node['node_id'] = new_prefix + suffix
    
    # 步骤5：更新杆件列表中的节点引用
    for member in members:
        for attr in ['node1_id', 'node2_id']:
            old_id = member[attr]
            prefix = old_id[:-1]
            suffix = old_id[-1]
            
            if prefix in id_mapping:
                member[attr] = id_mapping[prefix] + suffix
    
    # 步骤6：更新node_type=12的节点中的引用
    for node in nodes:
        if node['node_type'] == 12:
            for coord in ['X', 'Y','Z']:
                ref_id = node[coord]
                if isinstance(ref_id, str) and ref_id.startswith('1'):
                    # 去掉开头的1得到原始ID
                    original_id = ref_id[1:]
                    prefix = original_id[:-1]
                    suffix = original_id[-1]
                    
                    if prefix in id_mapping:
                        len1=len(id_mapping[prefix])+1
                        node[coord] = '1' + '0'*(4-len1)+id_mapping[prefix] + suffix
    
    return nodes, members, id_mapping


def create_excel_pandas(nodes, members, filename='structure_data.xlsx'):
    """
    使用pandas生成Excel文件（备选方案）
    """
    try:
        # 创建节点DataFrame
        nodes_df = pd.DataFrame(nodes)
        nodes_df = nodes_df[['node_id', 'symmetry_type', 'X', 'Y', 'Z']]
        nodes_df.columns = ['节点编号', '对称性', 'X坐标', 'Y坐标', 'Z坐标']
        
        # 创建杆件DataFrame
        members_df = pd.DataFrame(members)
        members_df = members_df[['member_id','node1_id', 'node2_id', 'symmetry_type']]
        members_df.columns = ['杆件编号','起点节点', '终点节点', '对称性']
        # members_df = members_df[['member_id','node1_id', 'node2_id', 'specifications','symmetry_type']]
        # members_df.columns = ['杆件编号','起点节点', '终点节点', '规格','对称性']

        # 写入Excel文件
        with pd.ExcelWriter(filename, engine='openpyxl') as writer:
            nodes_df.to_excel(writer, sheet_name='节点信息', index=False)
            members_df.to_excel(writer, sheet_name='杆件信息', index=False)
        
        print(f"Excel文件已生成: {filename}")
        
    except Exception as e:
        print(f"生成Excel文件时出错: {e}")

# 保留三位小数
def format_xyz_coordinates(data):
    for item in data:
        for key in ['X', 'Y', 'Z']:
            if key in item and isinstance(item[key], (int, float)):
                item[key] = round(item[key], 3)
    return data

def tran2dto3d(danjia_dir,tashen_dir,project_path,savepath_ui,drawing_type):

    # 添加杆件的规格，另外记得在上面杆件代码中添加
    # from v7 import ImageProcessor
    # numandtableDetector = ImageProcessor()
    # specs = {}
    # for root, dirs, files in os.walk(r"D:\Sanwei\table\1E2-SDJ"):
    #     for file in files:
    #         file_path = os.path.join(root, file)
    #         specs.update(numandtableDetector.extract_table_specs(file_path))


    ganjian_tashen, jiedian_tashen, pinjie_tashen = tower_body_reconstruction.build_tower_body(tashen_dir)
    jiedian_danjia,ganjian_danjia= xintrans.work(danjia_dir, pinjie_tashen,drawing_type)
    jiedian=jiedian_danjia+jiedian_tashen
    jiedian = format_xyz_coordinates(jiedian)
    savepath=os.path.join(project_path, "3d_result")
    if not os.path.exists(savepath):
        os.makedirs(savepath)
    ganjian=ganjian_danjia+ganjian_tashen

    # for item in ganjian:
    #     item['specifications'] = specs[item['member_id']]

    create_excel_pandas(jiedian, ganjian, os.path.join(savepath, 'chushi_data.xlsx'))
    create_excel_pandas(jiedian, ganjian, os.path.join(savepath_ui, 'chushi_data.xlsx'))
    jiedian, ganjian, mapping = normalize_node_ids(jiedian, ganjian)
    create_excel_pandas(jiedian, ganjian, os.path.join(savepath, 'zhenghe_data.xlsx'))
    create_excel_pandas(jiedian, ganjian, os.path.join(savepath_ui, 'zhenghe_data.xlsx'))

