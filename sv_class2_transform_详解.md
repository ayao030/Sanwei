# sv_class2_transform.py 代码详解

## 目录
1. [代码整体逻辑结构](#代码整体逻辑结构)
2. [第一部分：基础工具函数](#第一部分基础工具函数)
3. [第二部分：一类杆件提取与投影转换](#第二部分一类杆件提取与投影转换)
4. [第三部分：六步拓扑主函数](#第三部分六步拓扑主函数)
5. [第四部分：节点ID生成器](#第四部分节点id生成器)
6. [第五部分：数据结构详解](#第五部分数据结构详解)
7. [流程图总结](#流程图总结)

---

## 代码整体逻辑结构

这个模块的核心目标是：**将2D单视图的塔吊图纸转换成3D坐标数据**。通过"拓扑分类"的方式，将杆件分成三类，逐层处理，最终生成节点（jiedian）和杆件（ganjian）数据。

---

## 第一部分：基础工具函数

### 1. `dist_pt_seg(p, a, b)` - 点到线段距离计算

**作用**：计算点p到线段ab的最短距离（用于容差吸附）

**原理**：
- 把线段ab看作参数方程：`point(t) = a + t*(b-a)`，其中 `t∈[0,1]`
- 求垂足所在的参数 t 值
- 限制 t 在 [0,1] 范围内（确保垂足在线段上）
- 计算点p到该垂足的欧几里得距离

**关键公式**：$t = \frac{(p-a) \cdot (b-a)}{|b-a|^2}$

```python
def dist_pt_seg(p, a, b):
    x0, y0 = p; x1, y1 = a; x2, y2 = b
    dx = x2 - x1; dy = y2 - y1
    l2 = dx*dx + dy*dy
    if l2 == 0: return math.hypot(x0-x1, y0-y1)
    t = max(0, min(1, ((x0-x1)*dx + (x0-y1)*dy) / l2))
    px = x1 + t * dx; py = y1 + t * dy
    return math.hypot(x0-px, y0-py)
```

---

### 2. `clean_id(raw_id)` - ID清理

**作用**：去除CAD解析残留的后缀（如 `"1109_1"` → `"1109"`）

**实现**：用下划线分割，取第一部分

```python
def clean_id(raw_id):
    return str(raw_id).split("_")[0]
```

---

## 第二部分：一类杆件提取与投影转换

### 3. `extract_lines01(lines_dict)` - 提取一类杆件

**筛选条件**：杆件ID末尾必须是 `"01"` 或 `"02"`

**代表意义**：这些是塔的主要竖向结构（主柱）

```python
def extract_lines01(lines_dict):
    target = {}
    for mid, coords in lines_dict.items():
        s = clean_id(mid)
        if s[-2:] in ["01", "02"]:
            target[mid] = coords
    return target
```

---

### 4. `build_projector(lines01)` - 构建投影转换器【核心难点】

这是整个模块最复杂的部分。它要解决这个问题：
**怎样从2D CAD图纸坐标推断3D实际空间坐标？**

#### 投影器的初始化步骤：

**步骤1：识别左右边界主柱**
- 对所有一类杆件计算中心X坐标
- 排序后二分：左半部分和右半部分
- 找左组最左的线和右组最右的线
- 这两条线代表塔的左右边界

**步骤2：计算几何参数**
- `y_min`, `y_max` ← 两条边界线的竖向范围
- `h_cad` ← 图纸高度
- `w_top` ← 塔顶宽度（mm）
- `w_bot` ← 塔底宽度（mm）
- `z_top_3d` ← 塔的倾斜角度计算：$\frac{\sqrt{h^2 - ((w_{bot}-w_{top})/2)^2}}{1000}$
  
  用勾股定理从倾斜的主柱推断高度

**步骤3：返回投影函数**

返回一个闭包函数 `project(x, y)`，将任意2D坐标变换到3D

#### 投影函数的工作原理：

```python
def project(x, y):
    # 返回 (x3d, y3d, z3d)
    
    # 归一化参数：竖向位置比例 [0, 1]
    t = (y - y_min)/h_cad if h_cad > 1e-9 else 0
    
    # 计算Z坐标（高度）：线性增长
    z3d = t * z_top_3d
    
    # 计算X坐标（左右位置）
    xl = get_x(y, left_line)      # 左边界线在当前高度的X
    xr = get_x(y, right_line)     # 右边界线在当前高度的X
    cx = (xl + xr)/2.0            # 中心线
    cw = abs(xr - xl)             # 当前宽度
    rel_x = (x - cx)/cw if cw > 1e-9 else 0  # 相对位置 [-0.5, 0.5]
    
    # mm 转换为 m，考虑沿高度不同处宽度变化
    w_interp = w_top + t*(w_bot - w_top)
    x3d = rel_x * w_interp / 1000.0
    
    return round(x3d,6), round(x3d,6), round(z3d,6)
```

---

## 第三部分：六步拓扑主函数

### `single_view0201(line_coord)` - 主处理函数

这是整个模块的主逻辑。名字"六步"实际上是 **3个阶段，每阶段分析两类**。

#### 📊 拓扑分类体系：

| 分类 | 特征 | 数据类型 | 说明 |
|------|------|---------|------|
| **一类 (Tier 1)** | ID末尾 01 或 02 | node_type = 11 | 一级主柱（真实3D坐标） |
| **二类 (Tier 2)** | 两端都吸附到一类 | node_type = 12 | 连接杆（参考坐标） |
| **三类 (Tier 3)** | 一端吸附一类，另一端吸附二类 | node_type = 12 | 辅助杆（精细参考） |

---

### 🔄 核心处理步骤：

#### **步骤1：初始化**

```python
lines01 = extract_lines01(line_coord)
projector, center_x_cad = build_projector(lines01)
special_bar_id = clean_id(next(iter(lines01.keys())))
  # 选第一根一类杆作为参考基准（用于生成参考坐标）

used_nids = set()
tier2_nodes_map = {}
  # 用于追踪已使用的节点ID和二类杆的端点
```

---

#### **步骤2：处理一类杆件和一类节点**

```python
对每条一类杆件 k:
  ① 为两个端点各生成一个节点
     • node_type = 11（真实坐标）
     • (x3d, y3d, z3d) = projector(pt[0], pt[1])
  
  ② 用两个节点连接成一根杆件
     • member_id = 清理后的杆件ID
     • symmetry_type = 4（正面对称）
```

**结果**：生成塔的主骨架

```python
for k, seg in lines01.items():
    clean_k = clean_id(k)
    node_ids = []
    for i, pt in enumerate(seg):
        nid = get_safe_nid(clean_k)
        x3d, y3d, z3d = projector(pt[0], pt[1])
        jiedian.append({
            "node_id": str(nid), "node_type": 11, "symmetry_type": 4,
            "X": x3d, "Y": y3d, "Z": z3d
        })
        node_ids.append(str(nid))
    
    ganjian.append({
        "member_id": clean_k, 
        "node1_id": node_ids[0], 
        "node2_id": node_ids[1], 
        "symmetry_type": 4
    })
```

---

#### **步骤3-4：处理二类杆件**

```python
对每条未分类的杆件 k:
  ① 检查两个端点
     • h1 = find_host(seg[0], lines01)
     • h2 = find_host(seg[1], lines01)
  
  ② 判断条件：两个端点都吸附到一类杆件（距离 < 35mm）
  
  ③ 是则归为二类处理
```

**关键操作**：
- 为两个端点生成节点（node_type = 12）
- 节点使用参考坐标而非真实坐标：
  - 左侧：`(special_bar_id10, special_bar_id20)`
  - 右侧：`(special_bar_id11, special_bar_id21)`
  
  **设计优势**：只要特定的一类杆调整，所有二类杆都自动跟随（参数化设计）

- 保存真实节点ID到 `tier2_nodes_map`，供后续三类杆引用
- 根据杆件方向生成额外的侧面杆件

```python
clean_k = clean_id(k)
node_ids = []
for i, pt in enumerate(seg):
    nid = get_safe_nid(clean_k)
    _, _, z3d = projector(pt[0], pt[1])
    
    if pt[0] < center_x_cad:
        ref_x, ref_y = f"{special_bar_id}10", f"{special_bar_id}20"
    else:
        ref_x, ref_y = f"{special_bar_id}11", f"{special_bar_id}21"
    
    jiedian.append({
        "node_id": str(nid), "node_type": 12, "symmetry_type": 4,
        "X": ref_x, "Y": ref_y, "Z": z3d
    })
    node_ids.append(str(nid))

# 记录二类杆件真实的 10 和 20 节点
tier2_nodes_map[clean_k] = (node_ids[0], node_ids[1])
```

##### 侧面杆件生成逻辑（关键）：

```python
is_horiz = abs(seg[0][1] - seg[1][1]) < 25.0

if is_horiz:
    # 水平杆 → 生成对称的左右侧杆
    ganjian.append({
        "member_id": clean_k, 
        "node1_id": node_ids[0], 
        "node2_id": f"{node_ids[0][:-1]}1", 
        "symmetry_type": 2  # 左侧侧面
    })
    ganjian.append({
        "member_id": clean_k, 
        "node1_id": node_ids[0], 
        "node2_id": f"{node_ids[0][:-1]}2", 
        "symmetry_type": 1  # 右侧侧面
    })
else:
    # 不水平 → 交叉杆，生成正面和侧面
    ganjian.append({
        "member_id": clean_k, 
        "node1_id": node_ids[0], 
        "node2_id": node_ids[1], 
        "symmetry_type": 4  # 正面
    })
    ganjian.append({
        "member_id": clean_k, 
        "node1_id": node_ids[0], 
        "node2_id": f"{node_ids[1][:-1]}3", 
        "symmetry_type": 4  # 侧面
    })
```

---

#### **步骤5-6：处理三类杆件**

```python
对剩余的未分类杆件 k:
  ① 检查各个端点附着位置：
     • h1_t1 = 指向一类的距离
     • h1_t2 = 指向二类的距离
     • h2_t1 = 指向一类的距离
     • h2_t2 = 指向二类的距离
  
  ② 排除条件：
     两端都吸附到一类的杆 (已在Step 2处理)
  
  ③ 保留条件：
     一端吸附到{一类或二类}，另一端也吸附到{一类或二类}
```

**节点坐标映射方式**：

| 情况 | 端点吸附到 | 参考方式 |
|------|-----------|---------|
| **Case A** | 一类 | 使用通用参考坐标（特定一类杆的端点） |
| **Case B** | 二类 | 使用该二类杆的真实端点ID，形成更紧密的坐标网络 |

```python
for k, seg in list(unclassified.items()):
    h1_t1 = find_host(seg[0], lines01)
    h1_t2 = find_host(seg[0], tier2_members)
    h2_t1 = find_host(seg[1], lines01)
    h2_t2 = find_host(seg[1], tier2_members)
    
    if h1_t1 and h2_t1: continue  # 两头都在一类上，已处理过
    
    if (h1_t1 or h1_t2) and (h2_t1 or h2_t2):
        clean_k = clean_id(k)
        node_ids = []
        
        for i, pt in enumerate(seg):
            nid = get_safe_nid(clean_k)
            x3d, _, z3d = projector(pt[0], pt[1])
            
            h_t1 = h1_t1 if i == 0 else h2_t1
            h_t2 = h1_t2 if i == 0 else h2_t2
            
            if h_t1:
                # Case A: 吸附到一类
                ref_x = f"{special_bar_id}10" if pt[0] < center_x_cad else f"{special_bar_id}11"
                ref_y = f"{special_bar_id}20" if pt[0] < center_x_cad else f"{special_bar_id}21"
                jiedian.append({
                    "node_id": str(nid), "node_type": 12, "symmetry_type": 4,
                    "X": ref_x, "Y": ref_y, "Z": z3d
                })
            elif h_t2:
                # Case B: 吸附到二类
                clean_host = clean_id(h_t2)
                if clean_host in tier2_nodes_map:
                    # 完美引用：真实的干爹端点 ID
                    real_node_1, real_node_2 = tier2_nodes_map[clean_host]
                    sym = 2 if abs(x3d) < 0.01 else 1
                    jiedian.append({
                        "node_id": str(nid), "node_type": 12, "symmetry_type": sym,
                        "X": x3d, "Y": str(real_node_1), "Z": str(real_node_2)
                    })
            
            node_ids.append(str(nid))
        
        if len(node_ids) == 2:
            ganjian.append({
                "member_id": clean_k, 
                "node1_id": node_ids[0], 
                "node2_id": node_ids[1], 
                "symmetry_type": 4
            })
```

**对称类型判断**：
```python
if |x3d| < 0.01:
    sym = 2  # 内侧（中心线）
else:
    sym = 1  # 外侧
```

---

## 第四部分：节点ID生成器

### `get_safe_nid(base_id)` - 防冲突的ID分配

```python
def get_safe_nid(base_id):
    """保证每根杆都独占自己的一套节点 ID"""
    
    for suffix in range(10, 100, 10):
        test_nid = f"{base_id}{suffix}"
        if test_nid not in used_nids:
            used_nids.add(test_nid)
            return test_nid
    return f"{base_id}99"
```

**工作原理**：

- 尝试序列：`base_id + [10, 20, 30, ..., 90]`
- 如果全部碰撞：返回 `base_id + "99"`

**示例**：杆件 `"1109"` 生成
- 第一条杆：`1109 + 10 = "110910"`
- 第二条杆：`1109 + 20 = "110920"`
- 第三条杆：`1109 + 30 = "110930"`
- ...

**目的**：避免节点ID重复，每根杆在其ID空间内自留10级别的子ID

---

## 第五部分：数据结构详解

### 节点结构（jiedian）

```python
{
  "node_id": "110910",              # 唯一标识
  "node_type": 11 或 12,             # 11=真实3D坐标，12=参考坐标
  "symmetry_type": 1, 2, 4,          # 1=右侧, 2=左侧, 4=正面
  "X": 0.5 或 "110910",              # 数值（真实坐标）或参考ID
  "Y": -0.3 或 "110920",             # 数值（真实坐标）或参考ID
  "Z": 1.2                           # 通常是数值（高度）
}
```

**node_type 字段说明**：
- `11`：真实3D坐标，X/Y/Z 都是具体数值
- `12`：参考坐标，X/Y/Z 中至少一个是其他节点的ID引用

**symmetry_type 字段说明**：
- `1`：右侧对称
- `2`：左侧对称
- `4`：正面对称

---

### 杆件结构（ganjian）

```python
{
  "member_id": "1109",               # 归属的杆件ID
  "node1_id": "110910",              # 起点节点ID
  "node2_id": "110920",              # 终点节点ID
  "symmetry_type": 4                 # 对称关系
}
```

---

## 流程图总结

```
输入：line_coord (所有2D线段及ID)
  ↓
[提取一类]  →  extract_lines01()
  ↓
[构建投影]  →  build_projector()  【关键转换】
  ↓
[处理一类]  →  生成11型节点，存储main ganjian
  ↓
[处理二类]  →  两端都吸附一类 
             ↓ 生成12型参考节点
             ↓ 保存端点ID到 tier2_nodes_map
             ↓ 根据方向生成侧面杆件
  ↓
[处理三类]  →  混合吸附
             ↓ 选择参考方式（通用参考 或 真实端点）
             ↓ 根据吸附对象判断对称类型
  ↓
输出：(ganjian[], jiedian[])
```

---

## 设计理念总结

1. **分层拓扑法**：从主结构逐层细化，自动形成参数化关系
2. **容差吸附**：点到线段的距离判断（容差35mm），实现自动关联
3. **参考坐标**：使用node_type=12实现参数化，一级和二级之间形成刚性约束
4. **ID空间隔离**：每根杆保有独立的10级别ID子空间，避免冲突
5. **对称性保留**：symmetry_type字段记录结构关系，供下游处理使用
