import xml.etree.ElementTree as ET
import os

def extract_professional(input_file, output_file, seed_road_ids):
    print(f"正在加载原始文件: {input_file} ...")
    try:
        tree = ET.parse(input_file)
        root = tree.getroot()
    except Exception as e:
        print(f"文件解析失败: {e}")
        return

    # 1. 建立全图索引
    all_roads = {r.get('id'): r for r in root.findall('road')}
    all_junctions = {j.get('id'): j for j in root.findall('junction')}
    
    # 2. 智能确定提取范围 (Expansion Logic)
    #    规则：从用户指定的种子ID出发，如果该路属于某个 Junction，则必须提取该 Junction 下的所有路
    extraction_road_ids = set(str(x) for x in seed_road_ids)
    involved_junction_ids = set()

    # 第一轮扫描：找出涉及的路口
    for rid in list(extraction_road_ids):
        if rid in all_roads:
            road = all_roads[rid]
            jid = road.get('junction')
            if jid and jid != "-1":
                involved_junction_ids.add(jid)
    
    # 第二轮扫描：将涉及路口下的所有路全部加入提取列表
    print(f"检测到涉及的路口 ID: {involved_junction_ids}")
    for jid in involved_junction_ids:
        if jid in all_junctions:
            junction = all_junctions[jid]
            # 遍历该 junction 下的所有 connection，找出相关的 connectingRoad
            for connection in junction.findall('connection'):
                con_road = connection.get('connectingRoad')
                if con_road:
                    extraction_road_ids.add(con_road)
    
    print(f"根据路口完整性原则，道路提取数量修正为: {len(extraction_road_ids)} 条")

    # 3. 构建新的 OpenDRIVE 结构
    new_root = ET.Element('OpenDRIVE')
    
    # --- 关键修复：Header ---
    # 直接复制原始 Header 节点，但在写入时需要特殊处理 CDATA
    header = root.find('header')
    if header is not None:
        new_root.append(header)
    
    # 4. 提取道路并执行“边界熔断”
    count_roads = 0
    for rid in extraction_road_ids:
        if rid not in all_roads:
            continue
            
        road = all_roads[rid]
        
        # 检查并清理 Link (Predecessor / Successor)
        # 如果连接指向了 extraction_road_ids 以外的路，必须删除这个引用，否则 Esmini 会报错
        link = road.find('link')
        if link is not None:
            for tag in ['predecessor', 'successor']:
                item = link.find(tag)
                if item is not None and item.get('elementType') == 'road':
                    target_id = item.get('elementId')
                    if target_id not in extraction_road_ids:
                        # 这是一个通向外部的连接 -> 熔断它
                        # print(f"  [边界熔断] Road {rid} -> {target_id} ({tag})")
                        link.remove(item)
        
        new_root.append(road)
        count_roads += 1

    # 5. 提取路口节点
    count_junctions = 0
    for jid in involved_junction_ids:
        if jid in all_junctions:
            new_root.append(all_junctions[jid])
            count_junctions += 1

    # 6. 保存文件 (特殊处理 GeoReference CDATA)
    # 默认的 write 会转义 < 和 >，导致 <![CDATA[...]]> 变成 &lt;...&gt;，这是 Esmini 报错的主因
    # 我们先生成字符串，然后手动修复 Header
    
    # 粗略生成的 XML 字符串
    rough_string = ET.tostring(new_root, encoding='utf-8').decode('utf-8')
    
    # 原始文件中的 GeoReference 文本 (通常包含 CDATA)
    original_geo_ref = ""
    orig_geo_elem = root.find('.//geoReference')
    if orig_geo_elem is not None:
         # ElementTree 读取时会剥离 CDATA 标签，我们需要手动加回去
         # 或者更简单：直接用字符串替换修复
         pass

    # 由于 Python XML 库对 CDATA 支持不佳，我们采用“后处理”方式修复 Header
    # 1. 找到生成的 <geoReference>...</geoReference>
    # 2. 替换为标准的 CDATA 格式
    
    import re
    # 匹配生成的 geoReference 内容 (可能被转义了)
    # 比如 <geoReference>+proj=tmerc...</geoReference>
    # 我们要把它变成 <geoReference><![CDATA[+proj=tmerc...]]></geoReference>
    
    # 获取原始投影字符串
    proj_str = "+proj=tmerc +lon_0=121.2092870660126 +lat_0=31.292829882838856 +ellps=WGS84"
    if orig_geo_elem is not None and orig_geo_elem.text:
        proj_str = orig_geo_elem.text.strip()

    # 替换
    # 注意：生成的 XML 可能包含 <?xml ...?> 头，我们需要加上
    final_xml_str = '<?xml version="1.0" standalone="yes"?>\n' + rough_string
    
    # 修复 GeoReference: 简单暴力的字符串替换，确保格式绝对正确
    # 找到 <geoReference> 标签的位置
    pattern = re.compile(r'<geoReference>.*?</geoReference>', re.DOTALL)
    cdata_geo = f"<geoReference><![CDATA[{proj_str}]]></geoReference>"
    final_xml_str = re.sub(pattern, cdata_geo, final_xml_str)
    
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(final_xml_str)

    print("-" * 30)
    print(f"提取完成: {output_file}")
    print(f"包含道路: {count_roads} 条, 路口: {count_junctions} 个")
    print("已修复: 边界连接关系 (防止 Missing connecting road)")
    print("已修复: GeoReference 格式 (防止 Unsupported geo reference)")
    print("-" * 30)

# --- 配置区 ---
input_filename = "tjtest_sdxgproj_20251014143055A001_3.txt" 
output_filename = "roundabout_fixed.xodr"

# 你的目标 ID (包含你提到的以及分析出来的相关 ID)
# 这里的列表作为“种子”，脚本会自动把它们所属路口的其他路也吸附进来
target_seeds = [
    161, 162, 163, 164,   # 环岛核心
    93, 94, 95, 96, 97,   # 左侧连接
    99, 100, 190,         # 右侧连接
    109, 91, 107, 179, 195, 106 # 之前报错提到的路，建议加上作为种子
]

if __name__ == "__main__":
    extract_professional(input_filename, output_filename, target_seeds)