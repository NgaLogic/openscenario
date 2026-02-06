import xml.etree.ElementTree as ET
import json
import math
import time

def parse_xosc_to_json(xosc_file, output_file):
    tree = ET.parse(xosc_file)
    root = tree.getroot()

    # 1. 基础信息配置 (根据你的要求硬编码)
    json_data = {
        "id": "110000",
        "scenario_id": "4",
        "scenario_name": "环岛",
        "case_id": "1",
        "case_name": "30km/h环岛",
        "vut_id": "obu-0003",
        "vut_vel": "30", 
        "participant_id": "obu-0005", # 默认指代主要交互对象
        "participant_vel": "15",      # VT1的近似速度
        "enable_status": True,
        "creat_time": int(time.time() * 1000),
        "update_time": int(time.time() * 1000),
        "TTC": "2.5", # 示例值
        "participant_acc": "0",
        "participant_collision_dis": "TTC * participant_vel + (participant_vel^2) / (2 * participant_acc)",
        "vut_collision_dis": "TTC * vut_vel + (vut_vel * participant_vel) / participant_acc",
        "participant_vut_collision_dis": "sqrt( participant_collision_dis^2 + vut_collision_dis^2 )",
        "trigger_distance": "30",
        "veh_num": 0,
        "veh_content": [],
        "ped_num": 0,
        "ped_content": []
    }

    # 命名空间处理 (xosc通常有xmlns，处理起来比较麻烦，这里忽略命名空间直接查找)
    # 为了简单，我们遍历所有元素查找名为 Vertex 的节点
    
    # 辅助函数：将弧度转换为角度
    def rad_to_deg(rad):
        return float(rad) * (180 / math.pi)

    # 辅助函数：处理单个实体的轨迹
    def process_trajectory(entity_name, vehicle_id, static_velocity=None):
        traj_points = []
        
        # 在XML中查找对应实体的轨迹
        # 逻辑：找到名为Trajectory且name包含entity_name的节点
        target_traj_node = None
        for traj in root.iter():
            if 'Trajectory' in traj.tag and traj.get('name') == f"{entity_name}_Trajectory":
                target_traj_node = traj
                break
        
        if not target_traj_node:
            print(f"Warning: No trajectory found for {entity_name}")
            return None

        # 提取Vertex
        vertices = []
        for vertex in target_traj_node.iter():
            if 'Vertex' in vertex.tag:
                vertices.append(vertex)

        # 转换为JSON格式的点
        prev_x, prev_y, prev_t = None, None, None
        
        for i, vertex in enumerate(vertices):
            t = float(vertex.get('time'))
            pos_node = None
            for child in vertex:
                if 'Position' in child.tag:
                    for wp in child:
                        if 'WorldPosition' in wp.tag:
                            pos_node = wp
                            break
            
            if pos_node is None:
                continue

            x = float(pos_node.get('x'))
            y = float(pos_node.get('y'))
            h_rad = float(pos_node.get('h'))
            heading = rad_to_deg(h_rad)

            # 计算速度 (如果不是强制静态)
            velocity = 0.0
            if static_velocity is not None:
                velocity = static_velocity
            elif prev_x is not None:
                dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                dt = t - prev_t
                if dt > 0:
                    velocity = round(dist / dt, 2)
            
            # 第一个点的速度通常设为和第二个点一样，或者0
            if i == 0 and len(vertices) > 1 and static_velocity is None:
                # 预读下一个点算初速度
                pass # 简化处理，保持0或者在下一个循环修正，这里保持0即可

            point = {
                "time": t,
                "x": x,
                "y": y,
                "xyheading": round(heading, 2),
                "heading": round(heading, 2),
                "velocity": velocity,
                "acc": 0.0 # XML中没有直接加速度数据，设为0
            }
            traj_points.append(point)
            
            prev_x, prev_y, prev_t = x, y, t

        # 构建车辆对象
        veh_obj = {
            "veh_id": vehicle_id,
            "veh_vel": str(int(traj_points[len(traj_points)//2]['velocity'] * 3.6)) if traj_points else "0", # 取中间点速度转km/h作为标志
            "trajectory_num": 1,
            "trajectory_content": traj_points
        }
        return veh_obj

    # 2. 处理 VT1 (移动小车)
    vt1_data = process_trajectory("VT1", "obu-0005")
    if vt1_data:
        json_data["veh_content"].append(vt1_data)

    # 3. 处理 Target (静止车)
    # 虽然XML里Target有位移(其实坐标没变)，我们强制视其速度为0
    target_data = process_trajectory("Target", "obu-0006") 
    if target_data:
        # 修正Target的所有的计算速度为0，因为它是静止障碍物
        target_data["veh_vel"] = "0"
        for p in target_data["trajectory_content"]:
            p["velocity"] = 0.0
        json_data["veh_content"].append(target_data)

    # 更新车辆总数
    json_data["veh_num"] = len(json_data["veh_content"])

    # 4. 写入文件
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent=4, ensure_ascii=False)
    print(f"Successfully converted! Saved to {output_file}")

# --- 执行转换 ---
# 请确保你的XML文件名为 data.xosc
# 如果你的文件名不同，请修改下面这一行
try:
    parse_xosc_to_json("Roundabout.xosc", "output_110000.json")
except FileNotFoundError:
    print("错误：找不到文件 'Roundabout.xosc'。请将你的XML代码保存为该文件。")
except Exception as e:
    print(f"发生错误: {e}")