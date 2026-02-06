import xml.etree.ElementTree as ET
import math

def calculate_trajectory_points(xosc_file):
    print(f"正在读取文件: {xosc_file} ...")
    try:
        tree = ET.parse(xosc_file)
        root = tree.getroot()
    except Exception as e:
        print(f"读取XML出错: {e}")
        return

    # ================= 参数设置 =================
    ACC = 1.5                # 加速度 m/s^2
    TARGET_SPEED_KMH = 15.0  # 目标速度 km/h
    TARGET_SPEED_MS = TARGET_SPEED_KMH / 3.6 # 换算为 m/s (约 4.167)
    
    # 新的起点和原始起点
    NEW_START_X = -327.68
    NEW_START_Y = 122.16
    ORIGINAL_START_X = -332.0
    ORIGINAL_START_Y = 118.0
    
    # 计算加速阶段的参数
    acc_distance = (TARGET_SPEED_MS ** 2) / (2 * ACC)
    acc_time_duration = TARGET_SPEED_MS / ACC
    
    # 计算从新起点到原始起点的距离和方向
    dx_total = ORIGINAL_START_X - NEW_START_X
    dy_total = ORIGINAL_START_Y - NEW_START_Y
    distance_to_original = math.sqrt(dx_total**2 + dy_total**2)
    
    # 单位方向向量
    dir_x = dx_total / distance_to_original
    dir_y = dy_total / distance_to_original
    
    # 计算方向角
    heading_to_original = math.atan2(dy_total, dx_total)

    print(f"--- 运动学参数 ---")
    print(f"目标速度: {TARGET_SPEED_KMH} km/h ({TARGET_SPEED_MS:.3f} m/s)")
    print(f"加速度: {ACC} m/s^2")
    print(f"加速距离: {acc_distance:.3f} 米")
    print(f"从新起点 ({NEW_START_X}, {NEW_START_Y}) 到原始起点 ({ORIGINAL_START_X}, {ORIGINAL_START_Y})")
    print(f"距离: {distance_to_original:.3f} 米")
    print(f"------------------\n")

    # ================= 1. 提取原始几何点 =================
    raw_points = []
    
    # 查找 "VT1_Trajectory" 的轨迹节点
    # 在 OpenSCENARIO 中，轨迹在 FollowTrajectoryAction 中
    trajectory_node = None
    for traj in root.iter():
        traj_name = traj.get('name')
        if traj_name and 'VT1_Trajectory' in traj_name:
            trajectory_node = traj
            print(f"找到轨迹: {traj_name}")
            break
            
    if not trajectory_node:
        print("错误：在XML中找不到 'VT1_Trajectory'！")
        print("正在列出所有 Trajectory...")
        for traj in root.iter():
            if traj.get('name') and 'Trajectory' in traj.get('name'):
                print(f"  - {traj.get('name')}")
        return

    # 遍历该轨迹下所有的 Vertex
    # Vertex 是 Polyline 的子节点
    for vertex in trajectory_node.iter():
        if 'Vertex' in vertex.tag:
            # WorldPosition 在 Position 中
            pos = None
            for child in vertex.iter():
                if 'WorldPosition' in child.tag:
                    pos = child
                    break
            
            if pos is not None:
                try:
                    p = {
                        "x": float(pos.get('x')),
                        "y": float(pos.get('y')),
                        "h_rad": float(pos.get('h'))
                    }
                    raw_points.append(p)
                except (TypeError, ValueError):
                    continue

    total_raw_points = len(raw_points)
    print(f"提取到原始路径点数: {total_raw_points}")
    
    # 限制只使用前183个点（原始轨迹）
    if total_raw_points > 183:
        raw_points = raw_points[:183]
        total_raw_points = 183
        print(f"限制为前183个点（原始轨迹）")
    
    print(f"原始轨迹第一个点: ({raw_points[0]['x']:.2f}, {raw_points[0]['y']:.2f})")
    print(f"原始轨迹最后一个点: ({raw_points[-1]['x']:.2f}, {raw_points[-1]['y']:.2f})")
    print()

    # ================= 2. 生成从新起点到原始起点的加速段 =================
    final_points = []
    
    # 辅助函数：弧度转角度
    def rad_to_deg(r):
        return r * (180 / math.pi)
    
    # 步长：0.1 m 间隔
    step_size = 0.1
    num_steps = int(distance_to_original / step_size)
    
    for step in range(num_steps + 1):
        progress = min(step / num_steps, 1.0) if num_steps > 0 else 1.0
        
        # 当前点位置
        curr_x = NEW_START_X + dx_total * progress
        curr_y = NEW_START_Y + dy_total * progress
        curr_distance = distance_to_original * progress
        
        # 计算运动状态（从新起点开始加速）
        if curr_distance <= acc_distance:
            # 加速段
            current_time = math.sqrt(2 * curr_distance / ACC) if curr_distance > 0 else 0.0
            current_vel = ACC * current_time
            current_stage = "加速中"
        else:
            # 到达原始起点时已达目标速度，之后匀速
            dist_in_cruise = curr_distance - acc_distance
            time_in_cruise = dist_in_cruise / TARGET_SPEED_MS
            current_time = acc_time_duration + time_in_cruise
            current_vel = TARGET_SPEED_MS
            current_stage = "匀速"
        
        final_points.append({
            "time": round(current_time, 3),
            "x": curr_x,
            "y": curr_y,
            "heading": round(rad_to_deg(heading_to_original), 2),
            "h_rad": heading_to_original,
            "velocity": round(current_vel, 2),
            "acc": ACC if current_stage == "加速中" else 0.0,
            "stage": current_stage
        })
    
    # 记录到达原始起点时的时间
    time_at_original_start = final_points[-1]['time']
    print(f"到达原始起点时: 时间={time_at_original_start:.3f}s, 速度={final_points[-1]['velocity']:.2f} m/s\n")

    # ================= 3. 添加原始轨迹（以匀速4.17m/s继续）=================
    # 第二段直接以目标速度匀速行驶，不重新加速
    for i in range(total_raw_points):
        curr_p = raw_points[i]
        
        # 计算从起点到当前点的累积距离
        cumulative_distance = 0.0
        for j in range(1, i + 1):
            prev_p = raw_points[j - 1]
            dx = raw_points[j]['x'] - prev_p['x']
            dy = raw_points[j]['y'] - prev_p['y']
            cumulative_distance += math.sqrt(dx*dx + dy*dy)
        
        # 以匀速速度计算时间（从原始起点开始）
        time_from_original = cumulative_distance / TARGET_SPEED_MS
        
        # 总时间 = 加速段时间 + 原始轨迹上的时间
        final_time = time_at_original_start + time_from_original
        
        final_points.append({
            "time": round(final_time, 3),
            "x": curr_p['x'],
            "y": curr_p['y'],
            "heading": round(rad_to_deg(curr_p['h_rad']), 2),
            "h_rad": curr_p['h_rad'],
            "velocity": round(TARGET_SPEED_MS, 2),  # 始终保持目标速度
            "acc": 0.0,  # 匀速，加速度为0
            "stage": "匀速"
        })

    # ================= 4. 输出检查 =================
    print("\n--- 计算结果预览 (前5个点) ---")
    for p in final_points[:5]:
        print(p)

    print("\n--- 计算结果预览 (中间点) ---")
    mid_idx = len(final_points) // 2
    print(final_points[mid_idx])

    print("\n--- 计算结果预览 (最后5个点) ---")
    for p in final_points[-5:]:
        print(p)

    print(f"\n统计:")
    print(f"总点数: {len(final_points)}")
    print(f"总耗时: {final_points[-1]['time']} 秒")
    print(f"最终速度: {final_points[-1]['velocity']} m/s (应接近 {TARGET_SPEED_MS:.3f})")

    # ================= 5. 输出 XOSC 格式 =================
    print(f"\n--- XOSC 格式的 Vertex 点 ---\n")
    
    xosc_output = []
    for point in final_points:
        vertex_str = f'<Vertex time="{point["time"]:.4f}">\n'
        vertex_str += f'    <Position><WorldPosition x="{point["x"]:.4f}" y="{point["y"]:.4f}" z="0" h="{point["h_rad"]:.4f}"/></Position>\n'
        vertex_str += '</Vertex>'
        xosc_output.append(vertex_str)
        print(vertex_str)
    
    # 保存到文件
    output_file = "VT1_trajectory_output.txt"
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(xosc_output))
    
    print(f"\n已保存到文件: {output_file}")

# 运行
calculate_trajectory_points("Roundabout_20260203_full.xosc")