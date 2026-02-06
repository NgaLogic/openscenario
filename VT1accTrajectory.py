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
    
    # 计算加速阶段需要的距离和时间
    # v^2 - v0^2 = 2as  =>  s = v^2 / 2a
    acc_distance = (TARGET_SPEED_MS ** 2) / (2 * ACC)
    acc_time_duration = TARGET_SPEED_MS / ACC

    print(f"--- 运动学参数 ---")
    print(f"目标速度: {TARGET_SPEED_KMH} km/h ({TARGET_SPEED_MS:.3f} m/s)")
    print(f"加速度: {ACC} m/s^2")
    print(f"加速距离: {acc_distance:.3f} 米 (在此距离前加速，之后匀速)")
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
    if total_raw_points < 2:
        print("点数太少，无法计算！")
        return

    # ================= 2. 重新计算时间与速度 =================
    final_points = []
    cumulative_distance = 0.0
    
    # 辅助函数：弧度转角度
    def rad_to_deg(r):
        return r * (180 / math.pi)

    # 处理第0个点
    start_p = raw_points[0]
    final_points.append({
        "time": 0.0,
        "x": start_p['x'],
        "y": start_p['y'],
        "heading": round(rad_to_deg(start_p['h_rad']), 2),
        "h_rad": start_p['h_rad'],  # 保存原始弧度
        "velocity": 0.0,
        "acc": ACC,
        "stage": "启动"
    })

    prev_x = start_p['x']
    prev_y = start_p['y']

    for i in range(1, total_raw_points):
        curr_p = raw_points[i]
        
        # 计算这一段的距离
        dx = curr_p['x'] - prev_x
        dy = curr_p['y'] - prev_y
        dist_segment = math.sqrt(dx*dx + dy*dy)
        cumulative_distance += dist_segment

        current_time = 0.0
        current_vel = 0.0
        current_stage = ""

        # === 核心逻辑：判断是在加速段还是匀速段 ===
        if cumulative_distance <= acc_distance:
            # 【阶段1：纯加速】
            # S = 0.5 * a * t^2  =>  t = sqrt(2S / a)
            current_time = math.sqrt(2 * cumulative_distance / ACC)
            # v = a * t
            current_vel = ACC * current_time
            current_stage = "加速中"
        
        else:
            # 【阶段2：达到极速，匀速行驶】
            # 时间 = 加速段时间 + (总距离 - 加速段距离) / 匀速速度
            dist_in_cruise = cumulative_distance - acc_distance
            time_in_cruise = dist_in_cruise / TARGET_SPEED_MS
            current_time = acc_time_duration + time_in_cruise
            
            current_vel = TARGET_SPEED_MS # 保持极速
            current_stage = "匀速"

        # 添加点
        final_points.append({
            "time": round(current_time, 3),
            "x": curr_p['x'],
            "y": curr_p['y'],
            "heading": round(rad_to_deg(curr_p['h_rad']), 2),
            "h_rad": curr_p['h_rad'],  # 保存原始弧度供后续使用
            "velocity": round(current_vel, 2),
            "acc": ACC if current_stage == "加速中" else 0.0,
            "stage": current_stage
        })

        prev_x = curr_p['x']
        prev_y = curr_p['y']

    # ================= 3. 输出检查 =================
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

    # ================= 4. 输出 XOSC 格式 =================
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
calculate_trajectory_points("Roundabout.xosc")