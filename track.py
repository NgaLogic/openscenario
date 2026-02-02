import math
import os

# ================= 配置区域 =================
# 输出文件名
OUTPUT_VUT = "trajectory_vut.txt"
OUTPUT_TARGET = "trajectory_target.txt"

# 车辆参数
VUT_SPEED_KMH = 30.0  # 主车速度
DT = 0.1              # 时间步长 (秒)

# ================= 道路几何定义 (从 xodr 提取) =================
# 数据结构: (s_start, x, y, hdg, length, curvature)
# 注意：多段路会按顺序拼接
roads_db = {
    '96': [
        (0.0, -360.844, 119.543, 2.247, 9.024, 0.00377)
    ],
    '197': [
        (0.0, -363.009, 117.806, 5.389, 8.081, -0.03867),
        (8.081, -359.009, 110.822, -1.207, 7.114, -0.08187)
    ],
    '100': [
        (0.0, -355.037, 103.058, -1.789, 4.383, 0.07879)
    ],
    '101': [
        (0.0, -354.755, 98.763, -1.444, 11.189, 0.10580)
    ],
    '178': [
        (0.0, -363.464, 83.910, 0.762, 8.380, -0.03862),
        (8.380, -356.581, 88.625, 0.439, 8.591, -0.08136)
    ]
}

# ================= 核心计算逻辑 =================

def calc_point_on_arc(x0, y0, h0, s, k):
    """ 计算圆弧/直线上某一点的坐标 """
    if abs(k) < 1e-6:
        # 直线模型
        x = x0 + s * math.cos(h0)
        y = y0 + s * math.sin(h0)
        h = h0
    else:
        # 圆弧模型
        h = h0 + s * k
        x = x0 + (math.sin(h) - math.sin(h0)) / k
        y = y0 + (math.cos(h0) - math.cos(h)) / k
    return x, y, h

def sample_road_geometry(road_id, step_size=1.0):
    """ 对某条路进行离散化采样 """
    segments = roads_db[road_id]
    points = []
    
    for seg in segments:
        s_start, x0, y0, h0, length, k = seg
        # 在该段几何内采样
        num_steps = int(length / step_size) + 1
        for i in range(num_steps):
            ds = i * step_size
            if ds > length: ds = length
            
            px, py, ph = calc_point_on_arc(x0, y0, h0, ds, k)
            points.append((px, py, ph))
            
    return points

def generate_continuous_path(road_id_sequence, speed_mps, dt):
    """ 生成连续的时间-空间轨迹点 """
    traj_points = []
    current_time = 0.0
    
    for rid in road_id_sequence:
        segments = roads_db[rid]
        
        # 遍历该路的所有几何段
        for seg in segments:
            s_local_start, x0, y0, h0, length, k = seg
            
            # 在当前路段上行驶的时间
            duration = length / speed_mps
            num_steps = int(duration / dt)
            
            for i in range(num_steps):
                ds = speed_mps * (i * dt) # 当前段走过的距离
                px, py, ph = calc_point_on_arc(x0, y0, h0, ds, k)
                
                # 转换 heading 到角度 (0-360 或 -180-180)
                # h_deg = math.degrees(ph) % 360
                h_rad = ph # 保持弧度供后续写入（如果需要角度再转）
                
                traj_points.append({
                    'time': current_time,
                    'x': px, 'y': py, 'h': ph
                })
                current_time += dt
                
    return traj_points

# ================= 主程序 =================

def write_trajectory_file(filename, points):
    with open(filename, 'w', encoding='utf-8') as f:
        for p in points:
            t = p['time']
            x = p['x']
            y = p['y']
            h = p['h']
            
            f.write(f'<Vertex time="{t:.6f}">\n')
            f.write('    <Position>\n')
            # h 输出为弧度或角度？OpenDRIVE通常用弧度，ESMINI/OpenScenario有时用弧度
            # 这里直接输出弧度值
            f.write(f'        <WorldPosition x="{x:.6f}" y="{y:.6f}" z="0" h="{h:.6f}" p="0" r="0"/>\n')
            f.write('    </Position>\n')
            f.write('</Vertex>\n')
    print(f"已生成: {filename} ({len(points)} 个点)")

if __name__ == "__main__":
    # 1. 准备参数
    speed_mps = VUT_SPEED_KMH / 3.6
    
    # 2. 生成 VUT 轨迹 (96 -> 197 -> 100 -> 101)
    vut_path_ids = ['96', '197', '100', '101']
    vut_points = generate_continuous_path(vut_path_ids, speed_mps, DT)
    
    # 3. 生成 Target 轨迹 (178 上静止)
    # 我们把静止车放在 178 的第二段开始处 (坐标约 -356, 88)，这里离 101 的终点很近
    target_seg = roads_db['178'][1] # 取第二段
    tx, ty, th = calc_point_on_arc(target_seg[1], target_seg[2], target_seg[3], 0.0, target_seg[5])
    
    target_points = []
    # 目标车的时间长度与主车一致
    total_time = vut_points[-1]['time']
    num_steps = int(total_time / DT)
    
    for i in range(num_steps + 1):
        t = i * DT
        target_points.append({
            'time': t,
            'x': tx, 'y': ty, 'h': th
        })
        
    # 4. 写入文件
    write_trajectory_file(OUTPUT_VUT, vut_points)
    write_trajectory_file(OUTPUT_TARGET, target_points)
    
    print("\n完成！")
    print(f"主车 (VUT) 最终位置: x={vut_points[-1]['x']:.2f}, y={vut_points[-1]['y']:.2f}")
    print(f"目标车 (Target) 位置: x={target_points[0]['x']:.2f}, y={target_points[0]['y']:.2f}")
    
    # 简单的距离检查
    dist = math.sqrt((vut_points[-1]['x'] - tx)**2 + (vut_points[-1]['y'] - ty)**2)
    print(f"最终两车距离: {dist:.2f} 米 (如果小于5米则说明成功会面)")