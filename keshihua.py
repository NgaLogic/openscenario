import math
import matplotlib.pyplot as plt
import numpy as np

# ================= 配置 =================
OUTPUT_VUT = "trajectory_vut.txt"
OUTPUT_TARGET = "trajectory_target.txt"
VUT_SPEED_KMH = 30.0  # km/h
DT = 0.1              # 秒

# ================= 道路数据 =================
# 96 和 197 之间有约 2.8m 的物理间隙，这是造成折角的原因
roads_db = {
    '96':  [(0.0, -360.844, 119.543, 2.247, 9.024, 0.00377)],
    '197': [(0.0, -363.009, 117.806, 5.389, 8.081, -0.03867),
            (8.081, -359.009, 110.822, -1.207, 7.114, -0.08187)],
    '100': [(0.0, -355.037, 103.058, -1.789, 4.383, 0.07879)],
    '101': [(0.0, -354.755, 98.763, -1.444, 11.189, 0.10580)],
    '178': [(0.0, -363.464, 83.910, 0.762, 8.380, -0.03862),
            (8.380, -356.581, 88.625, 0.439, 8.591, -0.08136)] 
}

route_ids = ['96', '197', '100', '101']

# ================= 数学核心 =================
def calc_coords(x0, y0, h0, s, k):
    """标准 OpenDRIVE 几何计算"""
    if abs(k) < 1e-9:
        x = x0 + s * math.cos(h0)
        y = y0 + s * math.sin(h0)
        h = h0
    else:
        h = h0 + s * k
        x = x0 + (math.sin(h) - math.sin(h0)) / k
        y = y0 + (math.cos(h0) - math.cos(h)) / k
    return x, y, h

def get_road_end_state(rid, reverse=False):
    """获取某条路终点的状态 (x, y, h)"""
    geoms = roads_db[rid]
    # 计算顺向的终点
    last_g = geoms[-1]
    # 注意 geometry 的 s 是 offset，长度是 length
    # 局部计算终点
    lx, ly, lh = calc_coords(last_g[1], last_g[2], last_g[3], last_g[4], last_g[5])
    
    if not reverse:
        return lx, ly, lh
    else:
        # 如果这条路是逆向行驶，那它的“终点”其实是物理定义的“起点”
        first_g = geoms[0]
        # 逆向行驶结束时的 Heading 要反转 180度
        return first_g[1], first_g[2], first_g[3] + math.pi

def get_road_start_state(rid, reverse=False):
    """获取某条路起点的状态 (x, y, h)"""
    geoms = roads_db[rid]
    if not reverse:
        g0 = geoms[0]
        return g0[1], g0[2], g0[3]
    else:
        # 逆向的起点是顺向的终点
        return get_road_end_state(rid, reverse=False)[0], get_road_end_state(rid, reverse=False)[1], get_road_end_state(rid, reverse=False)[2] + math.pi

# ================= 平滑插值 (Bezier) =================
def bezier_interp(p0, p3, t):
    """简单三次贝塞尔曲线插值"""
    # 自动推算控制点 p1, p2
    dist = math.sqrt((p3[0]-p0[0])**2 + (p3[1]-p0[1])**2) / 3.0
    p1 = (p0[0] + dist * math.cos(p0[2]), p0[1] + dist * math.sin(p0[2]))
    p2 = (p3[0] - dist * math.cos(p3[2]), p3[1] - dist * math.sin(p3[2]))
    
    inv_t = 1 - t
    x = (inv_t**3)*p0[0] + 3*(inv_t**2)*t*p1[0] + 3*inv_t*(t**2)*p2[0] + (t**3)*p3[0]
    y = (inv_t**3)*p0[1] + 3*(inv_t**2)*t*p1[1] + 3*inv_t*(t**2)*p2[1] + (t**3)*p3[1]
    
    # 简单估算 Heading (切线)
    dx = 3*(inv_t**2)*(p1[0]-p0[0]) + 6*inv_t*t*(p2[0]-p1[0]) + 3*(t**2)*(p3[0]-p2[0])
    dy = 3*(inv_t**2)*(p1[1]-p0[1]) + 6*inv_t*t*(p2[1]-p1[1]) + 3*(t**2)*(p3[1]-p2[1])
    h = math.atan2(dy, dx)
    return x, y, h

# ================= 主逻辑 =================
print("生成轨迹中 (启用自动填补间隙)...")

# 1. 确定每条路的方向 (96是逆向，其他是顺向)
# 手动指定最稳妥，基于之前的分析
road_configs = [
    {'id': '96',  'rev': True},  # 逆向
    {'id': '197', 'rev': False}, # 顺向
    {'id': '100', 'rev': False}, # 顺向
    {'id': '101', 'rev': False}  # 顺向
]

vut_points = []
speed_mps = VUT_SPEED_KMH / 3.6
current_t = 0.0

for i in range(len(road_configs)):
    cfg = road_configs[i]
    rid = cfg['id']
    is_rev = cfg['rev']
    
    # 生成当前路段的轨迹
    # 采样步长
    geoms = roads_db[rid]
    total_len = sum([g[4] for g in geoms])
    
    # 在这条路上跑需要多久
    duration = total_len / speed_mps
    steps = int(duration / DT)
    
    for s_step in range(steps):
        dist_driven = s_step * DT * speed_mps
        
        # 计算局部 s
        local_s = dist_driven if not is_rev else (total_len - dist_driven)
        
        # 找 geometry
        acc_s = 0
        gx, gy, gh = 0, 0, 0
        
        # 如果是逆向，我们需要仔细倒推 geometry
        # 简单起见，我们不管 geometry 顺序，直接用总长度减去 local_s 定位
        # 寻找 local_s 落在哪个 geometry
        found_g = False
        for g in geoms:
            if local_s <= acc_s + g[4] + 0.001:
                geo_s = local_s - acc_s
                gx, gy, gh = calc_coords(g[1], g[2], g[3], geo_s, g[5])
                found_g = True
                break
            acc_s += g[4]
            
        if not found_g: # 精度误差处理
            last = geoms[-1]
            gx, gy, gh = calc_coords(last[1], last[2], last[3], last[4], last[5])

        if is_rev: gh += math.pi
        
        vut_points.append({'t': current_t, 'x': gx, 'y': gy, 'h': gh})
        current_t += DT

    # === 关键：处理与下一条路的间隙 (Gap Filling) ===
    if i < len(road_configs) - 1:
        next_cfg = road_configs[i+1]
        
        # 当前路终点
        p_curr_end = get_road_end_state(rid, is_rev) # (x, y, h)
        # 下一路起点
        p_next_start = get_road_start_state(next_cfg['id'], next_cfg['rev'])
        
        # 计算间隙距离
        gap_dist = math.sqrt((p_curr_end[0]-p_next_start[0])**2 + (p_curr_end[1]-p_next_start[1])**2)
        
        if gap_dist > 0.1: # 如果间隙大于 10cm
            print(f"  [修复] 发现间隙 {gap_dist:.2f}m (Road {rid} -> {next_cfg['id']})，正在生成过渡曲线...")
            
            # 估算通过间隙的时间
            gap_time = gap_dist / speed_mps
            gap_steps = int(gap_time / DT)
            
            for g_step in range(1, gap_steps + 1): # 留头不留尾
                t_ratio = g_step / (gap_steps + 1)
                bx, by, bh = bezier_interp(p_curr_end, p_next_start, t_ratio)
                vut_points.append({'t': current_t, 'x': bx, 'y': by, 'h': bh})
                current_t += DT

# ================= 生成目标车 (手动指定位置) =================

# 1. 在这里填入你想要的数值
manual_x = -358.68   # 你的 X 坐标
manual_y = 81.02     # 你的 Y 坐标
manual_h_deg = 45.0   # 你的朝向 (角度，0=向东, 90=向北)

# 2. 自动转换角度为弧度 (代码内部用弧度)
manual_h_rad = math.radians(manual_h_deg)

# 3. 锁定位置
best_t_pt = (manual_x, manual_y, manual_h_rad)

# 4. 生成轨迹点 (保持静止)
target_points = []
for p in vut_points:
    # 每一帧都呆在同一个位置
    target_points.append({
        't': p['t'], 
        'x': best_t_pt[0], 
        'y': best_t_pt[1], 
        'h': best_t_pt[2]
    })

# ================= 保存 =================
def save_f(fname, data):
    with open(fname, 'w') as f:
        for p in data:
            f.write(f'<Vertex time="{p["t"]:.4f}">\n')
            f.write(f'    <Position><WorldPosition x="{p["x"]:.4f}" y="{p["y"]:.4f}" z="0" h="{p["h"]:.4f}"/></Position>\n')
            f.write('</Vertex>\n')
    print(f"已生成: {fname}")

save_f(OUTPUT_VUT, vut_points)
save_f(OUTPUT_TARGET, target_points)

# 计算最小会车距离
best_dist = float('inf')
for p in vut_points:
    dist = math.sqrt((p['x'] - best_t_pt[0])**2 + (p['y'] - best_t_pt[1])**2)
    if dist < best_dist:
        best_dist = dist
print(f"完成。最终会车距离: {best_dist:.2f}m")

# 绘图
plt.figure(figsize=(6,6))
px = [p['x'] for p in vut_points]
py = [p['y'] for p in vut_points]
plt.plot(px, py, 'b.-', label='VUT (Smoothed)')
plt.plot(best_t_pt[0], best_t_pt[1], 'ro', label='Target')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.title("Smoothed Trajectory (Gap Filled)")
plt.show()