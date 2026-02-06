import math
import matplotlib.pyplot as plt

# ================= 配置 =================
OUTPUT_VUT = "trajectory_vut.txt"
OUTPUT_TARGET = "trajectory_target.txt"

# 1. 更新起点 (你指定的新坐标)
START_X = -372.0
START_Y = 131.0
# 2. 修正朝向 (使用负值避免打转，请确保 xosc Init 里也是这个值!)
START_H = -0.8606      # 约 -49度 (东南方向)

VUT_SPEED_KMH = 30.0   # 速度
DT = 0.1               # 采样时间步长

# 环岛几何参数
CENTER_X = -345.18
CENTER_Y = 100.73
RADIUS = 13#原10.12

# 目标：停在环岛正下方
BOTTOM_ANGLE = -math.pi / 2 

# ================= 核心工具函数 =================

def normalize_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def get_circle_pt(angle):
    x = CENTER_X + RADIUS * math.cos(angle)
    y = CENTER_Y + RADIUS * math.sin(angle)
    h = normalize_angle(angle + math.pi / 2)
    return x, y, h

def generate_trajectory():
    points = []
    current_t = 0.0
    speed_mps = VUT_SPEED_KMH / 3.6
    
    # --- 第1步：计算最佳入环策略 (针对新起点重新优化) ---
    # 新起点距离较远且指向圆心，需要较短的直行接长距离的缓弯
    best_straight_len = 5.0   # 直行 5米
    entry_angle_rad = 3.42    # 约 196度 切入 (非常顺滑)
    
    # --- 第2步：生成直行段 (Straight) ---
    steps_straight = int(best_straight_len / (speed_mps * DT))
    p_end_straight = (0,0,0) 
    
    print(f"1. 生成直行段: 长度 {best_straight_len:.2f}m")
    for i in range(steps_straight + 1):
        dist = i * DT * speed_mps
        px = START_X + dist * math.cos(START_H)
        py = START_Y + dist * math.sin(START_H)
        points.append({'t': current_t, 'x': px, 'y': py, 'h': START_H})
        current_t += DT
        p_end_straight = (px, py, START_H)
        
    # --- 第3步：生成右转入环段 (Transition Curve) ---
    entry_x, entry_y, entry_h = get_circle_pt(entry_angle_rad)
    
    p0 = (p_end_straight[0], p_end_straight[1])
    p3 = (entry_x, entry_y)
    
    dist_connect = math.sqrt((p3[0]-p0[0])**2 + (p3[1]-p0[1])**2)
    # 增加控制柄长度系数，使弯道更平缓
    ctrl_len = dist_connect * 0.5 
    
    p1 = (p0[0] + ctrl_len * math.cos(p_end_straight[2]),
          p0[1] + ctrl_len * math.sin(p_end_straight[2]))
    p2 = (p3[0] - ctrl_len * math.cos(entry_h),
          p3[1] - ctrl_len * math.sin(entry_h))
    
    curve_steps = 25 # 增加采样点保证平滑
    print(f"2. 生成右转入环段: 目标切入点 ({entry_x:.2f}, {entry_y:.2f})")
    
    for i in range(1, curve_steps + 1):
        t_val = i / curve_steps
        mt = 1 - t_val
        
        bx = mt**3*p0[0] + 3*mt**2*t_val*p1[0] + 3*mt*t_val**2*p2[0] + t_val**3*p3[0]
        by = mt**3*p0[1] + 3*mt**2*t_val*p1[1] + 3*mt*t_val**2*p2[1] + t_val**3*p3[1]
        
        dx = 3*mt**2*(p1[0]-p0[0]) + 6*mt*t_val*(p2[0]-p1[0]) + 3*t_val**2*(p3[0]-p2[0])
        dy = 3*mt**2*(p1[1]-p0[1]) + 6*mt*t_val*(p2[1]-p1[1]) + 3*t_val**2*(p3[1]-p2[1])
        bh = math.atan2(dy, dx)
        
        prev_p = points[-1]
        step_dist = math.sqrt((bx - prev_p['x'])**2 + (by - prev_p['y'])**2)
        current_t += step_dist / speed_mps
        
        points.append({'t': current_t, 'x': bx, 'y': by, 'h': bh})

    # --- 第4步：生成环行段 (Circle) ---
    start_ang = entry_angle_rad
    end_ang = 3 * math.pi / 2 + 0.1 
    
    if end_ang < start_ang: end_ang += 2*math.pi
        
    arc_len = (end_ang - start_ang) * RADIUS
    circle_steps = int(arc_len / (speed_mps * DT))
    
    print(f"3. 生成环行段: 到达底部，行驶弧长 {arc_len:.2f}m")
    
    for i in range(1, circle_steps + 1):
        frac = i / circle_steps
        ang = start_ang + frac * (end_ang - start_ang)
        cx, cy, ch = get_circle_pt(ang)
        points.append({'t': current_t, 'x': cx, 'y': cy, 'h': ch})
        current_t += DT

    return points

# ================= 执行并保存 =================

vut_path = generate_trajectory()

# 保存 VUT
with open(OUTPUT_VUT, 'w') as f:
    for p in vut_path:
        f.write(f'<Vertex time="{p["t"]:.4f}">\n')
        f.write(f'    <Position><WorldPosition x="{p["x"]:.4f}" y="{p["y"]:.4f}" z="0" h="{p["h"]:.4f}"/></Position>\n')
        f.write('</Vertex>\n')

# 生成 Target (保持静止)
with open(OUTPUT_TARGET, 'w') as f:
    target_x, target_y = -358.68, 81.02
    for p in vut_path:
        f.write(f'<Vertex time="{p["t"]:.4f}">\n')
        f.write(f'    <Position><WorldPosition x="{target_x}" y="{target_y}" z="0" h="0.87"/></Position>\n')
        f.write('</Vertex>\n')

print(f"\n成功生成轨迹文件: {OUTPUT_VUT}")
print(f"请记得修改 xosc Init 部分坐标为: x={START_X}, y={START_Y}, h={START_H}")

# ================= 绘图验证 =================
try:
    px = [p['x'] for p in vut_path]
    py = [p['y'] for p in vut_path]
    plt.figure(figsize=(6,6))
    plt.plot(px, py, 'b.-', label='VUT Path')
    plt.plot(CENTER_X, CENTER_Y, 'kx', label='Center')
    circle = plt.Circle((CENTER_X, CENTER_Y), RADIUS, color='g', fill=False, linestyle='--')
    plt.gca().add_patch(circle)
    plt.axis('equal')
    plt.legend()
    plt.title("Updated Path: New Start (-372, 131)")
    plt.show()
except:
    pass