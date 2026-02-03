import math
import matplotlib.pyplot as plt

# ================= 配置 =================
OUTPUT_FILE = "trajectory_vt1.txt"

# 1. VT1 起点 (你给定的坐标)
START_X = -332.0
START_Y = 118.0
# 2. VT1 朝向 (西南方向)
START_H = -2.3562

# 3. 速度 (根据之前的表格设为 15km/h，可自行修改)
SPEED_KMH = 15.0 
DT = 0.1               # 采样时间步长

# 环岛几何参数 (保持不变)
CENTER_X = -345.18
CENTER_Y = 100.73
RADIUS = 13.0          # 保持与VUT一致的行驶半径

# ================= 核心工具函数 =================

def normalize_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def get_circle_pt(angle):
    # 获取圆上某角度的坐标和切线朝向(CCW)
    x = CENTER_X + RADIUS * math.cos(angle)
    y = CENTER_Y + RADIUS * math.sin(angle)
    h = normalize_angle(angle + math.pi / 2)
    return x, y, h

def generate_trajectory():
    points = []
    current_t = 0.0
    speed_mps = SPEED_KMH / 3.6
    
    # --- 第1步：策略规划 ---
    # VT1 从东北侧(-332, 118) 向西南切入
    # 计算切入点：选择圆环上角度为 1.0 弧度 (约57度) 的位置
    # 这个位置在东北侧，切线方向与入环方向能顺滑连接
    entry_angle_rad = 1.0 
    
    best_straight_len = 3.0   # 直行缓冲距离
    
    # --- 第2步：生成直行段 (Straight) ---
    steps_straight = int(best_straight_len / (speed_mps * DT))
    p_end_straight = (START_X, START_Y, START_H)
    
    print(f"1. 生成直行段: 长度 {best_straight_len:.2f}m")
    for i in range(steps_straight + 1):
        dist = i * DT * speed_mps
        px = START_X + dist * math.cos(START_H)
        py = START_Y + dist * math.sin(START_H)
        points.append({'t': current_t, 'x': px, 'y': py, 'h': START_H})
        current_t += DT
        p_end_straight = (px, py, START_H)
        
    # --- 第3步：生成右转入环段 (Transition Bezier) ---
    entry_x, entry_y, entry_h = get_circle_pt(entry_angle_rad)
    
    p0 = (p_end_straight[0], p_end_straight[1])
    p3 = (entry_x, entry_y)
    
    # 计算控制点距离
    dist_connect = math.sqrt((p3[0]-p0[0])**2 + (p3[1]-p0[1])**2)
    ctrl_len = dist_connect * 0.4  # 系数0.4，避免弯道过冲
    
    # P1: 起点沿朝向延伸
    p1 = (p0[0] + ctrl_len * math.cos(p_end_straight[2]),
          p0[1] + ctrl_len * math.sin(p_end_straight[2]))
    # P2: 终点沿切线反向延伸
    p2 = (p3[0] - ctrl_len * math.cos(entry_h),
          p3[1] - ctrl_len * math.sin(entry_h))
    
    curve_steps = 30 # 采样点数
    print(f"2. 生成入环弯道: 目标点 ({entry_x:.2f}, {entry_y:.2f})")
    
    for i in range(1, curve_steps + 1):
        t_val = i / curve_steps
        mt = 1 - t_val
        
        # 三阶贝塞尔公式
        bx = mt**3*p0[0] + 3*mt**2*t_val*p1[0] + 3*mt*t_val**2*p2[0] + t_val**3*p3[0]
        by = mt**3*p0[1] + 3*mt**2*t_val*p1[1] + 3*mt*t_val**2*p2[1] + t_val**3*p3[1]
        
        # 计算切线角度
        dx = 3*mt**2*(p1[0]-p0[0]) + 6*mt*t_val*(p2[0]-p1[0]) + 3*t_val**2*(p3[0]-p2[0])
        dy = 3*mt**2*(p1[1]-p0[1]) + 6*mt*t_val*(p2[1]-p1[1]) + 3*t_val**2*(p3[1]-p2[1])
        bh = math.atan2(dy, dx)
        
        # 累加时间
        prev_p = points[-1]
        step_dist = math.sqrt((bx - prev_p['x'])**2 + (by - prev_p['y'])**2)
        current_t += step_dist / speed_mps
        
        points.append({'t': current_t, 'x': bx, 'y': by, 'h': bh})

    # --- 第4步：生成环行段 (Circle) ---
    # 从入环点(1.0) 绕行到 环岛左侧/下方 
    # VUT在下方(-1.57)，为了制造碰撞或交汇，VT1应该多绕一点
    start_ang = entry_angle_rad
    end_ang = start_ang + math.pi # 绕半圈，足够覆盖碰撞区域
    
    arc_len = (end_ang - start_ang) * RADIUS
    circle_steps = int(arc_len / (speed_mps * DT))
    
    print(f"3. 生成环行段: 弧长 {arc_len:.2f}m")
    
    for i in range(1, circle_steps + 1):
        frac = i / circle_steps
        ang = start_ang + frac * (end_ang - start_ang)
        cx, cy, ch = get_circle_pt(ang)
        points.append({'t': current_t, 'x': cx, 'y': cy, 'h': ch})
        current_t += DT

    return points

# ================= 执行并保存 =================

vt1_path = generate_trajectory()

with open(OUTPUT_FILE, 'w') as f:
    for p in vt1_path:
        # 注意: xosc 里的 time 如果是 relative 模式，这里的时间就是相对触发后的时间
        f.write(f'<Vertex time="{p["t"]:.4f}">\n')
        f.write(f'    <Position><WorldPosition x="{p["x"]:.4f}" y="{p["y"]:.4f}" z="0" h="{p["h"]:.4f}"/></Position>\n')
        f.write('</Vertex>\n')

print(f"\n成功生成: {OUTPUT_FILE}")

# ================= 绘图验证 =================
try:
    px = [p['x'] for p in vt1_path]
    py = [p['y'] for p in vt1_path]
    plt.figure(figsize=(6,6))
    
    # 画轨迹
    plt.plot(px, py, 'r.-', label='VT1 Path') # 红色轨迹
    # 画圆心和圆
    plt.plot(CENTER_X, CENTER_Y, 'kx', label='Center')
    circle = plt.Circle((CENTER_X, CENTER_Y), RADIUS, color='gray', fill=False, linestyle='--')
    plt.gca().add_patch(circle)
    
    # 标记起点
    plt.plot(START_X, START_Y, 'go', label='Start')
    
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title(f"VT1 Trajectory\nStart: ({START_X}, {START_Y})")
    plt.show()
except Exception as e:
    print("绘图库不可用或出错，但文件已生成。", e)