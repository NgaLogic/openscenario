import math
import matplotlib.pyplot as plt

# ================= 1. 参数配置区 =================

OUTPUT_FILE = "trajectory_vt1.txt"

# [车辆起点]
START_X = -332.0
START_Y = 118.0
START_H = -2.3562  # -135度

# [目标直道方向]
EXIT_P1_X, EXIT_P1_Y = -366.0, 86.0
EXIT_P2_X, EXIT_P2_Y = -397.0, 58.0

# [关键参数：回旋线长度]
# 越长出环越缓，越短出环越急
SPIRAL_LENGTH = 15.0 

# 【核心修改：额外绕行角度】 (单位：度)
# 0.0  = 自动计算最佳切点
# 30.0 = 在环岛上多走 30 度 (延迟出环)
# -15.0 = 少走 15 度 (提早出环)
EXTRA_ROTATION_DEG = 35.0  # <--- 在这里修改你想要的角度大小！

# [运动参数]
SPEED_KMH = 15.0
SPEED_MS = SPEED_KMH / 3.6
DT = 0.1

# [环岛几何]
CENTER_X = -345.18
CENTER_Y = 100.73
RADIUS = 13.0

# ================= 2. 数学工具 =================

class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        self.a0 = xs; self.a1 = vxs; self.a2 = axs / 2.0
        t = time; t2 = t*t; t3 = t2*t; t4 = t3*t; t5 = t4*t
        A11, A12, A13 = t3, t4, t5
        A21, A22, A23 = 3*t2, 4*t3, 5*t4
        A31, A32, A33 = 6*t, 12*t2, 20*t3
        b1 = xe - self.a0 - self.a1*t - self.a2*t2
        b2 = vxe - self.a1 - 2*self.a2*t
        b3 = axe - 2*self.a2
        det_A = A11*(A22*A33 - A23*A32) - A12*(A21*A33 - A23*A31) + A13*(A21*A32 - A22*A31)
        det_X = b1*(A22*A33 - A23*A32) - A12*(b2*A33 - A23*b3) + A13*(b2*A32 - A22*b3)
        det_Y = A11*(b2*A33 - A23*b3) - b1*(A21*A33 - A23*A31) + A13*(A21*b3 - b2*A31)
        det_Z = A11*(A22*b3 - b2*A32) - A12*(A21*b3 - b2*A31) + b1*(A21*A32 - A22*A31)
        self.a3 = det_X / det_A; self.a4 = det_Y / det_A; self.a5 = det_Z / det_A

    def calc_point(self, t):
        return self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5*t**5
    def calc_first_derivative(self, t):
        return self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
    def calc_second_derivative(self, t):
        return 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3

def normalize_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def get_circle_state(angle):
    x = CENTER_X + RADIUS * math.cos(angle)
    y = CENTER_Y + RADIUS * math.sin(angle)
    h = normalize_angle(angle + math.pi / 2.0)
    return x, y, h

def calc_unwinding_clothoid(start_x, start_y, start_h, start_k, length, step_size):
    points = []
    steps = int(length / step_size)
    curr_x, curr_y, curr_h = start_x, start_y, start_h
    for i in range(steps + 1):
        s = i * step_size
        k = start_k * (1.0 - s / length)
        points.append({'s': s, 'x': curr_x, 'y': curr_y, 'h': curr_h})
        if i < steps:
            s_mid = s + step_size / 2.0
            k_mid = start_k * (1.0 - s_mid / length)
            d_h = k_mid * step_size
            curr_x += step_size * math.cos(curr_h + d_h/2.0)
            curr_y += step_size * math.sin(curr_h + d_h/2.0)
            curr_h += d_h
    return points

# ================= 3. 轨迹规划主程序 =================

def plan_trajectory():
    points = []
    current_time = 0.0
    
    # --- Step 0: 计算目标方向 ---
    dx = EXIT_P2_X - EXIT_P1_X
    dy = EXIT_P2_Y - EXIT_P1_Y
    EXIT_TARGET_H = math.atan2(dy, dx)
    print(f"1. 目标直道方向: {EXIT_TARGET_H:.4f} rad")
    
    # --- Step 1: 入环规划 (保持不变) ---
    print("2. 计算入环路径...")
    sx, sy, sh = START_X, START_Y, START_H
    svx = SPEED_MS * math.cos(sh); svy = SPEED_MS * math.sin(sh)
    best_cost = float('inf'); best_poly_x = None; best_poly_y = None; best_T = 0; best_entry_angle = 0
    search_angles = [i * 0.1 for i in range(0, 30)] 
    search_times = [i * 0.5 for i in range(5, 12)]
    
    for ang in search_angles:
        ex, ey, eh = get_circle_state(ang)
        evx, evy = SPEED_MS * math.cos(eh), SPEED_MS * math.sin(eh)
        eax, eay = (SPEED_MS**2/RADIUS)*math.cos(ang+math.pi), (SPEED_MS**2/RADIUS)*math.sin(ang+math.pi)
        for T in search_times:
            px = QuinticPolynomial(sx, svx, 0, ex, evx, eax, T)
            py = QuinticPolynomial(sy, svy, 0, ey, evy, eay, T)
            mid_x = px.calc_point(T/2); mid_y = py.calc_point(T/2)
            dist_path = math.sqrt((sx-mid_x)**2 + (sy-mid_y)**2) + math.sqrt((mid_x-ex)**2 + (mid_y-ey)**2)
            dist_direct = math.sqrt((sx-ex)**2 + (sy-ey)**2)
            if dist_path < dist_direct * 1.6:
                cost = 0
                for k in range(5):
                    t_samp = T*(k/5)
                    cost += px.calc_second_derivative(t_samp)**2 + py.calc_second_derivative(t_samp)**2
                if cost < best_cost:
                    best_cost = cost; best_poly_x = px; best_poly_y = py; best_T = T; best_entry_angle = ang
    
    steps = int(best_T / DT)
    for i in range(steps + 1):
        t = i * DT
        x = best_poly_x.calc_point(t); y = best_poly_y.calc_point(t)
        vx = best_poly_x.calc_first_derivative(t); vy = best_poly_y.calc_first_derivative(t)
        points.append({'t': current_time + t, 'x': x, 'y': y, 'h': math.atan2(vy, vx)})
    current_time += steps * DT
    
    # --- Step 2: 确定出环角度 (加入用户控制) ---
    k_circle = 1.0 / RADIUS
    delta_theta_spiral = 0.5 * k_circle * SPIRAL_LENGTH
    
    # 理论最佳切出角度
    ideal_exit_start_heading = normalize_angle(EXIT_TARGET_H - delta_theta_spiral)
    ideal_exit_circle_angle = normalize_angle(ideal_exit_start_heading - math.pi / 2.0)
    
    # === 加入用户控制的额外角度 ===
    extra_rad = math.radians(EXTRA_ROTATION_DEG)
    final_exit_circle_angle = ideal_exit_circle_angle + extra_rad # 逆时针增加角度
    
    # 确保逆时针逻辑
    while final_exit_circle_angle <= best_entry_angle + 0.1:
        final_exit_circle_angle += 2 * math.pi
        
    print(f"3. 环岛行驶至角度 {final_exit_circle_angle:.2f} rad (包含额外 {EXTRA_ROTATION_DEG} 度)")
    
    # 生成圆周段
    arc_len = (final_exit_circle_angle - best_entry_angle) * RADIUS
    circle_steps = int(arc_len / (SPEED_MS * DT))
    omega = SPEED_MS / RADIUS
    for i in range(1, circle_steps + 1):
        t_add = i * DT
        ang = best_entry_angle + omega * t_add
        x, y, h = get_circle_state(ang)
        points.append({'t': current_time + t_add, 'x': x, 'y': y, 'h': h})
    current_time += circle_steps * DT
    
    # --- Step 3: 回旋线解旋 ---
    print(f"4. 生成 {SPIRAL_LENGTH}m 回旋线...")
    last_p = points[-1]
    clothoid_points = calc_unwinding_clothoid(
        last_p['x'], last_p['y'], last_p['h'], 
        k_circle, SPIRAL_LENGTH, SPEED_MS * DT
    )
    for cp in clothoid_points[1:]:
        current_time += DT
        points.append({'t': current_time, 'x': cp['x'], 'y': cp['y'], 'h': cp['h']})
        
    # --- Step 4: 最终横向+航向修正 (Snapping) ---
    # 因为强制增加绕行角度，回旋线结束时车头肯定不对准直道了
    # 我们用五次多项式把车头和车身都拉回直道
    last_p = points[-1]
    
    dist_vec = math.sqrt(dx*dx + dy*dy)
    ux, uy = dx/dist_vec, dy/dist_vec
    
    # 投影修正逻辑
    v_x = last_p['x'] - EXIT_P1_X; v_y = last_p['y'] - EXIT_P1_Y
    proj_len = v_x * ux + v_y * uy
    # 在目标线上选一个足够远的点作为“对齐点”
    snap_dist = 40.0 # 距离给大一点，保证修正过程平滑
    final_x = EXIT_P1_X + (proj_len + snap_dist) * ux
    final_y = EXIT_P1_Y + (proj_len + snap_dist) * uy
    
    # 起点：回旋线末端
    sx, sy = last_p['x'], last_p['y']
    svx, svy = SPEED_MS * math.cos(last_p['h']), SPEED_MS * math.sin(last_p['h'])
    
    # 终点：完全对齐直道
    ex, ey = final_x, final_y
    evx, evy = SPEED_MS * ux, SPEED_MS * uy # 速度向量强制对齐目标方向
    
    snap_poly_x = QuinticPolynomial(sx, svx, 0, ex, evx, 0, snap_dist/SPEED_MS)
    snap_poly_y = QuinticPolynomial(sy, svy, 0, ey, evy, 0, snap_dist/SPEED_MS)
    
    print("5. 执行最终修正...")
    snap_steps = int(snap_dist / (SPEED_MS * DT))
    for i in range(1, snap_steps + 1):
        t = i * DT
        current_time += DT
        x = snap_poly_x.calc_point(t)
        y = snap_poly_y.calc_point(t)
        vx = snap_poly_x.calc_first_derivative(t)
        vy = snap_poly_y.calc_first_derivative(t)
        points.append({'t': current_time, 'x': x, 'y': y, 'h': math.atan2(vy, vx)})
        
    # Step 5: 补充纯直线
    for i in range(1, 20):
        t = current_time + i*DT
        points.append({'t': t, 'x': ex + i*DT*evx, 'y': ey + i*DT*evy, 'h': EXIT_TARGET_H})

    return points

# ================= 4. 输出 =================

vt1_path = plan_trajectory()

with open(OUTPUT_FILE, 'w') as f:
    for p in vt1_path:
        f.write(f'<Vertex time="{p["t"]:.4f}">\n')
        f.write(f'    <Position><WorldPosition x="{p["x"]:.4f}" y="{p["y"]:.4f}" z="0" h="{p["h"]:.4f}"/></Position>\n')
        f.write('</Vertex>\n')

print(f"\n[Success] 轨迹已生成，额外绕行: {EXTRA_ROTATION_DEG}度")

# 绘图
try:
    px = [p['x'] for p in vt1_path]; py = [p['y'] for p in vt1_path]
    plt.figure(figsize=(8,8))
    plt.plot(px, py, 'b.-', label='Path')
    circle = plt.Circle((CENTER_X, CENTER_Y), RADIUS, color='gray', fill=False, linestyle='--')
    plt.gca().add_patch(circle)
    plt.plot(START_X, START_Y, 'go', label='Start')
    # 目标直线
    plt.plot([EXIT_P1_X-50, EXIT_P2_X+50], [EXIT_P1_Y-50*(EXIT_P2_Y-EXIT_P1_Y)/(EXIT_P2_X-EXIT_P1_X), EXIT_P2_Y+50*(EXIT_P2_Y-EXIT_P1_Y)/(EXIT_P2_X-EXIT_P1_X)], 'r--', alpha=0.5, label='Target Line')
    plt.axis('equal'); plt.legend(); plt.grid(True)
    plt.title(f"Extra Rotation: {EXTRA_ROTATION_DEG} deg")
    plt.show()
except: pass