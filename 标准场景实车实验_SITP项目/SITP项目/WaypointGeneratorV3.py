import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import os

class CollisionScenario:
    def __init__(self):
        self.v1 = 0.0      # Target vehicle speed (m/s)
        self.v2 = 0.0      # Target pedestrian speed (m/s)
        self.ttc = 3.5     # Time-to-collision from trigger points (s)
        self.a_vehicle = 3.47   # Vehicle acceleration (m/s^2)
        self.a_pedestrian = 2.0 # Pedestrian acceleration (m/s^2)
        self.post_collision_distance = 2.5  # Pedestrian walks 2.5m after collision

    def validate_inputs(self):
        if not (3.5 <= self.ttc <= 4.5):
            raise ValueError("TTC must be between 3.5 and 4.5 seconds")
        if self.v2 > 5.0:
            raise ValueError("Pedestrian speed must not exceed 5 m/s")
        if self.v1 <= 0 or self.v2 <= 0:
            raise ValueError("Speeds must be greater than 0")
        d_ac = self.v2 * self.ttc
        if d_ac > 5.25:
            raise ValueError(f"Distance from A to C = {d_ac:.2f} m exceeds 5.25 m limit")

    def calculate_positions(self):
        self.validate_inputs()

        cx, cy = 0.0, 0.0  # Collision point C

        # Vehicle: B to C = v1 * ttc
        d_bc = self.v1 * self.ttc
        bx, by = cx - d_bc, cy

        # Vehicle start F: accelerate from rest to v1 with a=3.47
        t_acc_v = self.v1 / self.a_vehicle
        d_fb = 0.5 * self.a_vehicle * t_acc_v**2  # = v1^2 / (2a)
        fx, fy = bx - d_fb, cy

        # Pedestrian: A to C = v2 * ttc
        d_ac = self.v2 * self.ttc
        ax, ay = cx, cy + d_ac

        # Pedestrian start E: accelerate from rest to v2 over distance d_ea
        t_acc_p = self.v2 / self.a_pedestrian
        d_ea = 0.5 * self.a_pedestrian * t_acc_p**2  # = v2^2 / (2a)
        ex, ey = cx, ay + d_ea

        return {
            'vehicle': {'start': (fx, fy), 'trigger': (bx, by), 'collision': (cx, cy)},
            'pedestrian': {'start': (ex, ey), 'trigger': (ax, ay), 'collision': (cx, cy)}
        }

    def generate_trajectory_points(self, num_points=200):
        self.validate_inputs()
        pos = self.calculate_positions()

        # === Vehicle trajectory: acceleration + constant speed ===
        t_acc_v = self.v1 / self.a_vehicle
        d_fb = self.v1**2 / (2 * self.a_vehicle)
        d_bc = self.v1 * self.ttc
        total_vehicle_time = t_acc_v + self.ttc
        t_vals_v = np.linspace(0, total_vehicle_time, num_points)

        vehicle_x = []
        vehicle_y = []
        for t in t_vals_v:
            if t <= t_acc_v:
                x = pos['vehicle']['start'][0] + 0.5 * self.a_vehicle * t**2
            else:
                x_accel_end = pos['vehicle']['start'][0] + d_fb
                x = x_accel_end + self.v1 * (t - t_acc_v)
            vehicle_x.append(x)
            vehicle_y.append(0.0)

        # === Pedestrian trajectory: acceleration + constant speed to C + post-collision walk ===
        t_acc_p = self.v2 / self.a_pedestrian
        d_ea = self.v2**2 / (2 * self.a_pedestrian)
        d_ac = self.v2 * self.ttc
        t_post = self.post_collision_distance / self.v2  # time to walk 2.5m after C

        total_ped_time = t_acc_p + self.ttc + t_post
        t_vals_p = np.linspace(0, total_ped_time, num_points)

        pedestrian_x = []
        pedestrian_y = []
        for t in t_vals_p:
            if t <= t_acc_p:
                # Accelerating from E to A
                y = pos['pedestrian']['start'][1] - 0.5 * self.a_pedestrian * t**2
            elif t <= t_acc_p + self.ttc:
                # Constant speed from A to C
                y = d_ac - self.v2 * (t - t_acc_p)
            else:
                # After collision: continue downward (negative y) for 2.5m
                y = -self.v2 * (t - t_acc_p - self.ttc)
                if y < -self.post_collision_distance:
                    y = -self.post_collision_distance
            pedestrian_x.append(0.0)
            pedestrian_y.append(y)

        return {
            'vehicle': (vehicle_x, vehicle_y),
            'pedestrian': (pedestrian_x, pedestrian_y),
            'vehicle_time': list(t_vals_v),
            'pedestrian_time': list(t_vals_p)
        }

    def save_trajectories_to_txt(self, vehicle_path='vehicle_trajectory.txt', pedestrian_path='pedestrian_trajectory.txt', num_points=20):
        self.validate_inputs()
        data = self.generate_trajectory_points(num_points=num_points)

        def write_vertices(path, xs, ys, ts, heading_deg):
            with open(path, 'w', encoding='utf-8') as f:
                for x, y, t in zip(xs, ys, ts):
                    f.write(f'<Vertex time="{t:.6f}">\n')
                    f.write('    <Position>\n')
                    f.write(f'        <WorldPosition x="{x:.6f}" y="{y:.6f}" z="0" h="{heading_deg:.6f}" p="0" r="0"/>\n')
                    f.write('    </Position>\n')
                    f.write('</Vertex>\n')

        # Vehicle: heading 0 degrees
        vx, vy = data['vehicle']
        vt = data['vehicle_time']
        write_vertices(vehicle_path, vx, vy, vt, heading_deg=0.0)

        # Pedestrian: heading 180 degrees
        px, py = data['pedestrian']
        pt = data['pedestrian_time']
        write_vertices(pedestrian_path, px, py, pt, heading_deg=180.0)

    def plot_scenario(self):
        try:
            traj = self.generate_trajectory_points()
            pos = self.calculate_positions()

            fig, ax = plt.subplots(figsize=(10, 8))

            # Road with two lanes (horizontal, centered on y=0)
            lane_width = 3.5
            lanes_count = 2
            road_width = lane_width * lanes_count
            road_left = pos['vehicle']['start'][0] - 30.0  # start 30m left of F point
            road_right = 60.0
            # Road background
            ax.add_patch(Rectangle((road_left, -road_width/2), road_right - road_left, road_width, facecolor='lightgray', edgecolor='black'))
            # Lane borders
            ax.plot([road_left, road_right], [-road_width/2, -road_width/2], 'k-', linewidth=1)
            ax.plot([road_left, road_right], [road_width/2, road_width/2], 'k-', linewidth=1)
            # Center dashed line between two lanes
            dash_x = np.linspace(road_left, road_right, 40)
            for i in range(0, len(dash_x)-1, 2):
                ax.plot([dash_x[i], dash_x[i+1]], [0, 0], color='white', linewidth=2)

            # Crosswalk at x=0
            for y in np.arange(-1.5, 1.6, 0.5):
                ax.plot([0, 0], [y - 0.25, y + 0.25], 'k-', linewidth=3)

            # Trajectories
            ax.plot(traj['vehicle'][0], traj['vehicle'][1], 'b-', linewidth=2, label='Vehicle Trajectory')
            ax.plot(traj['pedestrian'][0], traj['pedestrian'][1], 'r-', linewidth=2, label='Pedestrian Trajectory')

            # Key points
            ax.plot(*pos['vehicle']['start'], 'go', markersize=8, label='Vehicle Start (F)')
            ax.plot(*pos['vehicle']['trigger'], 'bo', markersize=8, label='Vehicle Trigger (B)')
            ax.plot(*pos['pedestrian']['start'], 'mo', markersize=8, label='Pedestrian Start (E)')
            ax.plot(*pos['pedestrian']['trigger'], 'ro', markersize=8, label='Pedestrian Trigger (A)')
            ax.plot(0, 0, 'ko', markersize=9, label='Collision Point (C)')

            # Icons: more realistic car & pedestrian drawings
            def draw_car(ax, center, length=4.2, width=1.8, body_color='#1f77b4', angle_deg=0.0):
                x, y = center
                # Car body as rotated rectangle
                car = Rectangle((x - length/2, y - width/2), length, width, angle=angle_deg,
                                facecolor=body_color, edgecolor='black', linewidth=1.5, alpha=0.95)
                ax.add_patch(car)
                # Wheels (black circles) positioned relative to body
                wheel_radius = width * 0.18
                # Offsets along length for front/back wheels
                offsets = [(-length*0.35, 0), (length*0.35, 0)]
                for dx, dy in offsets:
                    ax.add_patch(plt.Circle((x + dx, y - width/2 + wheel_radius), wheel_radius, color='black'))
                    ax.add_patch(plt.Circle((x + dx, y + width/2 - wheel_radius), wheel_radius, color='black'))
                # Windows (lighter rectangles)
                win_len = length * 0.5
                win_wid = width * 0.5
                window = Rectangle((x - win_len/2, y - win_wid/2), win_len, win_wid, angle=angle_deg,
                                   facecolor='#a6c8ff', edgecolor='black', linewidth=1)
                ax.add_patch(window)

            def draw_crashed_car(ax, center):
                # Slight rotation and different color to indicate collision
                draw_car(ax, center, body_color='#ff7f0e', angle_deg=25)
                # Add impact star
                cx, cy = center
                star_r = 0.8
                angles = np.linspace(0, 2*np.pi, 9)
                for i in range(len(angles)-1):
                    ax.plot([cx, cx + star_r*np.cos(angles[i])], [cy, cy + star_r*np.sin(angles[i])], color='#ff7f0e', linewidth=2)

            def draw_pedestrian(ax, center, height=1.7, color='#d62728'):
                x, y = center
                head_r = height * 0.12
                # Head
                ax.add_patch(plt.Circle((x, y + height*0.4), head_r, color=color, ec='black'))
                # Torso
                ax.plot([x, x], [y + height*0.3, y - height*0.2], color=color, linewidth=3)
                # Arms
                ax.plot([x - head_r*2.5, x + head_r*2.5], [y + height*0.15, y + height*0.05], color=color, linewidth=3)
                # Legs
                ax.plot([x, x - head_r*2], [y - height*0.2, y - height*0.5], color=color, linewidth=3)
                ax.plot([x, x + head_r*2], [y - height*0.2, y - height*0.5], color=color, linewidth=3)

            # Car heading along +x at start F
            draw_car(ax, pos['vehicle']['start'], angle_deg=0)
            # Crashed car at collision C
            draw_crashed_car(ax, (0, 0))
            # Pedestrian at start E
            draw_pedestrian(ax, pos['pedestrian']['start'])

            # Annotations
            for name, pt in [
                ('F', pos['vehicle']['start']),
                ('B', pos['vehicle']['trigger']),
                ('E', pos['pedestrian']['start']),
                ('A', pos['pedestrian']['trigger']),
                ('C', (0, 0))
            ]:
                ax.annotate(name, pt, xytext=(5, 5), textcoords='offset points', fontsize=9, fontweight='bold')

            # Adjust axis to show full vehicle path
            min_x = min(traj['vehicle'][0]) - 5
            max_x = max(traj['vehicle'][0]) + 5
            ax.set_xlim(min_x, 5)  # Show full left side, cut right after C
            ax.set_ylim(-3.5, max(traj['pedestrian'][1]) + 2)
            ax.set_aspect('auto')
            ax.grid(True, linestyle='--', alpha=0.6)
            # Place legend outside on the far left to avoid occlusion
            ax.legend(loc='center left', bbox_to_anchor=(-0.22, 0.5))
            ax.set_title("Vehicle-Pedestrian Collision Scenario (with Acceleration & Post-Collision Walk)")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")

            # Leave extra left margin for the outside legend
            fig.tight_layout(rect=[0.20, 0.05, 0.98, 0.95])

            # Show in a new window
            plt.show()

        except Exception as e:
            messagebox.showerror("Plot Error", str(e))
            return


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Vehicle-Pedestrian Collision Simulator")
        self.root.geometry("550x520")

        self.scenario = CollisionScenario()

        # UI
        tk.Label(root, text="Vehicle target speed v1 (m/s):").grid(row=0, column=0, padx=10, pady=8, sticky='w')
        self.entry_v1 = tk.Entry(root)
        self.entry_v1.grid(row=0, column=1, padx=10, pady=8)
        self.entry_v1.insert(0, "13.8")

        tk.Label(root, text="Pedestrian target speed v2 (m/s):").grid(row=1, column=0, padx=10, pady=8, sticky='w')
        self.entry_v2 = tk.Entry(root)
        self.entry_v2.grid(row=1, column=1, padx=10, pady=8)
        self.entry_v2.insert(0, "1.2")

        tk.Label(root, text="TTC (seconds, 3.5–4.5):").grid(row=2, column=0, padx=10, pady=8, sticky='w')
        self.entry_ttc = tk.Entry(root)
        self.entry_ttc.grid(row=2, column=1, padx=10, pady=8)
        self.entry_ttc.insert(0, "3.5")

        self.btn_calc = tk.Button(root, text="Calculate Start & Trigger Points", command=self.calculate_and_show)
        self.btn_calc.grid(row=3, column=0, columnspan=2, pady=12)

        self.btn_plot = tk.Button(root, text="Generate and Plot Trajectories", command=self.plot_trajectory)
        self.btn_plot.grid(row=4, column=0, columnspan=2, pady=12)

        self.result_text = tk.Text(root, height=11, width=70)
        self.result_text.grid(row=5, column=0, columnspan=2, padx=10, pady=10)

        # Save trajectories button
        self.btn_save = tk.Button(root, text="Save Trajectories to TXT (20 pts)", command=self.save_trajectories_txt)
        self.btn_save.grid(row=6, column=0, columnspan=2, pady=10)

    def get_inputs(self):
        self.scenario.v1 = float(self.entry_v1.get())
        self.scenario.v2 = float(self.entry_v2.get())
        self.scenario.ttc = float(self.entry_ttc.get())

    def calculate_and_show(self):
        try:
            self.get_inputs()
            pos = self.scenario.calculate_positions()

            d_bc = self.scenario.v1 * self.scenario.ttc
            d_ac = self.scenario.v2 * self.scenario.ttc
            d_fb = self.scenario.v1**2 / (2 * self.scenario.a_vehicle)
            d_ea = self.scenario.v2**2 / (2 * self.scenario.a_pedestrian)

            info = (
                f"[Parameters]\n"
                f"  TTC = {self.scenario.ttc:.2f} s\n"
                f"  Vehicle speed v1 = {self.scenario.v1:.2f} m/s\n"
                f"  Pedestrian speed v2 = {self.scenario.v2:.2f} m/s\n\n"
                f"[Validation]\n"
                f"  A→C = v2 × TTC = {d_ac:.2f} m (≤ 5.25 m: {'✓' if d_ac <= 5.25 else '✗'})\n"
                f"  B→C = v1 × TTC = {d_bc:.2f} m\n\n"
                f"[Accelerated Segments]\n"
                f"  Vehicle F→B = v1²/(2a_v) = {d_fb:.2f} m (a_v = {self.scenario.a_vehicle} m/s²)\n"
                f"  Pedestrian E→A = v2²/(2a_p) = {d_ea:.2f} m (a_p = {self.scenario.a_pedestrian} m/s²)\n\n"
                f"[Coordinates (C at origin)]\n"
                f"  Vehicle Start F: ({pos['vehicle']['start'][0]:.2f}, {pos['vehicle']['start'][1]:.2f})\n"
                f"  Vehicle Trigger B: ({pos['vehicle']['trigger'][0]:.2f}, {pos['vehicle']['trigger'][1]:.2f})\n"
                f"  Pedestrian Start E: ({pos['pedestrian']['start'][0]:.2f}, {pos['pedestrian']['start'][1]:.2f})\n"
                f"  Pedestrian Trigger A: ({pos['pedestrian']['trigger'][0]:.2f}, {pos['pedestrian']['trigger'][1]:.2f})\n"
                f"  Collision Point C: (0.00, 0.00)\n"
            )
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, info)

        except Exception as e:
            messagebox.showerror("Calculation Error", str(e))

    def plot_trajectory(self):
        try:
            self.get_inputs()
            self.scenario.plot_scenario()
        except Exception as e:
            messagebox.showerror("Plot Error", str(e))

    def save_trajectories_txt(self):
        try:
            self.get_inputs()
            base_dir = os.path.dirname(__file__)
            vehicle_path = os.path.join(base_dir, 'vehicle_trajectory.txt')
            pedestrian_path = os.path.join(base_dir, 'pedestrian_trajectory.txt')
            self.scenario.save_trajectories_to_txt(vehicle_path, pedestrian_path, num_points=20)
            messagebox.showinfo("保存成功", f"已保存：\n车辆: {vehicle_path}\n行人: {pedestrian_path}")
        except Exception as e:
            # Log detailed error info to log.txt alongside script
            try:
                log_path = os.path.join(os.path.dirname(__file__), 'log.txt')
                with open(log_path, 'a', encoding='utf-8') as logf:
                    logf.write(f"[SaveError] {str(e)}\n")
            except Exception:
                pass
            messagebox.showerror("保存错误", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()