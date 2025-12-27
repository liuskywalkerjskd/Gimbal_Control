import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import time

# ================= 配置参数 =================
CONFIG = {
    'L_BY': 1.5,
    'L_YP': 1.5,
    'PITCH_LIMIT': 45.0,
    'SENSOR_SIZE': [0.6, 0.4, 0.3], 
    'WAVE_AMP': 15.0,    
    'WAVE_FREQ': 1.0,    
    'COLORS': {
        'bg': '#121212',       
        'panel_bg': '#1e1e1e', 
        'grid': '#333333',
        'text': '#e0e0e0',
        'text_dim': '#888888',
        'link': '#00a8cc',
        'link_limit': '#ff3333',
        'trail': '#ffe552',    
        'sensor': '#aaaaaa',
        'ghost': '#444444'
    }
}

# ================= 数学核心 (IK) =================
def solve_gimbal_angles_with_limit(R_target_world, R_base_world, limit_deg):
    R_local = R_base_world.T @ R_target_world
    target_vec = R_local[:, 0]
    x, y, z = target_vec[0], target_vec[1], target_vec[2]
    
    h = np.sqrt(x**2 + y**2)
    r2_raw = np.arctan2(-z, h)
    r1 = np.arctan2(y, x)
    
    limit_rad = np.radians(limit_deg)
    is_limited = False
    
    if r2_raw > limit_rad:
        r2 = limit_rad
        is_limited = True
    elif r2_raw < -limit_rad:
        r2 = -limit_rad
        is_limited = True
    else:
        r2 = r2_raw
        
    return r1, r2, is_limited

# ================= 绘图工具类 =================
class GeometryUtils:
    @staticmethod
    def get_cuboid(center, size, R_mat):
        l, w, h = size
        corners = np.array([
            [l/2, w/2, h/2], [l/2, w/2, -h/2], [l/2, -w/2, h/2], [l/2, -w/2, -h/2],
            [-l/2, w/2, h/2], [-l/2, w/2, -h/2], [-l/2, -w/2, h/2], [-l/2, -w/2, -h/2]
        ])
        corners_world = (R_mat @ corners.T).T + center
        faces = [
            [corners_world[0], corners_world[2], corners_world[6], corners_world[4]],
            [corners_world[1], corners_world[3], corners_world[7], corners_world[5]],
            [corners_world[0], corners_world[1], corners_world[5], corners_world[4]],
            [corners_world[2], corners_world[3], corners_world[7], corners_world[6]],
            [corners_world[0], corners_world[1], corners_world[3], corners_world[2]],
            [corners_world[4], corners_world[5], corners_world[7], corners_world[6]]
        ]
        return faces

    @staticmethod
    def draw_cylinder(ax, p0, p1, r=0.1, color='gray', alpha=1.0):
        v = p1 - p0
        mag = np.linalg.norm(v)
        if mag == 0: return
        v = v / mag
        not_v = np.array([1, 0, 0])
        if np.abs(np.dot(not_v, v)) > 0.99: not_v = np.array([0, 1, 0])
        n1 = np.cross(not_v, v); n1 /= np.linalg.norm(n1)
        n2 = np.cross(v, n1)
        t = np.linspace(0, mag, 2)
        theta = np.linspace(0, 2*np.pi, 8) 
        t_grid, theta_grid = np.meshgrid(t, theta)
        X, Y, Z = [p0[i] + v[i] * t_grid + r * np.sin(theta_grid) * n1[i] + r * np.cos(theta_grid) * n2[i] for i in [0, 1, 2]]
        ax.plot_surface(X, Y, Z, color=color, alpha=alpha, shade=True)

    @staticmethod
    def draw_frame(ax, origin, R_mat, scale=1.0, label=None, style='solid', text_offset=None):
        """绘制RGB坐标轴，支持自定义文字偏移"""
        colors = ['r', 'g', 'b']
        ls = '-' if style == 'solid' else '--'
        lw = 2 if style == 'solid' else 1
        alpha = 1.0 if style == 'solid' else 0.4
        
        for i in range(3):
            vec = R_mat[:, i] * scale
            ax.plot([origin[0], origin[0]+vec[0]], [origin[1], origin[1]+vec[1]], [origin[2], origin[2]+vec[2]], 
                    color=colors[i], linestyle=ls, linewidth=lw, alpha=alpha)
        
        if label:
            if text_offset is None:
                text_offset = np.array([0, 0, scale*1.1])
            t_pos = origin + text_offset
            
            # 给文字加个背景框，防止看不清
            ax.text(t_pos[0], t_pos[1], t_pos[2], label, color='white', fontsize=8, weight='bold',
                   bbox=dict(facecolor='black', alpha=0.5, edgecolor='none', pad=1))

# ================= 主程序类 =================
class GimbalVisualizer:
    def __init__(self):
        self.trail_points = []
        self.is_dragging = False
        self.is_animating = False
        self.start_time = 0
        
        # 1. 窗口初始化
        self.fig = plt.figure(figsize=(16, 9), facecolor=CONFIG['COLORS']['bg'])
        
        # 3D 视图 (左侧 70%)
        self.ax = self.fig.add_axes([0.0, 0.0, 0.70, 1.0], projection='3d')
        self.ax.set_facecolor(CONFIG['COLORS']['bg'])
        self.ax.view_init(elev=20, azim=45)
        
        # UI 面板 (右侧 30%)
        self.panel_ax = self.fig.add_axes([0.70, 0.0, 0.30, 1.0])
        self.panel_ax.set_facecolor(CONFIG['COLORS']['panel_bg'])
        self.panel_ax.axis('off')
        self.fig.add_artist(plt.Line2D([0.70, 0.70], [0, 1], color='#333333', lw=2))

        self.sliders = {}
        self.setup_ui()
        
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.anim = animation.FuncAnimation(self.fig, self.anim_loop, interval=50, blit=False)
        
        self.update_system()

    def setup_ui(self):
        """完全重构的 UI 布局，防止重叠"""
        c_text = CONFIG['COLORS']['text']
        c_dim = CONFIG['COLORS']['text_dim']
        
        # --- 顶部区域：标题 ---
        self.panel_ax.text(0.05, 0.96, "GIMBAL STUDIO", color='white', fontsize=20, weight='bold', transform=self.panel_ax.transAxes)
        self.panel_ax.text(0.05, 0.93, "High-Dynamic Simulation", color=c_dim, fontsize=10, transform=self.panel_ax.transAxes)

        # --- 中上区域：滑块控件 (从 y=0.85 开始往下排) ---
        
        current_y = 0.85
        spacing = 0.05
        
        # Group 1: Target
        self.panel_ax.text(0.05, current_y, "TARGET ATTITUDE", color='#00a8cc', fontsize=11, weight='bold', transform=self.panel_ax.transAxes)
        current_y -= 0.04
        
        for k in ['Yaw', 'Pitch', 'Roll']:
            # Label (Left)
            self.fig.text(0.72, current_y + 0.005, k, color=c_text, fontsize=9, weight='bold')
            # Slider (Middle)
            ax_s = self.fig.add_axes([0.76, current_y, 0.15, 0.02])
            limit = 180 if k in ['Yaw', 'Roll'] else 90
            s = Slider(ax_s, "", -limit, limit, valinit=0, color='#00a8cc', alpha=0.7)
            s.valtext.set_visible(False) 
            # Value Display (Right)
            val_disp = self.fig.text(0.93, current_y + 0.005, "0.0°", color='white', fontsize=9, ha='left')
            
            s.on_changed(lambda val, disp=val_disp: self.on_slider_change(val, disp))
            self.sliders[f"t_{k}"] = s
            current_y -= spacing

        # Group 2: Base
        current_y -= 0.02 # Extra gap
        self.panel_ax.text(0.05, current_y, "BASE ATTITUDE", color='#cc8800', fontsize=11, weight='bold', transform=self.panel_ax.transAxes)
        current_y -= 0.04
        
        for k in ['Yaw', 'Pitch', 'Roll']:
            self.fig.text(0.72, current_y + 0.005, k, color=c_text, fontsize=9, weight='bold')
            ax_s = self.fig.add_axes([0.76, current_y, 0.15, 0.02])
            limit = 180 if k in ['Yaw', 'Roll'] else 90
            s = Slider(ax_s, "", -limit, limit, valinit=0, color='#cc8800', alpha=0.7)
            s.valtext.set_visible(False)
            val_disp = self.fig.text(0.93, current_y + 0.005, "0.0°", color='white', fontsize=9, ha='left')
            
            s.on_changed(lambda val, disp=val_disp: self.on_slider_change(val, disp))
            self.sliders[f"b_{k}"] = s
            current_y -= spacing

        # --- 中部区域：按钮 ---
        current_y -= 0.02
        btn_height = 0.04
        ax_btn_reset = self.fig.add_axes([0.72, current_y, 0.10, btn_height])
        self.btn_reset = Button(ax_btn_reset, 'RESET', color='#333333', hovercolor='#555555')
        self.btn_reset.label.set_color('white')
        self.btn_reset.on_clicked(self.reset_all)

        ax_btn_wave = self.fig.add_axes([0.84, current_y, 0.10, btn_height])
        self.btn_wave = Button(ax_btn_wave, 'SINE WAVE', color='#333333', hovercolor='#555555')
        self.btn_wave.label.set_color(CONFIG['COLORS']['link'])
        self.btn_wave.on_clicked(self.toggle_wave_mode)
        
        self.wave_info = self.fig.text(0.95, current_y + 0.015, "OFF", color='gray', fontsize=8, ha='left')

        # --- 底部区域：数据面板 (独立区域，绝不重叠) ---
        self.info_status = self.panel_ax.text(0.05, 0.25, "SYSTEM OPTIMAL", color='#00ff00', fontsize=14, weight='bold', transform=self.panel_ax.transAxes)
        
        # 放置详细数据
        self.info_metrics = self.panel_ax.text(0.05, 0.02, "", color=c_text, fontsize=9, fontfamily='monospace', va='bottom', linespacing=1.6, transform=self.panel_ax.transAxes)

    # ================= 交互逻辑 =================
    def on_slider_change(self, val, val_disp_obj):
        val_disp_obj.set_text(f"{val:.1f}°")
        self.is_dragging = True
        self.update_system()
        
    def on_release(self, event):
        self.is_dragging = False
        self.trail_points = []
        if not self.is_animating:
            self.update_system()

    def reset_all(self, event):
        self.is_animating = False
        self.btn_wave.label.set_text("SINE WAVE")
        self.wave_info.set_text("OFF")
        for s in self.sliders.values(): s.set_val(0)
        self.trail_points = []
        self.update_system()

    def toggle_wave_mode(self, event):
        self.is_animating = not self.is_animating
        if self.is_animating:
            self.start_time = time.time()
            self.btn_wave.color = '#004400'
            self.wave_info.set_text(f"ON")
            self.wave_info.set_color('#00ff00')
        else:
            self.btn_wave.color = '#333333'
            self.wave_info.set_text("OFF")
            self.wave_info.set_color('gray')
            self.update_system()

    def anim_loop(self, frame):
        if self.is_animating:
            self.update_system()

    # ================= 核心渲染 =================
    def update_system(self):
        self.ax.cla()
        
        # 数据获取与干扰
        ty, tp, tr = [np.radians(self.sliders[k].val) for k in ['t_Yaw', 't_Pitch', 't_Roll']]
        by_s, bp_s, br_s = [self.sliders[k].val for k in ['b_Yaw', 'b_Pitch', 'b_Roll']]
        
        if self.is_animating:
            t = time.time() - self.start_time
            offset = CONFIG['WAVE_AMP'] * np.sin(2 * np.pi * CONFIG['WAVE_FREQ'] * t)
            by = np.radians(by_s + offset)
            bp = np.radians(bp_s + offset * 0.7)
            br = np.radians(br_s + offset * 0.5)
        else:
            by, bp, br = np.radians([by_s, bp_s, br_s])

        # FK / IK 计算
        R_base = R.from_euler('ZYX', [by, bp, br]).as_matrix()
        R_target = R.from_euler('ZYX', [ty, tp, tr]).as_matrix()
        
        # 坐标定义：把 World 放在更低的位置 (-3,-3,-3)
        P_world_origin = np.array([-2.0, -2.0, -3.0]) 
        P_base = np.array([0, 0, 0])

        j_yaw, j_pit, is_limited = solve_gimbal_angles_with_limit(R_target, R_base, CONFIG['PITCH_LIMIT'])

        P_Y = P_base + R_base @ np.array([0, 0, CONFIG['L_BY']])
        R_Y = R_base @ R.from_euler('z', j_yaw).as_matrix()
        P_P = P_Y + R_Y @ np.array([0, CONFIG['L_YP'], 0])
        R_P = R_Y @ R.from_euler('y', j_pit).as_matrix()

        # 轨迹处理
        if self.is_dragging or self.is_animating:
            self.trail_points.append(P_P)
            if len(self.trail_points) > 50: 
                self.trail_points.pop(0)

        # --- 绘图 ---
        self.ax.set_xlim([-4, 4]); self.ax.set_ylim([-4, 4]); self.ax.set_zlim([-4, 4])
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.ax.grid(False); self.ax.axis('off')
        
        # 1. 绘制 World Floor Grid & Origin Frame (恢复左侧信息)
        xx, yy = np.meshgrid(np.arange(-4, 5, 2), np.arange(-4, 5, 2))
        self.ax.plot_wireframe(xx, yy, np.full_like(xx, -3), color=CONFIG['COLORS']['grid'], alpha=0.2)
        
        # 明确画出 World Frame
        GeometryUtils.draw_frame(self.ax, P_world_origin, np.eye(3), scale=1.0, label="WORLD", text_offset=[0,0,1.2])
        # 画一条线连到 Base，体现相对位置
        self.ax.plot([P_world_origin[0], P_base[0]], [P_world_origin[1], P_base[1]], [P_world_origin[2], P_base[2]], 
                     color='gray', linestyle=':', alpha=0.5)

        # 2. 轨迹
        if len(self.trail_points) > 1:
            pts = np.array(self.trail_points)
            self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color=CONFIG['COLORS']['trail'], lw=2, alpha=0.8)

        # 3. Base Frame & Link
        GeometryUtils.draw_frame(self.ax, P_base, R_base, scale=0.8, label="BASE", text_offset=[0,0,-1.0])
        GeometryUtils.draw_cylinder(self.ax, P_base, P_Y, r=0.1, color='#444444')
        
        # 4. Yaw Joint Frame (中间关节)
        GeometryUtils.draw_frame(self.ax, P_Y, R_Y, scale=0.6, label="Yaw_J", style='--')
        
        # 5. Sensor/End Effector
        link_col = CONFIG['COLORS']['link_limit'] if is_limited else CONFIG['COLORS']['link']
        GeometryUtils.draw_cylinder(self.ax, P_Y, P_P, r=0.08, color=link_col)
        
        poly = Poly3DCollection(GeometryUtils.get_cuboid(P_P, CONFIG['SENSOR_SIZE'], R_P), 
                                facecolors=link_col, edgecolors='k', alpha=0.9)
        self.ax.add_collection3d(poly)
        GeometryUtils.draw_frame(self.ax, P_P, R_P, scale=1.2, label="SENSOR") # 恢复 Label

        # 6. Target Ghost
        poly_ghost = Poly3DCollection(GeometryUtils.get_cuboid(P_P, CONFIG['SENSOR_SIZE'], R_target), 
                                      facecolors=CONFIG['COLORS']['ghost'], edgecolors='white', alpha=0.15, linewidths=0.5)
        self.ax.add_collection3d(poly_ghost)
        
        # 7. Beam
        beam_end = P_P + R_P[:, 0] * 4.0
        self.ax.plot([P_P[0], beam_end[0]], [P_P[1], beam_end[1]], [P_P[2], beam_end[2]], 
                     color='red' if is_limited else 'cyan', lw=1.5, alpha=0.6)

        # --- 更新面板信息 (底部区域) ---
        vec_act = R_P[:, 0]; vec_tar = R_target[:, 0]
        err_deg = np.degrees(np.arccos(np.clip(np.dot(vec_act, vec_tar), -1, 1)))
        
        if is_limited:
            self.info_status.set_text("⚠ LIMIT HIT")
            self.info_status.set_color('#ff3333')
        else:
            self.info_status.set_text("✓ SYSTEM OPTIMAL")
            self.info_status.set_color('#00ff00')
            
        info_str = (
            f"ERROR : {err_deg:6.2f}°\n"
            f"------------------\n"
            f"JOINT STATE:\n"
            f"Yaw   : {np.degrees(j_yaw):6.1f}°\n"
            f"Pitch : {np.degrees(j_pit):6.1f}°\n"
            f"------------------\n"
            f"BASE (LIVE):\n"
            f"Y: {np.degrees(by):5.1f}°\n"
            f"P: {np.degrees(bp):5.1f}°\n"
            f"R: {np.degrees(br):5.1f}°"
        )
        self.info_metrics.set_text(info_str)

if __name__ == "__main__":
    app = GimbalVisualizer()
    plt.show()