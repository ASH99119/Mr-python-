import socket
import time
import threading
import numpy as np
from pynput import keyboard

QUEST_IP = "172.20.10.9"
LINE_PORT = 50001
SPEED_PORT = 50002
AUTO_PITCH_PORT = 50007  # ✨ 新增：用于接收 cast_final.py 算出的自动角度

# 状态变量
current_number_input = ""
auto_pitch_deg = 0.0     # ✨ 新增：存放接收到的自动修正角度

# 独立出初始默认值
DEFAULT_LEFT = {"X_max": 40.0, "end_y": -3.5, "pitch": -6.0}
DEFAULT_RIGHT = {"X_max": 40.0, "end_y": 3.5, "pitch": -6.0}

# 运行时动态微调变量
left_lane_config = DEFAULT_LEFT.copy()
right_lane_config = DEFAULT_RIGHT.copy()

last_active_direction = 'left'

# 微调步长设置
LENGTH_STEP = 2.0   # 纵向每次增减 2 米
LATERAL_STEP = 0.2  # 横向每次增减 0.2 米
PITCH_STEP = 1.0    # 角度每次增减 1 度

# ================= 1. UDP 接收自动角度后台线程 =================
def receive_auto_pitch():
    global auto_pitch_deg
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", AUTO_PITCH_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            auto_pitch_deg = float(data.decode('utf-8').strip())
        except:
            pass

# ================= 2. 0.5秒定时刷新状态看板线程 =================
def print_status_loop():
    while True:
        config = left_lane_config if last_active_direction == 'left' else right_lane_config
        x = config["X_max"]
        y = config["end_y"]
        manual_p = config["pitch"]
        total_p = manual_p + auto_pitch_deg
        
        dir_str = "左道" if last_active_direction == 'left' else "右道"
        
        # 仪表盘 UI 单行刷新
        if current_number_input == "":
            print(f"\r[{dir_str}监控] 长度:{x:.1f}m | 横移:{y:.2f}m | 手动基准:{manual_p:.1f}° + 自动补偿:{auto_pitch_deg:.2f}° = 最终仰角:{total_p:.2f}°      ", end="", flush=True)
        time.sleep(0.5)

# 五次多项式实时规划器（带 3D 旋转变换）
class TrajectoryPlanner:
    def __init__(self, num_points=80):
        self.N = num_points
        
    def generate(self, start_y, end_y, X_max, pitch_deg):
        X_max = max(5.0, X_max)

        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, X_max, X_max**2, X_max**3, X_max**4, X_max**5],
            [0, 1, 2*X_max, 3*X_max**2, 4*X_max**3, 5*X_max**4],
            [0, 0, 2, 6*X_max, 12 * X_max**2, 20 * X_max**3]
        ])
        B = np.array([start_y, 0, 0, end_y, 0, 0])
        coeffs = np.linalg.solve(A, B)

        x_vals = np.linspace(0, X_max, self.N)
        y_vals = (coeffs[0] + coeffs[1]*x_vals + coeffs[2]*x_vals**2 +
                  coeffs[3]*x_vals**3 + coeffs[4]*x_vals**4 + coeffs[5]*x_vals**5)

        pitch_rad = np.radians(pitch_deg)
        cos_p = np.cos(pitch_rad)
        sin_p = np.sin(pitch_rad)

        final_points = []
        for x, y in zip(x_vals, y_vals):
            raw_x = y       # 横向 (Unity X)
            raw_y = 0     # 高度 (Unity Y)
            raw_z = x       # 纵向 (Unity Z)

            new_x = raw_x
            new_y = raw_y * cos_p + raw_z * sin_p
            new_z = -raw_y * sin_p + raw_z * cos_p

            final_points.append([round(new_x, 2), round(new_y, 2), round(new_z, 2)])

        return final_points

planner = TrajectoryPlanner(num_points=80)

# 规划并实时分发数据
def plan_and_send(direction):
    global last_active_direction
    last_active_direction = direction
    
    config = left_lane_config if direction == 'left' else right_lane_config
    X_max = config["X_max"]
    end_y = config["end_y"]
    pitch = config["pitch"]

    start_y = 0.0
    trajectory_points = planner.generate(start_y, end_y, X_max, pitch)
    trajectory_data = "\n".join([f"{p[0]},{p[1]},{p[2]}" for p in trajectory_points])

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.connect((QUEST_IP, LINE_PORT))
        s.sendall(trajectory_data.encode('utf-8'))
        s.close()
    except Exception as e:
        pass # 静默处理错误，交给0.5秒循环去显示正常状态

# 发送速度数据
def send_speed(speed_val_str):
    speed_data = f"{speed_val_str}km/h"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.connect((QUEST_IP, SPEED_PORT))
        s.sendall(speed_data.encode('utf-8'))
        s.close()
    except Exception as e:
        pass

# 键盘监听
def on_press(key):
    global current_number_input, left_lane_config, right_lane_config
    try:
        if hasattr(key, 'char') and key.char is not None:
            key_char = key.char.lower()

            # --- 左车道 ---
            if key_char == 'w': left_lane_config["X_max"] += LENGTH_STEP; plan_and_send('left'); return
            elif key_char == 's': left_lane_config["X_max"] -= LENGTH_STEP; plan_and_send('left'); return
            elif key_char == 'a': left_lane_config["end_y"] -= LATERAL_STEP; plan_and_send('left'); return
            elif key_char == 'd': left_lane_config["end_y"] += LATERAL_STEP; plan_and_send('left'); return
            elif key_char == 'q': left_lane_config["pitch"] += PITCH_STEP; plan_and_send('left'); return
            elif key_char == 'e': left_lane_config["pitch"] -= PITCH_STEP; plan_and_send('left'); return

            # --- 右车道 ---
            elif key_char == 'i': right_lane_config["X_max"] += LENGTH_STEP; plan_and_send('right'); return
            elif key_char == 'k': right_lane_config["X_max"] -= LENGTH_STEP; plan_and_send('right'); return
            elif key_char == 'j': right_lane_config["end_y"] -= LATERAL_STEP; plan_and_send('right'); return
            elif key_char == 'l': right_lane_config["end_y"] += LATERAL_STEP; plan_and_send('right'); return
            elif key_char == 'u': right_lane_config["pitch"] += PITCH_STEP; plan_and_send('right'); return
            elif key_char == 'o': right_lane_config["pitch"] -= PITCH_STEP; plan_and_send('right'); return
            
            # --- 重置 ---
            elif key_char == 'r':
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0)
                    s.connect((QUEST_IP, LINE_PORT))
                    s.sendall(b"RESET")
                    s.close()
                except: pass
                return
      
        if key == keyboard.Key.space:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((QUEST_IP, LINE_PORT))
                s.sendall(b"TOGGLE_NAIL")
                s.close()
            except: pass
            return

        # 拦截小键盘
        elif hasattr(key, 'vk') and 96 <= key.vk <= 105:
            current_number_input += str(key.vk - 96)
            print(f"\r[正在输入车速...] {current_number_input} km/h                   ", end="", flush=True)
            return
        
        elif key == keyboard.Key.up:
            plan_and_send(last_active_direction)
            return

        elif key == keyboard.Key.down:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((QUEST_IP, LINE_PORT))
                s.sendall(b"CLEAR")
                s.close()
            except: pass
            return
            
        elif key == keyboard.Key.enter:
            if current_number_input:
                send_speed(current_number_input)
                current_number_input = ""
                plan_and_send(last_active_direction)
            return
            
        elif key == keyboard.Key.backspace:
            if len(current_number_input) > 0:
                current_number_input = current_number_input[:-1]
                print(f"\r[正在输入车速...] {current_number_input} km/h                   ", end="", flush=True)
            return

    except Exception as e:
        pass

# 主服务启动
print(f"🚗 实车路测主控端已启动 | 目标IP: {QUEST_IP}")
print("   W/S/A/D/Q/E : 左车道控制")
print("   I/K/J/L/U/O : 右车道控制")
print("   Space:钉地 | R:对齐前方 | ↓:清屏")
print("-" * 60)

# 启动后台服务线程
threading.Thread(target=receive_auto_pitch, daemon=True).start()
threading.Thread(target=print_status_loop, daemon=True).start()

# 保持主线程监听按键
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()