import cv2
import time
import threading
import socket
import numpy as np
import win32gui
import win32con
import ctypes
import math
from ctypes import wintypes
from ultralytics import YOLO

# ==================== 网络配置 ====================
QUEST_IP = "172.20.10.9"
LINE_PORT = 50001
DEPTH_PORT = 5006
UDP_PITCH_PORT = 50007  # 本地键盘脚本接收端口


# ----------------- UDP 实时深度接收线程 -----------------
class RealTimeDepthReceiver:
    def __init__(self, port=5006):
        self.port = port
        self.stopped = False
        self.current_z = 3.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", self.port))
        self.sock.settimeout(0.1)
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self.stopped:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.current_z = float(data.decode('utf-8'))
            except socket.timeout:
                pass
            except Exception:
                pass

    def get_depth(self):
        return self.current_z

    def stop(self):
        self.stopped = True
        self.sock.close()


# ----------------- TCP 发送修正角度给 Unity -----------------
def send_pitch_correction_to_unity(real_dy, z):
    try:
        command = f"CORRECT_PITCH:{real_dy:.3f}:{z:.1f}"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.1) 
        s.connect((QUEST_IP, LINE_PORT))
        s.sendall(command.encode('utf-8'))
        s.close()
    except Exception:
        pass 


# ----------------- Windows API 底层抓图 -----------------
User32 = ctypes.windll.user32
Gdi32 = ctypes.windll.gdi32

class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [
        ('biSize', wintypes.DWORD),
        ('biWidth', wintypes.LONG),
        ('biHeight', wintypes.LONG),
        ('biPlanes', wintypes.WORD),
        ('biBitCount', wintypes.WORD),
        ('biCompression', wintypes.DWORD),
        ('biSizeImage', wintypes.DWORD),
        ('biXPelsPerMeter', wintypes.LONG),
        ('biYPelsPerMeter', wintypes.LONG),
        ('biClrUsed', wintypes.DWORD),
        ('biClrImportant', wintypes.DWORD)
    ]

def capture_window_raw(hwnd, crop_top=70, crop_bottom=40):
    rect = wintypes.RECT()
    User32.GetWindowRect(hwnd, ctypes.byref(rect))
    width = rect.right - rect.left
    height = rect.bottom - rect.top
    if width <= 0 or height <= 0: return None

    hwnd_dc = User32.GetWindowDC(hwnd)
    mfc_dc = Gdi32.CreateCompatibleDC(hwnd_dc)
    save_bitmap = Gdi32.CreateCompatibleBitmap(hwnd_dc, width, height)
    Gdi32.SelectObject(mfc_dc, save_bitmap)
    User32.PrintWindow(hwnd, mfc_dc, 3)

    bmi = BITMAPINFOHEADER()
    bmi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
    bmi.biWidth, bmi.biHeight = width, -height
    bmi.biPlanes, bmi.biBitCount, bmi.biCompression = 1, 32, 0

    buffer = np.zeros((height, width, 4), dtype=np.uint8)
    Gdi32.GetDIBits(hwnd_dc, save_bitmap, 0, height, buffer.ctypes.data_as(ctypes.c_void_p), ctypes.byref(bmi), 0)

    Gdi32.DeleteObject(save_bitmap)
    Gdi32.DeleteDC(mfc_dc)
    User32.ReleaseDC(hwnd, hwnd_dc)

    frame = cv2.cvtColor(buffer, cv2.COLOR_BGRA2BGR)
    h, w, _ = frame.shape
    if h > (crop_top + crop_bottom): frame = frame[crop_top:h - crop_bottom, :]
    return frame


# ----------------- 步骤 1：提取红色掩膜 -----------------
def get_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1, upper_red1 = np.array([0, 120, 120]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([160, 120, 120]), np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


# ----------------- 步骤 2：中心引力吸附雷达点 -----------------
def find_closest_red_dot(mask, quest_cx, quest_cy):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_center = None
    min_dist = float('inf')

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        bounding_area = w * h
        if 20 <= bounding_area < 15000:
            aspect_ratio = float(w) / h
            if 0.5 <= aspect_ratio <= 2.0:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = x + w // 2, y + h // 2

                dist = np.sqrt((cx - quest_cx) ** 2 + (cy - quest_cy) ** 2)
                if dist < min_dist and dist < 1500:
                    min_dist = dist
                    best_center = (cx, cy)
    return best_center


# ----------------- 异步抓图后台线程 -----------------
class WindowCaptureThread:
    def __init__(self, hwnd, crop_top=70, crop_bottom=40):
        self.hwnd, self.crop_top, self.crop_bottom = hwnd, crop_top, crop_bottom
        self.frame, self.stopped = None, False
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self.stopped:
            if not win32gui.IsWindow(self.hwnd): break
            if win32gui.IsIconic(self.hwnd): win32gui.ShowWindow(self.hwnd, win32con.SW_SHOWNOACTIVATE)
            img = capture_window_raw(self.hwnd, self.crop_top, self.crop_bottom)
            if img is not None: self.frame = img
            time.sleep(0.001)

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


# ----------------- 主流程 -----------------
def main():
    print("=" * 50)
    print("🚀 AR 视觉贴地纠偏引擎已启动 (极速瘦身版)")
    print("正在加载 YOLO 模型...")
    
    model = YOLO('yolov8n.pt')
    try:
        model.to('cuda')
        model.half()
    except:
        pass

    target_classes = [56]
    hwnd = win32gui.FindWindow(None, "Meta Casting")
    if not hwnd:
        print("❌ 未找到 Meta Quest Casting 窗口，请开启头显投屏！")
        return

    # 启动抓图与深度接收
    cap_thread = WindowCaptureThread(hwnd, crop_top=70, crop_bottom=40)
    depth_receiver = RealTimeDepthReceiver(port=5006)

    while cap_thread.read() is None: time.sleep(0.05)

    fy = 2282.118
    prev_time = time.time()
    last_pitch_send_time = 0.0

    # ✨ 优化 1：网络通信复用，移出高频循环
    udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("✅ 系统就绪！正在实时计算贴地误差...\n")

    while not cap_thread.stopped:
        frame = cap_thread.read()
        if frame is None or frame.size == 0: continue

        # ✨ 优化 2：底层位运算替代耗时的 np 数组填充，生成极速黑底红点 Debug 画面
        red_mask = get_red_mask(frame)
        #red_only_view = cv2.bitwise_and(frame, frame, mask=red_mask)

        results = model(frame, conf=0.5, classes=target_classes, verbose=False)

        screen_cx = frame.shape[1] // 2
        screen_cy = frame.shape[0] // 2

        best_box = None
        min_center_dist = float('inf')

        # --- 阶段 1：选拔主目标（中心凝视法） ---
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                quest_cx = int((x1 + x2) / 2)
                quest_cy = int((y1 + y2) / 2)
                
                dist = math.hypot(quest_cx - screen_cx, quest_cy - screen_cy)
                if dist < min_center_dist:
                    min_center_dist = dist
                    best_box = box

        # --- 阶段 2：执行唯一主目标计算 ---
        if best_box is not None:
            x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
            quest_cx = int((x1 + x2) / 2)
            quest_cy = int((y1 + y2) / 2)

            # ✨ 优化 4：去掉在 red_only_view 上多余的绘图，专注在主画面渲染
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (quest_cx, quest_cy), 8, (0, 0, 255), -1) # 圆点改小，不挡视线

            lidar_pt = find_closest_red_dot(red_mask, quest_cx, quest_cy)

            if lidar_pt is not None:
                lidar_px, lidar_py = lidar_pt
                cv2.circle(frame, (lidar_px, lidar_py), 6, (255, 0, 255), -1)
                cv2.line(frame, (quest_cx, quest_cy), (quest_cx, lidar_py), (255, 0, 255), 2)
                cv2.line(frame, (quest_cx, lidar_py), (lidar_px, lidar_py), (255, 0, 255), 2)

                pixel_diff_y = quest_cy - lidar_py

                # 获取深度 Z (供 Unity 发送和文字显示使用)
                Z = depth_receiver.get_depth()
                if Z <= 0: Z = 3.0
                real_height_diff = (Z / fy) * pixel_diff_y

                # ✨ 优化 3：极简公式计算俯仰角，彻底摆脱 Z 运算
                pitch_angle_deg = math.degrees(math.atan2(pixel_diff_y, fy))
                
                # 使用外面初始化好的 udp_sender，不再反复创建销毁
                udp_sender.sendto(str(pitch_angle_deg).encode('utf-8'), ("127.0.0.1", UDP_PITCH_PORT))
                
                curr_t = time.time()
                if curr_t - last_pitch_send_time > 0.2:
                    send_pitch_correction_to_unity(real_height_diff, Z)
                    last_pitch_send_time = curr_t
                    print(f"\r[AR 贴地引擎] 深度: {Z:.2f}m | 高度误差: {real_height_diff:.3f}m | 俯仰角: {pitch_angle_deg:.2f}°      ", end="", flush=True)

                info_text = f"dY(px): {pixel_diff_y} | Real_dY: {real_height_diff:.2f}m @ Z={Z:.2f}m"
                cv2.putText(frame, info_text, (int(x1), max(30, int(y1) - 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 帧率计算与显示
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Quest 3 - AR Target Tracking", frame)
        #cv2.imshow("Red Filter Debug Window", red_only_view)

        if cv2.waitKey(1) & 0xFF == ord('q'): break

    # 退出资源释放
    cap_thread.stop()
    depth_receiver.stop()
    udp_sender.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()