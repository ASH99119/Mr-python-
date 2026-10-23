#多进程主程序调度模块
#利用 Python 的 multiprocessing 模块，将原本三个互斥的 while True 死循环强行拆分到了三个独立的物理核心（进程）上同时运行，
#解决了 cv2.imshow 和 pynput 键盘监听互相卡死的问题

# 架构说明：采用 3 个物理隔离的独立进程，确保视觉抓图、C++通信、轨迹计算互不抢占 CPU
#进程 1 (process_ar_casting)：专门负责高速截屏、跑 YOLO 识别投屏画面、计算红点俯仰角误差，并将纠偏指令发给 Unity 和本地键盘进程。
#进程 2 (process_keyboard_controller)：监听按键，检测到深度变化或自动角度传过来，瞬间重算 80 个 3D 轨迹点并丢给 Unity 画线，同时负责回传车速数字。
#进程 3 (process_radar_sender)：独立拉起物理摄像头，跑第二个 YOLO 模型找现实车辆，和 C++ 程序交换坐标后，将精准的三维数据发给 Unity 渲染 3D 预警框

import multiprocessing  #多进程核心库：强行调动 CPU 的多个物理核心
import threading    #多线程库：用于在单个进程内开辟“不费脑子”的后台等待任务
import time         #时间库：用于控制循环频率、计算 FPS
import math         #数学库
import socket       #网络库
import json         #json数据库，把python字典发给c++
import cv2          # opencv库
import win32gui     #Windows 底层 API：用于通过名字查找投屏窗口
import win32file    #Windows 文件系统底层 API：用来读写与 C++ 通信的命名管道
import struct       #C语言数据打包库：处理底层的二进制封包
from pynput import keyboard     # 键盘监听库
from ultralytics import YOLO    # YOLOv8 深度学习目标检测模型

# 导入底层工具包
from config import *
from network_core import RealTimeDepthReceiver, TrajectoryPlanner
from vision_core import WindowCaptureThread, get_red_mask, find_closest_red_dot, read_distance


# ==================== 进程 1：AR 贴地引擎 ====================
def process_ar_casting():
    """
    AR 视觉贴地纠偏进程：负责实时截取头显投屏窗口，通过 YOLO 定位基准目标，
    并结合地面激光红点与本地深度数据计算物理高度差和俯仰角误差，发送给 Unity 和控制器。
    """
    model = YOLO('yolov8n.pt')  
    try:
        model.to('cuda');   # 尝试调用 Nvidia 独立显卡
        model.half()        # 开启半精度 (FP16) 推理：显存减半，速度翻倍
    except Exception:
        pass

    # 精确查找 Quest 3 官方投屏窗口的底层句柄，后续窗口名变了改Meta.....
    hwnd = win32gui.FindWindow(None, "Meta Casting")
    #没开投屏软件则直接退出
    if not hwnd: return


    # 启动后台疯狂抓图线程，裁掉 UI 黑边
    cap_thread = WindowCaptureThread(hwnd, crop_top=70, crop_bottom=40)
    # 启动后台静默接收 C++ 算出的物理深度,也就是真实深度
    depth_receiver = RealTimeDepthReceiver(port=DEPTH_PORT)
    # 建立 UDP 发送端，给进程2发送
    udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 等待抓图线程吐出第一张画面
    while cap_thread.read() is None: time.sleep(0.05)
    prev_time, last_pitch_send_time = time.time(), 0.0

    # 【新增】初始化上一帧的滤波角度
    last_filtered_pitch = 0.0

    while not cap_thread.stopped:
        frame = cap_thread.read()
        # 抓空了直接跳过，防止 OpenCV 崩溃
        if frame is None or frame.size == 0: continue

        # 提取全图红色掩膜
        red_mask = get_red_mask(frame)
        # YOLO 识别 COCO 第 56 类
        results = model(frame, conf=0.5, classes=[56], verbose=False)
        # 算出物理屏幕的几何中心
        screen_cx, screen_cy = frame.shape[1] // 2, frame.shape[0] // 2
        best_box, min_center_dist = None, float('inf')

        # 阶段一：筛选出距离画面正中心最近的特定目标
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                quest_cx, quest_cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                # 勾股定理算距离
                dist = math.hypot(quest_cx - screen_cx, quest_cy - screen_cy)
                if dist < min_center_dist: min_center_dist, best_box = dist, box

        # 阶段二：围绕目标计算红点像素差并推导贴地纠偏角
        if best_box is not None:
            x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
            quest_cx, quest_cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (quest_cx, quest_cy), 8, (0, 0, 255), -1)

            # 在红色掩膜里，找一个离这个目标中心最近的真实激光红点
            lidar_pt = find_closest_red_dot(red_mask, quest_cx, quest_cy)
            if lidar_pt is not None:
                lidar_px, lidar_py = lidar_pt
                cv2.circle(frame, (lidar_px, lidar_py), 6, (255, 0, 255), -1)
                cv2.line(frame, (quest_cx, quest_cy), (quest_cx, lidar_py), (255, 0, 255), 2)
                cv2.line(frame, (quest_cx, lidar_py), (lidar_px, lidar_py), (255, 0, 255), 2)

                pixel_diff_y = quest_cy - lidar_py  # 计算基准框中心和真实红点的像素高度差
                Z = depth_receiver.get_depth()      # 瞬间读出 C++ 最新的物理深度
                if Z <= 0: Z = 3.0                  # 防御机制：防止除数为 0
                real_height_diff = (Z / FY) * pixel_diff_y          # 小孔成像：物理高度误差 = (深度/焦距) * 像素差
                #计算原始物理角度
                raw_pitch_angle = math.degrees(math.atan2(pixel_diff_y, FY))    # 三角函数：求出镜头补偿仰角
                #新增
                # 2. 绝对限幅 (Hard Clamp)：强制把单次补偿角度限制在 ±5.0 度以内
                # 汽车悬挂的物理形变和道路坡度造成的俯仰误差，绝不可能单次超过 5 度
                MAX_PITCH = 5.0
                clamped_pitch = max(-MAX_PITCH, min(MAX_PITCH, raw_pitch_angle))

                # 3. 一阶低通滤波 (EMA Filter)：消除 YOLO 框的连续高频抖动
                # 权重 0.2 表示只信任 20% 的当前新角度，保留 80% 的历史平滑角度
                ALPHA = 0.2
                filtered_pitch = ALPHA * clamped_pitch + (1.0 - ALPHA) * last_filtered_pitch
                last_filtered_pitch = filtered_pitch  # 更新历史值

                # 将最终的平滑限幅角度交给网络发送
                pitch_angle_deg = filtered_pitch

                # 将算出的自动俯仰角通过本地 UDP 广播给键盘轨迹规划进程
                udp_sender.sendto(str(pitch_angle_deg).encode('utf-8'), ("127.0.0.1", UDP_PITCH_PORT))

                curr_t = time.time()
                # 限制 TCP 发往 Unity 的频率 (约 5Hz)，防止网络拥堵导致 Unity 崩溃
                if curr_t - last_pitch_send_time > 0.2:
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(0.1)
                        s.connect((QUEST_IP_AR, LINE_PORT))
                        s.sendall(f"CORRECT_PITCH:{real_height_diff:.3f}:{Z:.1f}".encode('utf-8'))
                        s.close()
                    except Exception:
                        pass
                    last_pitch_send_time = curr_t

                info_text = f"dY(px): {pixel_diff_y} | Real_dY: {real_height_diff:.2f}m @ Z={Z:.2f}m"
                cv2.putText(frame, info_text, (int(x1), max(30, int(y1) - 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255), 2)

        # 实时计算并渲染帧率显示
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Quest 3 - AR Target Tracking", frame)
        # 按 Q 退出
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    # 释放抓图线程、深度接收器及网络套接字资源
    cap_thread.stop()
    depth_receiver.stop()
    udp_sender.close()
    cv2.destroyAllWindows()


# ==================== 进程 2：按键与轨迹规划器 ====================
def process_keyboard_controller():
    """
    路测交互控制进程：在后台独立挂载键盘监听器，实时响应车道微调、清屏或钉地指令；
    同时融合进程 1 传来的视觉补偿角，利用五次多项式生成 80 个 3D 轨迹点并下发至 Unity。
    """
    left_lane_config = DEFAULT_LEFT.copy()  # 深拷贝左车道参数
    right_lane_config = DEFAULT_RIGHT.copy()    # 深拷贝右车道参数
    state = {"current_number_input": "", "auto_pitch_deg": 0.0, "last_active_direction": 'left'}
    planner = TrajectoryPlanner(num_points=80)  # 实例化五次多项式矩阵生成器

    def receive_auto_pitch():
        """后台进程1：后台专职接收本地 UDP 50007 端口发来的自动俯仰角补偿      1：专职接收进程 1 算出来的俯仰角补偿"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", UDP_PITCH_PORT))
        while True:
            try:
                data, _ = sock.recvfrom(1024)
                state["auto_pitch_deg"] = float(data.decode('utf-8').strip())
            except Exception:
                pass

    def print_status_loop():
        """后台进程2：每隔 0.5 秒在终端单行刷新当前车道状态及仰角合成数值，防止刷屏"""
        while True:
            config = left_lane_config if state["last_active_direction"] == 'left' else right_lane_config
            x, y, manual_p = config["X_max"], config["end_y"], config["pitch"]
            total_p = manual_p + state["auto_pitch_deg"]
            dir_str = "左道" if state["last_active_direction"] == 'left' else "右道"
            if state["current_number_input"] == "":
                print(
                    f"\r[{dir_str}监控] 长度:{x:.1f}m | 横移:{y:.2f}m | 手动基准:{manual_p:.1f}° + 自动补偿:{state['auto_pitch_deg']:.2f}° = 最终仰角:{total_p:.2f}°      ",
                    end="", flush=True)
            time.sleep(0.5)

    def plan_and_send(direction):
        """核心轨迹分发函数：基于当前车道配置和最终合成仰角生成 80 个点，通过 TCP 传给 Unity"""
        state["last_active_direction"] = direction
        config = left_lane_config if direction == 'left' else right_lane_config
        trajectory_points = planner.generate(0.0, config["end_y"], config["X_max"], config["pitch"])
        trajectory_data = "\n".join([f"{p[0]},{p[1]},{p[2]}" for p in trajectory_points])
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((QUEST_IP_AR, LINE_PORT))
            s.sendall(trajectory_data.encode('utf-8'))
            s.close()
        except Exception:
            pass

    def send_speed(speed_val_str):
        """向 Unity 测速端口发送车速字符串数据"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((QUEST_IP_AR, SPEED_PORT))
            s.sendall(f"{speed_val_str}km/h".encode('utf-8'))
            s.close()
        except Exception:
            pass



#后续删掉这个改为CAN读取速度
    def on_press(key):
        """pynput 键盘监听回调：处理 WASD/IJKL 微调、空格钉地、小键盘车速输入及回车/退格逻辑"""
        try:
            if hasattr(key, 'char') and key.char is not None:
                key_char = key.char.lower()
                if key_char == 'w':
                    left_lane_config["X_max"] += LENGTH_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 's':
                    left_lane_config["X_max"] -= LENGTH_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 'a':
                    left_lane_config["end_y"] -= LATERAL_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 'd':
                    left_lane_config["end_y"] += LATERAL_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 'q':
                    left_lane_config["pitch"] += PITCH_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 'e':
                    left_lane_config["pitch"] -= PITCH_STEP;
                    plan_and_send('left');
                    return
                elif key_char == 'i':
                    right_lane_config["X_max"] += LENGTH_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'k':
                    right_lane_config["X_max"] -= LENGTH_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'j':
                    right_lane_config["end_y"] -= LATERAL_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'l':
                    right_lane_config["end_y"] += LATERAL_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'u':
                    right_lane_config["pitch"] += PITCH_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'o':
                    right_lane_config["pitch"] -= PITCH_STEP;
                    plan_and_send('right');
                    return
                elif key_char == 'r':
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(1.0);
                        s.connect((QUEST_IP_AR, LINE_PORT));
                        s.sendall(b"RESET");
                        s.close()
                    except Exception:
                        pass
                    return

            if key == keyboard.Key.space:   # 空格触发 Unity 钉地模式，钉在地上不以速度往后移动
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0);
                    s.connect((QUEST_IP_AR, LINE_PORT));
                    s.sendall(b"TOGGLE_NAIL");
                    s.close()
                except Exception:
                    pass
                return
            elif hasattr(key, 'vk') and 96 <= key.vk <= 105:    # 拦截小键盘数字
                state["current_number_input"] += str(key.vk - 96)
                print(f"\r[正在输入车速...] {state['current_number_input']} km/h                   ", end="",
                      flush=True)
                return
            elif key == keyboard.Key.up:
                plan_and_send(state["last_active_direction"]);
                return
            elif key == keyboard.Key.down:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0);
                    s.connect((QUEST_IP_AR, LINE_PORT));
                    s.sendall(b"CLEAR");
                    s.close()
                except Exception:
                    pass
                return
            elif key == keyboard.Key.enter:
                if state["current_number_input"]:
                    send_speed(state["current_number_input"])
                    state["current_number_input"] = ""
                    plan_and_send(state["last_active_direction"])
                return
            elif key == keyboard.Key.backspace:
                if len(state["current_number_input"]) > 0:
                    state["current_number_input"] = state["current_number_input"][:-1]
                    print(f"\r[正在输入车速...] {state['current_number_input']} km/h                   ", end="",
                          flush=True)
                return
        except Exception:
            pass

    def can_speed_receiver_loop():
        """
        后台线程 3：接收 C++ 底层读取的真实 CAN 车速，并驱动全系统更新
        """
        # 1. 建立 UDP 监听端口，专门等 C++ 传数据过来
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 50009))  # 与 C++ 宏定义的端口一致

        last_speed = -1.0  # 记录上一次的车速，防止重复发包

        while True:
            try:
                # 2. 接收 C++ 发来的纯数字字符串 (如 "60")
                data, _ = sock.recvfrom(1024)
                real_speed = float(data.decode('utf-8'))

                # 3. 如果车速没变，就不需要重算轨迹，节省 CPU
                if real_speed == last_speed:
                    continue

                # 4. 把车速发给 Unity 顶部的虚拟 UI 看板 (复用已有的 TCP 50002 通道)
                send_speed(f"{real_speed:.1f}")

                # ==========================================
                # 【动态轨迹算法拓展区】
                # 你可以在这里根据车速动态调整车道的预测长度 (X_max)
                # 例如：速度越快，规划的预警线就越长。假设 1km/h 对应 0.5米
                # config = left_lane_config if state["last_active_direction"] == 'left' else right_lane_config
                # config["X_max"] = max(10.0, real_speed * 0.5)
                # ==========================================

                # 5. 车速一旦变化，重新计算 80 个 3D 轨迹点下发
                plan_and_send(state["last_active_direction"])

                last_speed = real_speed

            except Exception:
                pass


    # 启动后台守护线程接收自动角度及刷新看板，主线程挂载 pynput 键盘监听
    # 1. 启动后台自动角度接收线程
    threading.Thread(target=receive_auto_pitch, daemon=True).start()
    # 2. 启动后台状态看板刷新线程
    threading.Thread(target=print_status_loop, daemon=True).start()
    # 3. 启动后台 CAN 车速接收线程 (让底层硬件全自动接管车速)
    threading.Thread(target=can_speed_receiver_loop, daemon=True).start()

    # 这句 listener.join() 目前不仅在监听键盘，还在充当死循环防止主线程退出。
    # 未来删掉键盘代码换成 CAN 线程时，把这里替换成 `while True: time.sleep(1.0)`
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


# ==================== 进程 3：雷达目标发送端 ====================
def process_radar_sender():
    """
    雷达与视觉融合及数据转发进程：
    极速 UDP 版本：绝不卡顿，绝不阻塞画面！
    """
    model = YOLO("yolov8n.pt")
    pipe = win32file.CreateFile(r'\\.\pipe\yolo_pipe', win32file.GENERIC_WRITE | win32file.GENERIC_READ, 0, None,
                                win32file.OPEN_EXISTING, 0, None)

    cap = cv2.VideoCapture(1)  # 如果用笔记本自带请改为 0
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # 建立专门发给 Quest 3 的 UDP 客户端
    udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 建立专门发给本地 5006 端口 (进程1) 的 UDP 客户端
    local_udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        ret, frame = cap.read()
        if not ret: continue

        # 如果你没显卡，记得把 device=0 改成 device='cpu'
        results = model(frame, device='cpu', classes=[2, 5, 7], verbose=False)
        boxes_to_send = []

        if len(results[0].boxes) > 0:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()

            for i in range(len(boxes)):
                xmin, ymin, xmax, ymax = boxes[i]
                boxes_to_send.append([int(xmin), int(ymin), int(xmax), int(ymax), int(classes[i]), float(confs[i])])
                cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)

        # 封包图片给 C++
        _, img_encoded = cv2.imencode(".jpg", frame)
        img_bytes = img_encoded.tobytes()
        bbox_json = json.dumps(boxes_to_send).encode()
        packet = struct.pack("I", len(img_bytes)) + img_bytes + struct.pack("I", len(bbox_json)) + bbox_json

        try:
            win32file.WriteFile(pipe, packet)
        except Exception:
            break

        # 读取 C++ 计算结果
        cxx_dist_data = read_distance(pipe)
        if cxx_dist_data is not None:
            mid_dist = cxx_dist_data.get("mid_dist", -1.0)
            mid_X = cxx_dist_data.get("mid_X", 0.0)
            mid_Y = cxx_dist_data.get("mid_Y", 0.0)
            mid_Z = cxx_dist_data.get("mid_Z", 0.0)

            if mid_dist > 0:
                # 终端调试输出
                print(f"正在发送UDP -> X:{mid_X:.2f}, Y:{mid_Y:.2f}, Z:{mid_Z:.2f}")

                info_text = f"Dist: {mid_dist:.2f}m | XYZ: ({mid_X:.2f}, {mid_Y:.2f}, {mid_Z:.2f})"
                cv2.putText(frame, info_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # 映射 Unity 坐标
                udp_msg = {"x": mid_X, "y": mid_Z, "z": -mid_Y}

                # 直接使用 UDP 发送，不用 JSON 后面的 + "\n"，也不用重连逻辑
                try:
                    udp_client.sendto(json.dumps(udp_msg).encode('utf-8'), (QUEST_IP_RADAR, QUEST_PORT))
                except Exception:
                    pass

                try:
                    local_udp_client.sendto(str(mid_dist).encode('utf-8'), ("127.0.0.1", DEPTH_PORT))
                except Exception:
                    pass

        cv2.imshow("Python -> Unity Sender (UDP)", frame)
        if cv2.waitKey(1) == 27: break

    cap.release()
    cv2.destroyAllWindows()

# ==================== 主程序入口 ====================
if __name__ == '__main__':
    # 必须加上 freeze_support 避免 Windows 多进程打包/运行时的递归启动异常
    multiprocessing.freeze_support()

    # 实例化三个独立进程，分别对应 AR 视觉贴地纠偏、轨迹规划控制、雷达目标发送
    p1 = multiprocessing.Process(target=process_ar_casting)
    p2 = multiprocessing.Process(target=process_keyboard_controller)
    p3 = multiprocessing.Process(target=process_radar_sender)

    # 启动所有子进程
    p1.start()
    p2.start()
    p3.start()

    # 阻塞主线程以保持子进程持续稳定运行
    p1.join()
    p2.join()
    p3.join()
