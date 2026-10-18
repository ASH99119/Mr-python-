import win32file
import struct
import cv2
import json
import socket  # 重新引入网络通信库
from ultralytics import YOLO

# ==========================================
# 1. 通信配置 (发往 Unity/Quest 3)
# ==========================================
# 注意：如果是连着线在电脑 Unity 编辑器里测试，写 127.0.0.1
# 如果是打包到 Quest 3 里面脱机运行，写 Quest 3 的局域网 IP
QUEST_IP = "172.20.10.9"
QUEST_PORT = 5005  # 对应你 C# 里的 public int port = 5005;

# 初始化 UDP 客户端
udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# ==========================================
# 2. 从 C++ 读取距离和三维坐标数据的辅助函数
# ==========================================
def read_distance(pipe):
    try:
        _, raw_len = win32file.ReadFile(pipe, 4)
        msg_len = struct.unpack("I", raw_len)[0]

        _, data = win32file.ReadFile(pipe, msg_len)
        result = json.loads(data.decode())
        return result
    except:
        return None

    # ==========================================


# 3. 初始化 YOLO 和 命名管道
# ==========================================
model = YOLO("yolov8n.pt")

pipe = win32file.CreateFile(
    r'\\.\pipe\yolo_pipe',
    win32file.GENERIC_WRITE | win32file.GENERIC_READ,
    0,
    None,
    win32file.OPEN_EXISTING,
    0,
    None
)

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print(f"雷达端 Python YOLO 已启动，将向 {QUEST_IP}:{QUEST_PORT} 发送 3D 坐标...")

# ==========================================
# 4. 主循环
# ==========================================
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    results = model(frame, device=0, classes=[56], verbose=False)
    boxes_to_send = []

    if len(results[0].boxes) > 0:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()
        confs = results[0].boxes.conf.cpu().numpy()

        for i in range(len(boxes)):
            xmin, ymin, xmax, ymax = boxes[i]
            cls = int(classes[i])
            conf = float(confs[i])
            boxes_to_send.append([
                int(xmin), int(ymin), int(xmax), int(ymax), cls, conf
            ])

            cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)

    # --- A：发送图像和框给 C++ ---
    _, img_encoded = cv2.imencode(".jpg", frame)
    img_bytes = img_encoded.tobytes()
    bbox_json = json.dumps(boxes_to_send).encode()

    packet = (
            struct.pack("I", len(img_bytes)) +
            img_bytes +
            struct.pack("I", len(bbox_json)) +
            bbox_json
    )

    try:
        win32file.WriteFile(pipe, packet)
    except Exception as e:
        print("管道写入失败，C++端可能已断开:", e)
        break

    # --- B：读取 C++ 物理坐标并发送给 Unity ---
    cxx_dist_data = read_distance(pipe)

    if cxx_dist_data is not None:
        mid_dist = cxx_dist_data.get("mid_dist", -1.0)
        mid_X = cxx_dist_data.get("mid_X", 0.0)
        mid_Y = cxx_dist_data.get("mid_Y", 0.0)
        mid_Z = cxx_dist_data.get("mid_Z", 0.0)

        if mid_dist > 0:
            # 1. 打印测试信息

            # 加入下面这行，让终端疯狂打印，确认程序没卡死
            print(f"正在发送UDP -> X:{mid_X:.2f}, Y:{mid_Y:.2f}, Z:{mid_Z:.2f}")


            info_text = f"Dist: {mid_dist:.2f}m | XYZ: ({mid_X:.2f}, {mid_Y:.2f}, {mid_Z:.2f})"
            cv2.putText(frame, info_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 2. 组装 Unity 期望的 JSON 格式
            # 2. 组装 Unity 期望的 JSON 格式 (在 Python 端完成坐标系转换)
            udp_msg = {
                "x": mid_X,  # 左右对应真实世界的 X
                "y": mid_Z,  # Unity 里的 y (深度) 对应真实世界的 Z
                "z": -mid_Y  # Unity 里的 z (高度) 对应真实世界取反后的 Y
            }

            # 3. 发送 UDP 数据包
            try:
                # 发送给 Quest 3 / Unity
                udp_client.sendto(json.dumps(udp_msg).encode('utf-8'), (QUEST_IP, QUEST_PORT))

                # 【新增】将深度数据单独发送给本地的第二段代码（端口 5006）
                udp_client.sendto(str(mid_dist).encode('utf-8'), ("127.0.0.1", 5006))
            except Exception as e:
                pass

                # 显示本地画面
    cv2.imshow("Python -> Unity Sender", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()