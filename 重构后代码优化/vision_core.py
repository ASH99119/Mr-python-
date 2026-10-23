#底层视觉 API 模块
import cv2  # OpenCV 视觉库，用于画框、画线、显示调试画面
import win32gui # Windows 底层 API，用于查找和操作特定窗口 (如 Meta Casting)
import win32con # Windows 常量库：包含了很多底层的系统控制代码（比如窗口最大化、隐藏的标识符）
import win32file   # Windows 文件底层操作库：在这里专门用来读取“命名管道”
import ctypes # C 语言数据类型接口：让 Python 能够直接调用底层的 C 语言动态链接库 (DLL)
import numpy as np  # 矩阵计算库：用来把 C 语言的底层内存数据转化为 Python 能懂的图像矩阵
import struct   # C 语言结构体打包解包库：用来处理底层的二进制字节流
import json # 数据序列化库：用来解析 C++ 传过来的目标坐标字典
import time # 时间库：用于线程休眠控制
import threading    # 多线程库：用于后台抓图
from ctypes import wintypes # 引入 ctypes 里专门为 Windows 预定义的 C 语言类型 (如 DWORD, LONG, RECT)

# ==========================================
# 1. Windows GDI 底层内存抓图模块
# ==========================================
# 引入底层 API，绕过高层截图库（如 pyautogui），实现对特定句柄窗口的直接内存读取
# User32：负责窗口管理和用户输入； Gdi32：负责图形设备接口（Graphic Device Interface），也就是显卡画图的底层API
User32, Gdi32 = ctypes.windll.user32, ctypes.windll.gdi32


class BITMAPINFOHEADER(ctypes.Structure):
    """
    用 Python 模仿 C 语言的结构体 (Struct)）
    定义 Windows 位图信息头结构体
    用于告诉系统我们将要提取的 DIB (设备无关位图) 的宽高、色深和压缩格式
    """

    _fields_ = [('biSize', wintypes.DWORD), # 结构体本身的大小
                ('biWidth', wintypes.LONG), # 图像宽度
                ('biHeight', wintypes.LONG),    # 图像高度
                ('biPlanes', wintypes.WORD),    # 色彩平面数 (必须为1)
                ('biBitCount', wintypes.WORD),  # 色深：我们后面设为 32，即 BGRA 四个通道
                ('biCompression', wintypes.DWORD),  # 压缩方式：0 表示不压缩
                ('biSizeImage', wintypes.DWORD),    # 图像字节大小 (设为0系统会自动算)
                ('biXPelsPerMeter', wintypes.LONG), # 水平分辨率 (设为0)
                ('biYPelsPerMeter', wintypes.LONG), # 垂直分辨率 (设为0)
                ('biClrUsed', wintypes.DWORD),  # 使用的颜色数 (设为0)
                ('biClrImportant', wintypes.DWORD)] # 重要的颜色数 (设为0)


def capture_window_raw(hwnd, crop_top=70, crop_bottom=40):
    """
    基于 GDI 的后台窗口抓取 API
    特性：即使 "Meta Casting" 窗口被其他应用部分遮挡，也能强制重绘并抓取完整画面
    """
    rect = wintypes.RECT()  # 实例化一个 C 语言的矩形结构体
    User32.GetWindowRect(hwnd, ctypes.byref(rect))  # 调用系统 API，把目标窗口的左上角和右下角坐标填进 rect 里
    width, height = rect.right - rect.left, rect.bottom - rect.top  # 算出窗口的实际像素宽和高
    if width <= 0 or height <= 0: return None   #如果窗口最小化或者没了，长宽是 0，直接返回空

    # 第一步：获取源窗口的“画板”(Device Context, DC)，并向系统显存申请一块一模一样大的“兼容虚拟画板”(mfc_dc)
    hwnd_dc = User32.GetWindowDC(hwnd)
    mfc_dc = Gdi32.CreateCompatibleDC(hwnd_dc)
    save_bitmap = Gdi32.CreateCompatibleBitmap(hwnd_dc, width, height)  # 在虚拟画板上铺一张兼容的空白画布
    Gdi32.SelectObject(mfc_dc, save_bitmap) # 把画布和画板绑定

    # 第二步:强制让这个窗口把它最新的画面，立刻画到我们刚才申请的内存画布 (mfc_dc) 上，无视屏幕遮挡
    User32.PrintWindow(hwnd, mfc_dc, 3)

    # 第三步：填写上面定义的位图申请表
    bmi = BITMAPINFOHEADER()
    bmi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
    # 高度给负数 (-height)。因为 Windows 底层的 BMP 图片是从下往上倒着存的，给负数强制系统从上往下正着存
    bmi.biWidth, bmi.biHeight = width, -height
    bmi.biPlanes, bmi.biBitCount, bmi.biCompression = 1, 32, 0  # 1层，32位色彩(BGRA)，无压缩

    # 第四步：在 Python 里开辟一块对应大小的 Numpy 内存数组
    buffer = np.zeros((height, width, 4), dtype=np.uint8)
    # 调用底层的 GetDIBits 接口，把刚才 C 语言画板上的二进制像素，直接拷贝到 Python 的 buffer 数组里
    Gdi32.GetDIBits(hwnd_dc, save_bitmap, 0, height, buffer.ctypes.data_as(ctypes.c_void_p), ctypes.byref(bmi), 0)

    # 第五步：C 语言的内存不会自动回收，用完必须手动 Delete 掉，否则每抓一帧就吃掉几MB内存，几十秒电脑就卡死了
    Gdi32.DeleteObject(save_bitmap)
    Gdi32.DeleteDC(mfc_dc)
    User32.ReleaseDC(hwnd, hwnd_dc)

    #第六步：数据格式转换与裁剪
    frame = cv2.cvtColor(buffer, cv2.COLOR_BGRA2BGR)  # Windows 抓出来的是 BGRA(带透明通道)，转成标准的 BGR 彩色图
    h, w, _ = frame.shape
    # 裁掉上面 70 像素（浏览器/软件的标题栏），裁掉下面 40 像素（黑边），只留纯净的投屏画面
    if h > (crop_top + crop_bottom): frame = frame[crop_top:h - crop_bottom, :]
    return frame


class WindowCaptureThread:
    """
    异步画面推流线程
    功能：将耗时约 10-20ms 的 GDI 抓图操作放到后台死循环执行，
         主程序yolo调用 read() 瞬间即可拿到最新帧，绝不阻塞 YOLO 推理。
    """

    def __init__(self, hwnd, crop_top=70, crop_bottom=40):
        self.hwnd, self.crop_top, self.crop_bottom = hwnd, crop_top, crop_bottom
        self.frame, self.stopped = None, False
        threading.Thread(target=self._update, daemon=True).start()  # 挂载后台守护线程

    def _update(self):
        while not self.stopped:
            # 如果投屏软件被用户关了，自动退出线程
            if not win32gui.IsWindow(self.hwnd): break
            # 若投屏窗口被最小化，则静默恢复它（无焦点强制展示），保证能抓到画面
            # SW_SHOWNOACTIVATE 的意思是：把窗口从任务栏静默恢复，但不要抢走当前用户的鼠标焦点。
            if win32gui.IsIconic(self.hwnd): win32gui.ShowWindow(self.hwnd, win32con.SW_SHOWNOACTIVATE)

            # 调用上面写好的 GDI 抓图
            img = capture_window_raw(self.hwnd, self.crop_top, self.crop_bottom)
            # 把抓到的画面存进成员变量，供外部随时取用
            if img is not None: self.frame = img
            # 时停 1 毫秒，防止把 CPU 的这一个核心彻底跑满 100%
            time.sleep(0.001)

    def read(self):
        return self.frame   # 外部瞬间读取最新帧

    def stop(self):
        self.stopped = True # 安全停止标识


# ==========================================
# 2. OpenCV 视觉图像算子
# ==========================================

def get_red_mask(frame):
    """
    自适应投屏压缩的红色提取器：放宽明度下限至 140，确保红球轮廓完整
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 降低明度 V 下限至 140（抗压缩衰减），饱和度 S 保持 120 排除白光反射
    lower_red1 = np.array([0, 120, 140])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 120, 140])
    upper_red2 = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    # 闭运算修补因反光产生的小孔洞，保证轮廓是一个完整的实心圆
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_closest_red_dot(mask, quest_cx, quest_cy):
    """
    基于圆度与面积自适应的红球抓取算法
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_center = None
    min_dist = float('inf')

    for cnt in contours:
        area = cv2.contourArea(cnt)

        # 1. 放宽面积区间：容纳 15~80 像素直径的红球（约 180 ~ 5000 像素）
        if not (180 <= area <= 5000):
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        # 2. 核心圆度过滤：正圆约为 0.78~1.0，门牌、方块文字圆度通常低于 0.5
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        if circularity < 0.65:
            continue

        # 3. 计算红球重心
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        else:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2

        # 4. 放宽搜索半径至 600 像素，红球无论在椅子上方多高都能被捕获
        dist = np.hypot(cx - quest_cx, cy - quest_cy)
        if dist < min_dist and dist < 600:
            min_dist = dist
            best_center = (cx, cy)

    return best_center

# ==========================================
# 3. IPC 进程间通信 (Windows 命名管道)
# ==========================================
def read_distance(pipe):
    """
    从 C++ 进程读取 JSON 格式的 3D 物理坐标
    协议约定：基于自定义的 TLV (Type-Length-Value) 封包格式。
             C++ 端先发一个 4 字节的无符号整数 (DWORD) 表示后续 JSON 字符串的长度，
             然后再发送 JSON 数据本身。
    """
    try:
        # 第一步：只向水管里读 4 个字节。
        # 在 C++ 端，发 JSON 数据之前，会先发一个 4 字节的无符号整数 (DWORD)，里面写着“接下来的 JSON 有多长”
        _, raw_len = win32file.ReadFile(pipe, 4)
        # 使用 Python 的 struct 模块，把 C 语言底层的 4字节无符号整型 (I) 还原成数字
        msg_len = struct.unpack("I", raw_len)[0]

        # 第二步：精确读取载荷。
        # 既然知道了 JSON 有多长 (msg_len)，那我们就精确地从水管里抽这么多个字节出来，哪怕水管里还有其他数据也不会多读
        _, data = win32file.ReadFile(pipe, msg_len)

        # 第三步：数据反序列化。把 C++ 发来的原始字节流 (data) 按照 utf-8 规则解码成字符串，然后还原为 Python 字典并返回
        return json.loads(data.decode())
    except Exception:
        # 如果水管干了没数据，或者 C++ 端还没开始发，直接静默返回空，交回给主循环重试，绝不报错崩溃
        return None