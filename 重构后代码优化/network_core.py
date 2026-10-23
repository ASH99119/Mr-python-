#多项式规划器：封装了 TrajectoryPlanner 类，负责通过五次多项式矩阵运算实时生成 80 个平滑的 3D 空间路线坐标。
#本地数据中转：封装了 RealTimeDepthReceiver 类，用于在后台纯异步地接收底层雷达广播出来的 Z 轴深度数据。
#(通信与轨迹算法模块)
import socket   #导入底层的网络套接字库，用于实现 UDP 和 TCP 数据传输
import threading  #导入多线程模块，用于在后台开启独立任务，不阻塞主程序运行
import numpy as np  #导入数学计算库 Numpy，用于求解矩阵方程
from config import *  #从写的 config.py 中导入所有的全局常量（如端口号、物理参数）


class RealTimeDepthReceiver:
    # 定义一个实时接收雷达深度的类，充当后台数据中转站
    """
    UDP 实时深度接收线程
    功能: 在后台纯异步监听 5006 端口，接收底层 C++ 雷达程序广播出的 Z 轴深度数据。
    """

    def __init__(self, port=DEPTH_PORT):
        # 类的初始化函数，默认监听从 config.py 导入的 DEPTH_PORT (5006端口)
        self.port = port   #让类的port实例化有值
        self.stopped = False    # 设置一个控制线程运行状态的标志位，默认值为 False（不停止）
        self.current_z = 3.0  # 默认深度为 3.0 米，防止雷达无数据时报错

        # 初始化 UDP Socket (IPv4, 数据报模式)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 将sock绑定到电脑回环 IP  和指定的端口上，开始监听本地网络
        self.sock.bind(("127.0.0.1", self.port))

        # 核心设置：0.1秒超时。防止 recvfrom 死锁阻塞整个线程，保证 stop() 能够顺利介入退出
        self.sock.settimeout(0.1)

        # 创建并启动一个后台线程运行 _update 方法。daemon=True 表示主程序退出时它会自动销毁
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        """后台轮询接收深度数据的死循环"""
        while not self.stopped:         # 只要停止标志位还是 False，就一直循环执行
            try:
                # 尝试从绑定的 UDP 端口接收最多 1024 字节的数据，data 是内容，
                # _ 是发送方的地址信息（用_忽略）
                data, _ = self.sock.recvfrom(1024)
                # 把收到的字节流解码成 utf-8 字符串，再转换成浮点数，更新当前的深度 Z
                self.current_z = float(data.decode('utf-8'))
            except Exception:
                # 捕获超时或其他异常就pass跳过处理，继续下一轮监听
                pass

    def get_depth(self):
        """供外部主线程安全调用的数据读取接口"""
        #获取最新深度的接口函数
        return self.current_z  # 直接返回当前内存里保存的最新的 Z 深度值

    def stop(self):
        """安全释放 Socket 资源并终止线程"""
        # 提供一个安全关闭接收器的方法接口函数
        self.stopped = True # 将标志位置为 True，打破 _update 里面的 while 循环
        self.sock.close()   # 关闭 UDP sockt，释放操作系统的网络端口资源


class TrajectoryPlanner:
    """
    五次多项式实时规划器 (带 3D 旋转变换)
    功能: 根据前方的纵深 (X_max) 和横向偏移 (end_y)，利用五次多项式生成 80 个平滑过渡的 3D 坐标点，
          并根据俯仰角 (pitch) 对轨迹进行 3D 空间旋转，最终映射为 Unity 坐标系。
    """

    def __init__(self, num_points=80):
        self.N = num_points  # 生成的轨迹点数量，80个点足以保证 AR 曲线平滑

    def generate(self, start_y, end_y, X_max, pitch_deg):
        """
        生成 3D 轨迹点阵
        :param start_y: 起点横向位置 (通常为 0.0)
        :param end_y: 终点横向位置 (如左侧车道 -3.5m)
        :param X_max: 轨迹的纵向总长度 (如 40m)
        :param pitch_deg: 轨迹的整体俯仰角 (用于贴合上下坡)
        """
        X_max = max(5.0, X_max)  # 限制最短规划距离为 5 米，防止分母过小导致矩阵奇异

        # 构建五次多项式矩阵 A
        # 方程形式: y(x) = c0 + c1*x + c2*x^2 + c3*x^3 + c4*x^4 + c5*x^5
        A = np.array([
            [1, 0, 0, 0, 0, 0],  # 条件1: 起点位置 (x=0)
            [0, 1, 0, 0, 0, 0],  # 条件2: 起点横向速度 = 0
            [0, 0, 2, 0, 0, 0],  # 条件3: 起点横向加速度 = 0
            [1, X_max, X_max ** 2, X_max ** 3, X_max ** 4, X_max ** 5],  # 条件4: 终点位置 (x=X_max)
            [0, 1, 2 * X_max, 3 * X_max ** 2, 4 * X_max ** 3, 5 * X_max ** 4],  # 条件5: 终点横向速度 = 0
            [0, 0, 2, 6 * X_max, 12 * X_max ** 2, 20 * X_max ** 3]  # 条件6: 终点横向加速度 = 0
        ])

        # 边界条件向量 B: 分别对应 起点y, 起点v, 起点a, 终点y, 终点v, 终点a
        B = np.array([start_y, 0, 0, end_y, 0, 0])

        # 求解线性方程组 A * C = B，得到 6 个系数 (c0 到 c5)
        coeffs = np.linalg.solve(A, B)

        # 在 0 到 X_max 之间均匀生成 N 个纵向采样点
        x_vals = np.linspace(0, X_max, self.N)

        # 代入求出的系数，计算出每个纵向点对应的横向坐标 y
        y_vals = (coeffs[0] + coeffs[1] * x_vals + coeffs[2] * x_vals ** 2 +
                  coeffs[3] * x_vals ** 3 + coeffs[4] * x_vals ** 4 + coeffs[5] * x_vals ** 5)

        # 准备俯仰角旋转矩阵参数
        pitch_rad = np.radians(pitch_deg)
        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)

        final_points = []
        for x, y in zip(x_vals, y_vals):
            # 将 2D 轨迹映射到 3D 物理空间
            # raw_x: 横向偏移; raw_y: 高度(默认为0); raw_z: 纵向深度
            raw_x, raw_y, raw_z = y, 0, x

            # 绕 X 轴旋转 (Pitch 俯仰)，使得曲线不仅是平面的，还能随坡度起伏
            new_x = raw_x
            new_y = raw_y * cos_p + raw_z * sin_p
            new_z = -raw_y * sin_p + raw_z * cos_p

            # Unity 坐标系映射 (保留两位小数节省带宽):
            # new_x 对应 Unity X (左右)
            # new_y 对应 Unity Y (上下)
            # new_z 对应 Unity Z (前后)
            final_points.append([round(new_x, 2), round(new_y, 2), round(new_z, 2)])

        return final_points