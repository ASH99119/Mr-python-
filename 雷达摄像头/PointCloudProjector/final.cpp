//#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "RSLidar.h"
#include "CameraCV.h"
#include <windows.h>
#include "json.hpp"                     //接受python的josn数组
#include <algorithm>                    //计算中位数的头文件



// 使用 nlohmann::json 库
using json = nlohmann::json;


// 目标检测框结构体（用于发给 Python）
struct BBox {
    int xmin;       // 左上角 x 坐标
    int ymin;       // 左上角 y 坐标
    int xmax;       // 右下角 x 坐标
    int ymax;       // 右下角 y 坐标
    int cls;        // 类别
    float conf;     // 置信度
};

// 全局车道距离结构体（中位数距离 + 中心最近点的真实物理坐标 XYZ）
struct DistData {
    float left_dist = -1.0f;                               // 仅需左车道距离

    float mid_dist = -1.0f;                                // 中间车道距离
    float mid_X = 0.0f;    float mid_Y = 0.0f;    float mid_Z = 0.0f;  // 中间车道保留 3D 坐标给 Quest 3

    float right_dist = -1.0f;                              // 仅需右车道距离
};


DistData g_dist_data;          // 存储最新的车道距离
std::mutex dist_data_mtx;      // 距离数据锁

std::vector<BBox> g_bboxes;    // 存储最新的检测框
std::mutex bbox_mtx;            // 检测框数据锁

// 全局变量和缓冲区
cv::Mat g_cam_frame;            // 摄像头图像缓冲区
std::mutex cam_mtx;             // 摄像头图像缓冲区锁
std::atomic<bool> g_isCamReady(false);          // 摄像头图像是否准备好
std::condition_variable camflow;                // 摄像头图像流条件变量


// 点云相关的缓冲区和变量参考你的原有代码
// 这里只举例左摄像头的点云像素缓冲区
Eigen::MatrixXf g_point_cloud_p_left; // 投射到左摄像头的像素坐标
std::mutex pcd_mtx;                   // 点云像素缓冲区锁
std::atomic<bool> g_isPointCloudDataflowReady(false);           // 点云数据是否准备好

cv::Mat cropImageCenter(const cv::Mat& src, int crop_width, int crop_height) {
    // 原始分辨率
    int src_width = src.cols;               // 获取图像的宽度
    int src_height = src.rows;              // 获取图像的高度

    // 如果目标分辨率大于等于原始分辨率，直接返回原图
    if (crop_width >= src_width && crop_height >= src_height)
        return src.clone();

    // 计算裁切区域左上角坐标，使裁切框居中
    int x = max(0, (src_width - crop_width) / 2);
    int y = max(0, (src_height - crop_height) / 2);

    // 限制裁切宽高不越界
    int w = min(crop_width, src_width - x);
    int h = min(crop_height, src_height - y);

    cv::Rect roi(x, y, w, h);
    return src(roi).clone();
}




// 摄像头图像读取线程
void camReader(int cam_id = 0) {
    cv::VideoCapture cap(cam_id);
    if (!cap.isOpened()) {
        std::cerr << "Camera open failed!" << std::endl;
        return;
    }
    // 设置分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 9999);            // 设置摄像头分辨率为最大值，实际分辨率由摄像头硬件决定
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 9999);           // 设置摄像头分辨率为最大值，实际分辨率由摄像头硬件决定

    while (true) {          // 循环读取摄像头图像
        cv::Mat frame;      // 读取一帧图像
        cap >> frame;       // 从摄像头读取图像
        if (frame.empty()) continue;            // 如果读取失败，继续下一帧
        {
            std::lock_guard<std::mutex> lock(cam_mtx);
            g_cam_frame = frame.clone();
            g_isCamReady = true;
        }
        camflow.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 控制频率
    }
}








// 点云生产者线程
void pointCloudWorker() {
    using namespace robosense::lidar;       // 使用 Robosense 的 LiDAR 命名空间

    // 相机内参
    Eigen::Matrix3f K;
    K << 2.284888504404298e+03, 0, 9.652978674936777e+02,
        0, 2.282118185033347e+03, 4.788071028530923e+02,
        0, 0, 1;

    // LiDAR到相机的变换，外参
    Eigen::Matrix<float, 3, 4> lidar_to_cam;
    lidar_to_cam << 0.091961047691452, -0.995691668709137, 0.011885561438362, 0.003652313922745,
        -0.027694109641717, -0.014488957120332, -0.999511433857921, -0.101755656509753,
        0.995377416861887, 0.091586958595532, -0.028907213919318, -0.073324598121015;

    RSDriverParam param;
    param.input_type = InputType::ONLINE_LIDAR;         // 在线 LiDAR 输入
    param.input_param.msop_port = 6699;                 // 设置 msop 端口
    param.input_param.difop_port = 7788;                // 设置 difop 端口
    param.lidar_type = LidarType::RSHELIOS_16P;         // 设置 LiDAR 类型

    LidarDriver<PointCloudMsg> driver;
    driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback);        // 注册点云回调函数
    driver.regExceptionCallback(exceptionCallback);                                                                     // 注册异常回调函数
    if (!driver.init(param)) return;                                                                                    // 初始化驱动
    driver.start();                                                                                                     // 启动驱动

    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    std::shared_ptr<PointCloudMsg> msg;
    Eigen::MatrixXf rawPointCloud;

    while (true) {
        msg = stuffed_cloud_queue.popWait(); // 获取点云消息
        if (!msg) continue;

        rawPointCloud = convertPointCloudToEigenMatrix(msg); // N x 3 点云

        // 投影到相机像素平面并生成 pcd_pixel (N x 5)
        Eigen::MatrixXf pcd_pixel(rawPointCloud.rows(), 5);

		// 遍历每个点云点，进行投影
        for (int i = 0; i < rawPointCloud.rows(); ++i) {
			Eigen::Vector3f pt_lidar = rawPointCloud.row(i).head(3);        // 获取 LiDAR 坐标系下的点
			Eigen::Vector3f pt_cam = lidar_to_cam.leftCols(3) * pt_lidar + lidar_to_cam.col(3).head(3);     // 转换到相机坐标系

            // 过滤掉 Z<=0 的点
            if (pt_cam[2] <= 0) {
                pcd_pixel.row(i).setConstant(-1); // 标记不可用
                continue;
            }

			// 投影到像素平面
            float u = K(0, 0) * pt_cam[0] / pt_cam[2] + K(0, 2);
            float v = K(1, 1) * pt_cam[1] / pt_cam[2] + K(1, 2);

            // 过滤不在图像内的点
            if (u < 0 || u >= 1920 || v < 0 || v >= 1080) {
                pcd_pixel.row(i).setConstant(-1);
                continue;
            }

            pcd_pixel(i, 0) = u;
            pcd_pixel(i, 1) = v;
            pcd_pixel(i, 2) = pt_cam[0]; // X
            pcd_pixel(i, 3) = pt_cam[1]; // Y
            pcd_pixel(i, 4) = pt_cam[2]; // Z
        }

        // 写入双缓冲
        writeSharedVariable(
            pcd_pixel,
            g_point_cloud_p_buffers_left.buffer1,
            g_point_cloud_p_buffers_left.buffer2,
            g_point_cloud_p_buffers_left.using_buffer1,
            mtx
        );

        // 通知 consumer 数据准备好
        if (!g_isPointCloudDataflowReady) {
            std::lock_guard<std::mutex> lock(mtx);
            g_isPointCloudDataflowReady = true;
            dataflow.notify_one();
        }
    }
}





// 后处理线程：在摄像头图像上绘制点云
void consumer() {
    const int WIDTH = 1920;                                         // 原始图像宽度
    const int HEIGHT = 1080;                                        // 原始图像高度

    {
        std::unique_lock<std::mutex> cam_lock(cam_mtx);             // 等待摄像头图像准备好
        camflow.wait(cam_lock, [] { return g_isCamReady.load(); });     // 等待条件变量通知
    }

    {
        std::unique_lock<std::mutex> pcd_lock(pcd_mtx);
        dataflow.wait(pcd_lock, [] { return g_isPointCloudDataflowReady.load(); });
    }

    cv::Mat img;
    Eigen::MatrixXf pcd_pixel;

    while (true) {                  // 循环处理每一帧数据
        // 读取YOLO检测框
        std::vector<BBox> boxes;            // 存储当前帧的检测框
        {
            std::lock_guard<std::mutex> lock(bbox_mtx);                 // 锁住检测框数据
            boxes = g_bboxes;                                           // 读取当前帧的检测框
        }

        // 读取摄像头图像
        {
            std::lock_guard<std::mutex> cam_lock(cam_mtx);              // 锁住摄像头图像数据
            img = g_cam_frame.clone();                                  // 读取当前帧的摄像头图像
        }

        // 读取投影点云
        {
            std::lock_guard<std::mutex> pcd_lock(pcd_mtx);              // 锁住点云数据
            readSharedVariable(                                         // 读取投影点云数据
                pcd_pixel,
                g_point_cloud_p_buffers_left.buffer1,
                g_point_cloud_p_buffers_left.buffer2,
                g_point_cloud_p_buffers_left.using_buffer1,
                mtx
            );
        }



        // 记录当前帧三个车道的最近距离（中位数）及其对应的中心物理坐标
        float cur_left_dist = -1.0f, cl_X = 0, cl_Y = 0, cl_Z = 0;
        float cur_mid_dist = -1.0f, cm_X = 0, cm_Y = 0, cm_Z = 0;
        float cur_right_dist = -1.0f, cr_X = 0, cr_Y = 0, cr_Z = 0;

        // 遍历每个bbox
        for (auto& box : boxes)
        {
            cv::rectangle(img, cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax), cv::Scalar(255, 0, 0), 2);

            float center_x = (box.xmin + box.xmax) / 2.0f;
            float center_y = (box.ymin + box.ymax) / 2.0f;

            // 用于存所有点（u, v, X, Y, Z）来算中位数
            std::vector<std::tuple<float, float, float, float, float>> points;

            float min_pixel_dist = 999999.0f;
            float best_X = 0.0f, best_Y = 0.0f, best_Z = -1.0f;
            float best_u = 0.0f, best_v = 0.0f;
            bool found_closest = false;

            for (int j = 0; j < pcd_pixel.rows(); ++j) {
                float u = pcd_pixel(j, 0);
                float v = pcd_pixel(j, 1);

                if (u > box.xmin && u < box.xmax && v > box.ymin && v < box.ymax) {
                    float X = pcd_pixel(j, 2); // 真实物理 X
                    float Y = pcd_pixel(j, 3); // 真实物理 Y
                    float Z = pcd_pixel(j, 4); // 真实物理 Z (深度)

                    if (Z > 0.5 && Z < 50) {
                        // 1. 把所有点存起来用于算中位数
                        points.emplace_back(u, v, X, Y, Z);

                        // 2. 同时计算当前点与框中心的 2D 像素距离
                        float dx = u - center_x;
                        float dy = v - center_y;
                        float pixel_dist = std::sqrt(dx * dx + dy * dy);

                        // 找到最靠近框中心的点，记录它的物理坐标
                        if (pixel_dist < min_pixel_dist) {
                            min_pixel_dist = pixel_dist;
                            best_u = u; best_v = v;
                            best_X = X; best_Y = Y; best_Z = Z;
                            found_closest = true;
                        }

                        // 小绿点（原有）
                        cv::circle(img, cv::Point(int(u), int(v)), 3, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }

            if (!points.empty()) {
                // 按深度Z排序找中位数点
                std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
                    return std::get<4>(a) < std::get<4>(b); // 索引4是Z
                    });
                auto mid = points[points.size() / 2];
                float z_mid = std::get<4>(mid); // 中位数距离 (用于测距)

                // 画红点表示中位数点
                cv::circle(img, cv::Point(int(std::get<0>(mid)), int(std::get<1>(mid))), 8, cv::Scalar(0, 0, 255), -1);

                // 画黄点表示框中心最近的那个物理点 (用于轨迹修正)
                if (found_closest) {
                    cv::circle(img, cv::Point(int(best_u), int(best_v)), 8, cv::Scalar(0, 255, 255), -1);
                }

                // 显示距离(中位数)
                char text[64];
                snprintf(text, sizeof(text), "Dist: %.2f m", z_mid);
                cv::putText(img, text, cv::Point(box.xmin, box.ymin - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                // 根据车道分类，更新最短距离和对应的物理坐标
                if (center_x < 640) {
                    if (cur_left_dist < 0 || z_mid < cur_left_dist) {
                        cur_left_dist = z_mid; // 使用中位数判断哪辆车最近
                        if (found_closest) { cl_X = best_X; cl_Y = best_Y; cl_Z = best_Z; }
                    }
                }
                else if (center_x > 1280) {
                    if (cur_right_dist < 0 || z_mid < cur_right_dist) {
                        cur_right_dist = z_mid;
                        if (found_closest) { cr_X = best_X; cr_Y = best_Y; cr_Z = best_Z; }
                    }
                }
                else {
                    if (cur_mid_dist < 0 || z_mid < cur_mid_dist) {
                        cur_mid_dist = z_mid;
                        if (found_closest) { cm_X = best_X; cm_Y = best_Y; cm_Z = best_Z; }
                    }
                }
            }
        }

        // 写入全局变量
        {
            std::lock_guard<std::mutex> lk(dist_data_mtx);
            g_dist_data.left_dist = cur_left_dist; 
            g_dist_data.mid_dist = cur_mid_dist;   g_dist_data.mid_X = cm_X;  g_dist_data.mid_Y = cm_Y;  g_dist_data.mid_Z = cm_Z;
            g_dist_data.right_dist = cur_right_dist; 
        }




        // 显示缩小图像
        cv::resize(img, img, cv::Size(WIDTH / 2, HEIGHT / 2));
        cv::imshow("PointCloud Overlay", img);

        if (cv::waitKey(1) == 27) // ESC退出
            break;

        // 重置条件
        g_isCamReady = false;
        g_isPointCloudDataflowReady = false;
    }
}







// YOLO管道读取线程，用于接收Python端YOLO检测框和图像，并发送距离数据回去
void yoloPipeReader() {
    std::cout << "YOLO thread started\n";
    HANDLE hPipe;



    // 改为 PIPE_ACCESS_DUPLEX，支持双向通信读写！
	//这里在windows上创建一个叫 yolo_pipe命名管道，等待Python端的YOLO进程连接
    hPipe = CreateNamedPipe(
        TEXT("\\\\.\\pipe\\yolo_pipe"),
		PIPE_ACCESS_DUPLEX,     // 双向通信
        PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
        1,
        1024 * 1024,
        1024 * 1024,
        0,
        NULL
    );

	// 检查管道是否创建成功
    if (hPipe == INVALID_HANDLE_VALUE) {
        std::cout << "CreateNamedPipe failed\n";
        return;
    }
    
    std::cout << "Waiting Python YOLO...\n";
    
	// 等待Python端连接
    BOOL connected = ConnectNamedPipe(hPipe, NULL) ?
        TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

	// 检查连接是否成功
    if (!connected) {
        std::cout << "Pipe connect failed\n";
        return;
    }

    std::cout << "YOLO Connected\n";


	// 读取来自Python的YOLO检测框和图像的循环
    while (true) {
        DWORD img_size;
        DWORD bytesRead;

		// 读取图像大小，然后根据大小读取图像数据
        ReadFile(hPipe, &img_size, sizeof(img_size), &bytesRead, NULL);
        std::vector<uchar> img_buffer(img_size);

        // 读取图像
        ReadFile(hPipe, img_buffer.data(), img_size, &bytesRead, NULL);
        cv::Mat img = cv::imdecode(img_buffer, cv::IMREAD_COLOR);

        {
            std::lock_guard<std::mutex> lock(cam_mtx);
            g_cam_frame = img.clone();
            g_isCamReady = true;
        }
        camflow.notify_one();

        DWORD bbox_size;
        // 读取bbox大小
        ReadFile(hPipe, &bbox_size, sizeof(bbox_size), &bytesRead, NULL);

        // 读取bbox数据
        std::vector<char> bbox_buffer(bbox_size);
        ReadFile(hPipe, bbox_buffer.data(), bbox_size, &bytesRead, NULL);

        std::string data(bbox_buffer.begin(), bbox_buffer.end());
        // std::cout << "recv bbox: " << data << std::endl; // 可以注释掉避免刷屏

        try {
            std::lock_guard<std::mutex> lock(bbox_mtx);
            g_bboxes.clear();
            auto j = json::parse(data);

            for (auto& item : j) {
                if (item.size() < 6) continue;
                BBox box;
                box.xmin = item[0];
                box.ymin = item[1];
                box.xmax = item[2];
                box.ymax = item[3];
                box.cls = item[4];
                box.conf = item[5];
                g_bboxes.push_back(box);
            }
        }
        catch (...) {
            std::cout << "JSON parse error!" << std::endl;
        }




        // 按照新的结构打包
        json reply_j;
        {
            std::lock_guard<std::mutex> lk(dist_data_mtx);
            // 左右车道：只传深度
            reply_j["left_dist"] = g_dist_data.left_dist;

            // 中间车道：深度 + 完整 3D 坐标（用于 MR 空间渲染）
            reply_j["mid_dist"] = g_dist_data.mid_dist;
            reply_j["mid_X"] = g_dist_data.mid_X;
            reply_j["mid_Y"] = g_dist_data.mid_Y;
            reply_j["mid_Z"] = g_dist_data.mid_Z;

            // 右侧车道：只传深度
            reply_j["right_dist"] = g_dist_data.right_dist;
        }


        std::string reply_str = reply_j.dump();
        DWORD reply_len = static_cast<DWORD>(reply_str.size());
        DWORD bytesWritten;

        // 按照与 Python 约定的协议，先写 4 字节长度，再写内容
        WriteFile(hPipe, &reply_len, sizeof(reply_len), &bytesWritten, NULL);
        WriteFile(hPipe, reply_str.c_str(), reply_len, &bytesWritten, NULL);
    }

    CloseHandle(hPipe);
}

int main() {
    std::thread lidar_thread(pointCloudWorker);
    //std::thread cam_thread(camReader, 1); //注意摄像头ID需要根据实际情况调整
    std::thread consumer_thread(consumer);
    std::thread yolo_thread(yoloPipeReader);

    lidar_thread.join();
    //cam_thread.join();
    consumer_thread.join();
    yolo_thread.join();

    return 0;
}
