//#include <iostream>
//#include <thread>
//#include <mutex>
//#include <condition_variable>
//#include <atomic>
//#include <opencv2/opencv.hpp>
//#include "RSLidar.h"
//#include "CameraCV.h"
//#include <windows.h>
//#include "json.hpp"							//接受python的josn数组
//#include <algorithm>						//计算中位数
//
//using json = nlohmann::json;
//HANDLE g_pipe = NULL;
//std::mutex pipe_mtx;
//
//struct BBox {
//	int xmin;
//	int ymin;
//	int xmax;
//	int ymax;
//	int cls;
//	float conf;
//};
//
//std::vector<BBox> g_bboxes;
//std::mutex bbox_mtx;
//
//// 全局变量和缓冲区（示例，实际根据你的结构调整）
//cv::Mat g_cam_frame;
//std::mutex cam_mtx;
//std::atomic<bool> g_isCamReady(false);
//std::condition_variable camflow;
//
//// 点云相关的缓冲区和变量参考你的原有代码
//// 这里只举例左摄像头的点云像素缓冲区
//Eigen::MatrixXf g_point_cloud_p_left; // 投射到左摄像头的像素坐标
//std::mutex pcd_mtx;
//std::atomic<bool> g_isPointCloudDataflowReady(false);
//
//cv::Mat cropImageCenter(const cv::Mat& src, int crop_width, int crop_height) {
//
//	/**
//	  * @brief 从输入图片中心裁切指定分辨率的图片
//	  * @param src 原始图片
//	  * @param crop_width 裁切目标宽度
//	  * @param crop_height 裁切目标高度
//	  * @return 裁切后的图片
//	*/
//
//
//	// 原始分辨率
//	int src_width = src.cols;
//	int src_height = src.rows;
//
//	// 如果目标分辨率大于等于原始分辨率，直接返回原图
//	if (crop_width >= src_width && crop_height >= src_height)
//		return src.clone();
//
//	// 计算裁切区域左上角坐标，使裁切框居中
//	int x = max(0, (src_width - crop_width) / 2);
//	int y = max(0, (src_height - crop_height) / 2);
//
//	// 限制裁切宽高不越界
//	int w = min(crop_width, src_width - x);
//	int h = min(crop_height, src_height - y);
//
//	cv::Rect roi(x, y, w, h);
//	return src(roi).clone();
//}
//
//
//// 摄像头图像读取线程
//void camReader(int cam_id = 0) {
//	cv::VideoCapture cap(cam_id);
//	if (!cap.isOpened()) {
//		std::cerr << "Camera open failed!" << std::endl;
//		return;
//	}
//	// 设置分辨率
//	// 这里的值大于最大CMOS分辨率即可
//	cap.set(cv::CAP_PROP_FRAME_WIDTH, 9999);
//	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 9999);
//
//
//	while (true) {
//		cv::Mat frame;
//		cap >> frame;
//		if (frame.empty()) continue;
//		{
//			std::lock_guard<std::mutex> lock(cam_mtx);
//			g_cam_frame = frame.clone();
//			g_isCamReady = true;
//		}
//		camflow.notify_one();
//		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 控制频率
//	}
//}
//
//// 点云生产者线程
//void pointCloudWorker() {
//	using namespace robosense::lidar;
//
//	// 相机内参
//	Eigen::Matrix3f K;
//	K << 2.284888504404298e+03, 0, 9.652978674936777e+02,
//		0, 2.282118185033347e+03, 4.788071028530923e+02,
//		0, 0, 1;
//
//	// LiDAR到相机的变换
//	Eigen::Matrix<float, 3, 4> lidar_to_cam;
//	lidar_to_cam << 0.091961047691452, -0.995691668709137, 0.011885561438362, 0.003652313922745,
//		-0.027694109641717, -0.014488957120332, -0.999511433857921, -0.101755656509753,
//		0.995377416861887, 0.091586958595532, -0.028907213919318, -0.073324598121015;
//
//	RSDriverParam param;
//	param.input_type = InputType::ONLINE_LIDAR;
//	param.input_param.msop_port = 6699;
//	param.input_param.difop_port = 7788;
//	param.lidar_type = LidarType::RSHELIOS_16P;
//
//	LidarDriver<PointCloudMsg> driver;
//	driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback);
//	driver.regExceptionCallback(exceptionCallback);
//	if (!driver.init(param)) return;
//	driver.start();
//
//	RS_TITLE << "------------------------------------------------------" << RS_REND;
//	RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
//	RS_TITLE << "------------------------------------------------------" << RS_REND;
//
//	std::shared_ptr<PointCloudMsg> msg;
//	Eigen::MatrixXf rawPointCloud;
//
//	while (true) {
//		msg = stuffed_cloud_queue.popWait(); // 获取点云消息
//		if (!msg) continue;
//
//		rawPointCloud = convertPointCloudToEigenMatrix(msg); // N x 3 点云
//
//		// 投影到相机像素平面并生成 pcd_pixel (N x 5)
//		Eigen::MatrixXf pcd_pixel(rawPointCloud.rows(), 5);
//
//		for (int i = 0; i < rawPointCloud.rows(); ++i) {
//			Eigen::Vector3f pt_lidar = rawPointCloud.row(i).head(3);
//			Eigen::Vector3f pt_cam = lidar_to_cam.leftCols(3) * pt_lidar + lidar_to_cam.col(3).head(3);
//
//			// 过滤掉 Z<=0 的点
//			if (pt_cam[2] <= 0) {
//				pcd_pixel.row(i).setConstant(-1); // 标记不可用
//				continue;
//			}
//
//			float u = K(0, 0) * pt_cam[0] / pt_cam[2] + K(0, 2);
//			float v = K(1, 1) * pt_cam[1] / pt_cam[2] + K(1, 2);
//
//			// 过滤不在图像内的点
//			if (u < 0 || u >= 1920 || v < 0 || v >= 1080) {
//				pcd_pixel.row(i).setConstant(-1);
//				continue;
//			}
//
//			pcd_pixel(i, 0) = u;
//			pcd_pixel(i, 1) = v;
//			pcd_pixel(i, 2) = pt_cam[0]; // X
//			pcd_pixel(i, 3) = pt_cam[1]; // Y
//			pcd_pixel(i, 4) = pt_cam[2]; // Z
//		}
//
//		// 写入双缓冲
//		writeSharedVariable(
//			pcd_pixel,
//			g_point_cloud_p_buffers_left.buffer1,
//			g_point_cloud_p_buffers_left.buffer2,
//			g_point_cloud_p_buffers_left.using_buffer1,
//			mtx
//		);
//
//		// 通知 consumer 数据准备好
//		if (!g_isPointCloudDataflowReady) {
//			std::lock_guard<std::mutex> lock(mtx);
//			g_isPointCloudDataflowReady = true;
//			dataflow.notify_one();
//		}
//	}
//}
//
//// 后处理线程：在摄像头图像上绘制点云
//void consumer() {
//
//	const int WIDTH = 1920;
//	const int HEIGHT = 1080;
//
//	{
//		std::unique_lock<std::mutex> cam_lock(cam_mtx);
//		camflow.wait(cam_lock, [] { return g_isCamReady.load(); });
//	}
//
//	{
//		std::unique_lock<std::mutex> pcd_lock(pcd_mtx);
//		dataflow.wait(pcd_lock, [] { return g_isPointCloudDataflowReady.load(); });
//	}
//
//	cv::Mat img;
//	Eigen::MatrixXf pcd_pixel;
//	
//	while (true) {
//
//
//
//
//		// 读取YOLO检测框
//		std::vector<BBox> boxes;
//		{
//			std::lock_guard<std::mutex> lock(bbox_mtx);
//			boxes = g_bboxes;
//		}
//
//
//		// 读取摄像头图像
//		{
//			std::lock_guard<std::mutex> cam_lock(cam_mtx);
//			img = g_cam_frame.clone();
//		}
//
//
//		// 读取投影点云
//		{
//			std::lock_guard<std::mutex> pcd_lock(pcd_mtx);
//			readSharedVariable(
//				pcd_pixel,
//				g_point_cloud_p_buffers_left.buffer1,
//				g_point_cloud_p_buffers_left.buffer2,
//				g_point_cloud_p_buffers_left.using_buffer1,
//				mtx
//			);
//		}
//
//		float left_dist = 9999.0f;
//		float mid_dist = 9999.0f;    //假设所有车距离为9999，后续更新，找最近
//		float right_dist = 9999.0f;
//
//		bool has_left = false;
//		bool has_mid = false;
//		bool has_right = false;
//
//		// 遍历每个bbox
//		for (auto& box : boxes)
//		{
//			// 绘制bbox
//			cv::rectangle(
//				img,
//				cv::Point(box.xmin, box.ymin),
//				cv::Point(box.xmax, box.ymax),
//				cv::Scalar(255, 0, 0),
//				2
//			);
//
//			////平均值版本
//			// 
//			// //bbox 内 LiDAR 坐标累计
//			//float sumX = 0, sumY = 0, sumZ = 0;
//			//int count = 0;
//
//			//for (int j = 0; j < pcd_pixel.rows(); ++j) {
//			//	float u = pcd_pixel(j, 0);
//			//	float v = pcd_pixel(j, 1);
//
//			//	// 检查点是否在bbox内
//			//	if (u > box.xmin && u < box.xmax && v > box.ymin && v < box.ymax) {
//			//		float X = pcd_pixel(j, 2);
//			//		float Y = pcd_pixel(j, 3);
//			//		float Z = pcd_pixel(j, 4);
//
//			//		sumX += X;
//			//		sumY += Y;
//			//		sumZ += Z;
//			//		count++;
//
//			//		// 绘制点投影
//			//		cv::circle(img, cv::Point(int(u), int(v)), 3, cv::Scalar(0, 255, 0), -1);
//			//	}
//			//}
//
//			//// 显示平均LiDAR坐标
//			//if (count > 0) {
//			//	char text[64];
//			//	snprintf(text, sizeof(text), "LiDAR:(%.2f,%.2f,%.2f)", sumX / count, sumY / count, sumZ / count);
//			//	cv::putText(
//			//		img,
//			//		text,
//			//		cv::Point(box.xmin, box.ymin - 5),
//			//		cv::FONT_HERSHEY_SIMPLEX,
//			//		0.5,
//			//		cv::Scalar(0, 255, 255),
//			//		2
//			//	);
//			//}
//
//			// 
//			// 
//
//			/*中位数版本111111111111111*/
//			//std::vector<std::tuple<float, float, float>> points; // u, v, Z
//
//			//for (int j = 0; j < pcd_pixel.rows(); ++j) {
//			//	float u = pcd_pixel(j, 0);
//			//	float v = pcd_pixel(j, 1);
//
//			//	if (u > box.xmin && u < box.xmax && v > box.ymin && v < box.ymax) {
//			//		float Z = pcd_pixel(j, 4);
//
//			//		if (Z > 0.5 && Z < 50) {
//			//			points.emplace_back(u, v, Z);
//			//		}
//
//			//		// 小绿点（原有）
//			//		cv::circle(img, cv::Point(int(u), int(v)), 3, cv::Scalar(0, 255, 0), -1);
//			//	}
//			//}
//
//			//if (!points.empty()) {
//
//			//	// 按Z排序
//			//	std::sort(points.begin(), points.end(),
//			//		[](const auto& a, const auto& b) {
//			//			return std::get<2>(a) < std::get<2>(b);
//			//		});
//
//			//	// 中位数点
//			//	auto mid = points[points.size() / 2];
//
//			//	float u_mid = std::get<0>(mid);
//			//	float v_mid = std::get<1>(mid);
//			//	float z_mid = std::get<2>(mid);
//
//			//	// 画红点（中位数点）
//			//	cv::circle(
//			//		img,
//			//		cv::Point(int(u_mid), int(v_mid)),
//			//		8,                      // 半径（大一点）
//			//		cv::Scalar(0, 0, 255),  // 红色
//			//		-1
//			//	);
//
//			//	// 显示距离
//			//	char text[64];
//			//	snprintf(text, sizeof(text), "Dist: %.2f m", z_mid);
//
//			//	cv::putText(
//			//		img,
//			//		text,
//			//		cv::Point(box.xmin, box.ymin - 5),
//			//		cv::FONT_HERSHEY_SIMPLEX,
//			//		0.6,
//			//		cv::Scalar(0, 255, 255),
//			//		2
//			//	);
//			//}
//				//中位数版本2，取所有bbox中点平均值，让点几乎不动
//			//std::vector<std::tuple<float, float, float>> points; // u, v, Z
//
//			//for (int j = 0; j < pcd_pixel.rows(); ++j) {
//			//	float u = pcd_pixel(j, 0);
//			//	float v = pcd_pixel(j, 1);
//
//			//	if (u > box.xmin && u < box.xmax && v > box.ymin && v < box.ymax) {
//			//		float Z = pcd_pixel(j, 4);
//
//			//		if (Z > 0.5 && Z < 50) {
//			//			points.emplace_back(u, v, Z);
//			//		}
//
//			//		// 小绿点（原有）
//			//		cv::circle(img, cv::Point(int(u), int(v)), 3, cv::Scalar(0, 255, 0), -1);
//			//	}
//			//}
//
//			//if (!points.empty()) {
//
//			//	// 按Z排序
//			//	std::sort(points.begin(), points.end(),
//			//		[](const auto& a, const auto& b) {
//			//			return std::get<2>(a) < std::get<2>(b);
//			//		});
//
//			//	// 中位数点
//			//	// 算中位数Z
//			//	auto mid = points[points.size() / 2];
//			//	float z_mid = std::get<2>(mid);
//
//			//	// 再算所有点的平均位置
//			//	float sum_u = 0, sum_v = 0;
//
//			//	for (auto& p : points) {
//			//		sum_u += std::get<0>(p);
//			//		sum_v += std::get<1>(p);
//			//	}
//
//			//	float u_avg = sum_u / points.size();
//			//	float v_avg = sum_v / points.size();
//
//			//	//用平均位置画红点-----让点尽量不动
//			//	cv::circle(
//			//		img,
//			//		cv::Point(int(u_avg), int(v_avg)),
//			//		8,
//			//		cv::Scalar(0, 0, 255),
//			//		-1
//			//	);
//
//			//	// 显示距离
//			//	char text[64];
//			//	snprintf(text, sizeof(text), "Dist: %.2f m", z_mid);
//
//			//	cv::putText(
//			//		img,
//			//		text,
//			//		cv::Point(box.xmin, box.ymin - 5),
//			//		cv::FONT_HERSHEY_SIMPLEX,
//			//		0.7,
//			//		cv::Scalar(0, 255, 255),
//			//		2
//			//	);
//			//}	
//			std::vector<std::tuple<float, float, float>> points; // u, v, Z
//			for (int j = 0; j < pcd_pixel.rows(); ++j) 
//			{
//				float u = pcd_pixel(j, 0);
//				float v = pcd_pixel(j, 1);
//
//				if (u > box.xmin && u < box.xmax && v > box.ymin && v < box.ymax) 
//				{
//					float Z = pcd_pixel(j, 4);
//
//					if (Z > 0.5 && Z < 50) 
//					{
//						points.emplace_back(u, v, Z);
//					}
//
//					cv::circle(img, cv::Point(int(u), int(v)), 3, cv::Scalar(0, 255, 0), -1);
//				}
//			}
//
//			if (!points.empty()) 
//			{
//
//				std::sort(points.begin(), points.end(),
//					[](const auto& a, const auto& b) {
//						return std::get<2>(a) < std::get<2>(b);
//					});
//
//				int n = points.size();
//
//				// 取中间40%~60%
//				int start = n * 0.4;
//				int end = n * 0.6;
//
//				float sum_u = 0;
//				float sum_v = 0;
//				float z_mid = 0;
//				int count = 0;
//
//				for (int i = start; i <= end; i++) 
//				{
//					sum_u += std::get<0>(points[i]);
//					sum_v += std::get<1>(points[i]);
//					//z_mid = std::get<2>(points[i]); // 取第三个数字
//					z_mid += std::get<2>(points[i]);  // 取平均Z
//					count++;
//				}
//
//				if (count == 0) continue;
//
//				float u_avg = sum_u / count;
//				float v_avg = sum_v / count;
//				z_mid /= count;
//				int WIDTH = 1920;
//
//				// 判断车道
//				if (u_avg < WIDTH / 3)
//				{
//					if (z_mid < left_dist)
//					{
//						left_dist = z_mid;
//						has_left = true;
//					}
//				}
//				else if (u_avg < 2 * WIDTH / 3)
//				{
//					if (z_mid < mid_dist)
//					{
//						mid_dist = z_mid;
//						has_mid = true;
//					}
//				}
//				else
//				{
//					if (z_mid < right_dist)
//					{
//						right_dist = z_mid;
//						has_right = true;
//					}
//				}
//				//这个为测单车道的时候
//				//if (z_mid < min_dist)
//				//{
//				//	min_dist = z_mid;
//				//	has_mid = true;
//				//}
//
//				// 画红点
//				cv::circle(
//					img,
//					cv::Point(int(u_avg), int(v_avg)),
//					8,
//					cv::Scalar(0, 0, 255),
//					-1
//				);
//
//
//				// 显示距离
//				char text[64];
//				snprintf(text, sizeof(text), "Dist: %.2f m", z_mid);
//
//				cv::putText(
//					img,
//					text,
//					cv::Point(box.xmin, box.ymin - 5),
//					cv::FONT_HERSHEY_SIMPLEX,
//					0.7,
//					cv::Scalar(0, 255, 255),
//					2
//				);
//			}
//		}
//		//控制发送频率
//		static auto last_send = std::chrono::steady_clock::now();
//		auto now = std::chrono::steady_clock::now();
//		// =======================
//		// 发最近的车距离
//		// =======================
//		//if (has_target &&
//		//	std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() > 100)
//		//{
//		//	last_send = now;
//
//		//	if (g_pipe != NULL)
//		//	{
//		//		json result;
//		//		result["distance"] = min_dist;
//
//		//		std::string s = result.dump();
//
//		//		DWORD len = s.size();
//		//		DWORD written;
//
//		//		std::lock_guard<std::mutex> lock(pipe_mtx);
//
//		//		WriteFile(g_pipe, &len, 4, &written, NULL);
//		//		WriteFile(g_pipe, s.c_str(), len, &written, NULL);
//		//	}
//		//}
//
//		//发送三车道
//		if ((has_left || has_mid || has_right) &&
//			std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() > 100)
//		{
//			last_send = now;
//
//			if (g_pipe != NULL)
//			{
//				json result;
//
//				result["left"] = has_left ? left_dist : -1;    //-1代表无车
//				result["middle"] = has_mid ? mid_dist : -1;
//				result["right"] = has_right ? right_dist : -1;
//
//				std::string s = result.dump();
//
//				DWORD len = s.size();
//				DWORD written;
//
//				std::lock_guard<std::mutex> lock(pipe_mtx);
//
//				WriteFile(g_pipe, &len, 4, &written, NULL);
//				WriteFile(g_pipe, s.c_str(), len, &written, NULL);
//			}
//		}
//
//
//
//
//
//
//
//		// 显示缩小图像
//		cv::resize(img, img, cv::Size(WIDTH / 2, HEIGHT / 2));
//		cv::imshow("PointCloud Overlay", img);
//
//		if (cv::waitKey(1) == 27) // ESC退出
//			break;
//			#
//		// 重置条件
//		g_isCamReady = false;
//		g_isPointCloudDataflowReady = false;
//	}
//
//}
//
//void yoloPipeReader() {
//
//	std::cout << "YOLO thread started\n";     //代表开始运行管道接受程序
//
//	HANDLE hPipe;
//
//
//	hPipe = CreateNamedPipe(
//		TEXT("\\\\.\\pipe\\yolo_pipe"),						//创建管道名字
//		//PIPE_ACCESS_INBOUND,								//只读，该程序只能读不能写，python那方在写（代表单向传输）
//		PIPE_ACCESS_DUPLEX,									//双向传输
//		PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,    //字节流，等待模式	
//		1,													//最多一个连接
//		1024 * 1024,										//输出缓存
//		1024 * 1024,										//输入缓存
//		0,
//		NULL
//	);
//
//
//	if (hPipe == INVALID_HANDLE_VALUE) {					//下面三行来看管道是否创建成功
//		std::cout << "CreateNamedPipe failed\n";
//		return;
//	}
//
//	std::cout << "Waiting Python YOLO...\n";				
//
//	BOOL connected = ConnectNamedPipe(hPipe, NULL) ?
//		TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);			//等待连接
//
//	if (!connected) {										
//		std::cout << "Pipe connect failed\n";
//		return;
//	}
//
//	std::cout << "YOLO Connected\n";
//
//	g_pipe = hPipe;
//
//	while (true) {							//循环一直为真代表一直接受数据
//
//		DWORD img_size;						//定义变量用来存图片大小
//		DWORD bytesRead;					//定义变量用来存读到多少数据
//
//		// 读取图像大小
//		ReadFile(hPipe, &img_size, sizeof(img_size), &bytesRead, NULL);			//从传输管道读4个字节（python定义了一个图片为4字节）放进img_size
//
//		std::vector<uchar> img_buffer(img_size);								//创建一块内存，大小刚好等于图片大小
//
//		// 读取图像
//		ReadFile(hPipe, img_buffer.data(), img_size, &bytesRead, NULL);			//从管道里读取整张图片  放进img_buffer
//
//		cv::Mat img = cv::imdecode(img_buffer, cv::IMREAD_COLOR);				//把jpg字节数据，还原成 OpenCV 图片
//
//		{
//			std::lock_guard<std::mutex> lock(cam_mtx);
//			g_cam_frame = img.clone();
//			g_isCamReady = true;
//		}
//
//		camflow.notify_one();
//
//
//		DWORD bbox_size;					//定义变量来存：检测框json的长度
//
//		// 读取bbox大小
//		ReadFile(hPipe, &bbox_size, sizeof(bbox_size), &bytesRead, NULL); //从管道读4个字节得到json数据有多长
//
//		// 读取bbox数据
//		std::vector<char> bbox_buffer(bbox_size);                         //创建一块内存，存json字符串
//		ReadFile(hPipe, bbox_buffer.data(), bbox_size, &bytesRead, NULL); //读取json数据  放进bbox_buffer
//
//		std::string data(bbox_buffer.begin(), bbox_buffer.end());    //把字节转成字符串
//
//		std::cout << "recv bbox: " << data << std::endl;
//
//		std::lock_guard<std::mutex> lock(bbox_mtx);
//		g_bboxes.clear();
//
//		try {													//尝试解析json，出错不会崩程序
//			auto j = json::parse(data);							//把字符串解析成json
//
//			for (auto& item : j) {								//遍历每一个检测框
//				if (item.size() < 6) continue;					//数据不够6个数字，跳过---对应xmin, ymin, xmax, ymax, 类别，置信度
//
//				BBox box;
//				box.xmin = item[0];
//				box.ymin = item[1];
//				box.xmax = item[2];
//				box.ymax = item[3];
//				box.cls = item[4];
//				box.conf = item[5];
//
//				g_bboxes.push_back(box);
//			}
//
//			std::cout << "bbox count: " << g_bboxes.size() << std::endl;
//		}
//		catch (...) {
//			std::cout << "JSON parse error!" << std::endl;
//		}
//	}
//
//	CloseHandle(hPipe);
//}
//
//int main() {
//	std::thread lidar_thread(pointCloudWorker);
//	//std::thread cam_thread(camReader, 1); //注意摄像头ID需要根据实际情况调整
//	std::thread consumer_thread(consumer);
//	std::thread yolo_thread(yoloPipeReader);
//
//	lidar_thread.join();
//	//cam_thread.join();
//	consumer_thread.join();
//	yolo_thread.join();
//
//	return 0;
//}