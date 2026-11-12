#include "CameraCV.h"

//yolo图像帧
ImageBuffer g_image_buffers_L;
ImageBuffer g_image_buffers_R;

//yolo识别结果
MartixBuffer g_yoloresults_buffers_L;
MartixBuffer g_yoloresults_buffers_R; //这个只用于老yolo端，存yolo的result，也就是识别结果矩阵

//yolop识别结果
MartixBuffer g_yolopPred_buffers_L;
MartixBuffer g_yolopPred_buffers_R; //这个只用于yolop端，存yolop的识别结果矩阵
MartixBuffer g_yolopDrivableArea_buffers_L;
MartixBuffer g_yolopDrivableArea_buffers_R; //这个只用于yolop端，存yolop的可行驶区域矩阵
MartixBuffer g_yolopLaneline_buffers_L;
MartixBuffer g_yolopLaneline_buffers_R; //这个只用于yolop端，存yolop的车道线矩阵



Eigen::MatrixXf deserialize_to_eigen_matrix_with_shape(const char* resultBuffer, size_t buffer_size) {
	/*
		将yolo识别结果result比特流反序列化为eigen矩阵
		params:
			resultBuffer:result比特流
			buffer_size:比特流大小
		return:
			matrix:eigen矩阵

	*/

	// 确保缓冲区大小足够
	if (buffer_size < 2 * sizeof(int)) {
		std::cerr << "Buffer size too small for reading dimensions." << std::endl;
		throw std::runtime_error("Buffer size too small");
	}

	// 从 resultBuffer 的前 8 个字节中读取矩阵的行数和列数
	int rows, cols;
	std::memcpy(&rows, resultBuffer, sizeof(int));
	std::memcpy(&cols, resultBuffer + sizeof(int), sizeof(int));

	// 打印行数和列数以调试
	//std::cout << "Rows: " << rows << std::endl;
	//std::cout << "Cols: " << cols << std::endl;

	// 计算数据部分的大小
	size_t data_size = buffer_size - 2 * sizeof(int);

	// 确保数据大小匹配
	if (data_size != rows * cols * sizeof(float)) {
		std::cerr << "Data size mismatch: expected " << rows * cols * sizeof(float) << ", got " << data_size << std::endl;
		throw std::runtime_error("Data size mismatch");
	}

	// 创建 Eigen 矩阵，自动分配大小，以行主序
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix(rows, cols);

	// 将数据部分复制到 Eigen 矩阵中
	std::memcpy(matrix.data(), resultBuffer + 2 * sizeof(int), data_size);

	return matrix;
}

cv::Mat decodeJpegFromByteStream(const std::vector<unsigned char>& byte_stream) {
	/*
		用opencv解码jpeg编码图像比特流
		params：
			byte_stream：图像比特流
		return：
			image：opencv图像对象
	*/
	// 检查字节流是否为空
	if (byte_stream.empty()) {
		throw std::runtime_error("Byte stream is empty.");
	}

	// 使用 OpenCV 的 imdecode 函数将字节流解码为图像
	cv::Mat image = cv::imdecode(byte_stream, cv::IMREAD_COLOR);


	// 检查解码是否成功
	if (image.empty()) {
		throw std::runtime_error("Failed to decode JPEG image from byte stream.");
	}

	return image;
}

void readFramePipe(HANDLE fPipe, std::atomic<bool>& isImageDataflowReady, ImageBuffer& image_buffers) {

	// 单独用于处理frame类型的pipe数据流


	const DWORD buffer_size = 10000000;
	std::vector<unsigned char> frameBuffer(buffer_size);
	DWORD frameBytesRead = 0;

	while (true) {


		BOOL isFrameRead = ReadFile(
			fPipe,          // 命名管道的句柄
			frameBuffer.data(),    // 缓冲区，用于接收数据
			buffer_size,    // 要读取的字节数
			&frameBytesRead,     // 实际读取的字节数
			NULL            // 不使用重叠I/O
		);

		//判断pipe是否正常工作
		if (!isFrameRead || frameBytesRead == 0) {
			std::cerr << "读取数据失败或管道已关闭！" << std::endl;
			break;
		}

		//写入最新的frame
		writeSharedVariable(decodeJpegFromByteStream(frameBuffer), image_buffers.buffer1, image_buffers.buffer2, image_buffers.using_buffer1, mtx);

		//展示
		//cv::imshow("Image", image_buffers.buffer1);
		//cv::waitKey(1);

		if (!isImageDataflowReady.exchange(true)) {  // 原子操作，确保只执行一次

			{
				// 获取锁，确保对条件的安全修改
				std::lock_guard<std::mutex> lock(mtx);

				//计数器+1
				g_cond_dataflow_counter = g_cond_dataflow_counter + 1;
			}

			//计数器计数到达要求时，则唤醒线程
			if (g_cond_dataflow_counter == g_cond_dataflow_require) {
				//尝试通知唤醒阻塞线程
				dataflow.notify_all();
			}

		}

		//输出
		//std::cout << g_results.format(HeavyFmt) << std::endl;

	}
}

void readResultPipe(HANDLE rPipe, std::atomic<bool>& isResultDataflowReady, MartixBuffer& yoloresults_buffers) {

	// 用于处理yolo识别结果的管道流
	const DWORD buffer_size = 10000000;
	char* resultBuffer = new char[buffer_size];
	DWORD resultBytesRead = 0;

	while (true) {

		BOOL isResultRead = ReadFile(
			rPipe,          // 命名管道的句柄
			resultBuffer,         // 缓冲区，用于接收数据
			buffer_size,    // 要读取的字节数
			&resultBytesRead,     // 实际读取的字节数
			NULL            // 不使用重叠I/O
		);

		//检测pipe是否正常工作
		if (!isResultRead || resultBytesRead == 0) {
			std::cerr << "读取数据失败或管道已关闭！" << std::endl;
			break;
		}


		if (!isResultDataflowReady.exchange(true)) {  // 原子操作，确保只执行一次

			{
				// 获取锁，确保对条件的安全修改
				std::lock_guard<std::mutex> lock(mtx);

				//计数器+1
				g_cond_dataflow_counter = g_cond_dataflow_counter + 1;
			}

			//计数器计数到达要求时，则唤醒线程
			if (g_cond_dataflow_counter == g_cond_dataflow_require) {
				//尝试通知唤醒阻塞线程
				dataflow.notify_all();
			}

		}

		//更新yolo识别的结果，双缓冲机制原理见readFramePipe()中，其相应位置有介绍
		writeSharedVariable(deserialize_to_eigen_matrix_with_shape(resultBuffer, resultBytesRead), yoloresults_buffers.buffer1, yoloresults_buffers.buffer2, yoloresults_buffers.using_buffer1, mtx);

		//更新这个大数据块的状态
		if (yoloresults_buffers.isEmpty) {
			yoloresults_buffers.isEmpty = false; //标记为非空
		}
		else if (!yoloresults_buffers.isFull) {
			yoloresults_buffers.isFull = true; //标记为满
		}

	}

}

bool connectToPipe(HANDLE& hPipe, const char* pipeName) {
	/*
		连接到pipe
		params：
			hPipe:句柄名称
			pipeName:命名管道名称
		return:
			BOOL
	*/
	int retries = 0;
	const int max_retries = 5;
	const int retry_delay = 1000; // milliseconds

	while (retries < max_retries) {
		hPipe = CreateFileA(
			pipeName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL
		);

		if (hPipe != INVALID_HANDLE_VALUE) {
			std::cout << "成功连接到管道：" << pipeName << std::endl;
			return true;
		}
		//std::cout << "正尝试连接到管道：" << pipeName << std::endl;
		//这里可以进行计次retries
		//retries++;
		Sleep(retry_delay);
	}

	std::cout << "连接到管道" << pipeName << "时错误" << std::endl;
	return false;
}