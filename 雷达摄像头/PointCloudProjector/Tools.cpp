#include "Tools.h"
#include <iostream>
//锁
std::mutex mtx;

//线程唤醒条件变量
std::condition_variable dataflow;
int g_cond_dataflow_counter = 0; //dataflow条件变量唤醒计数器
int g_cond_dataflow_require = 8;  //唤醒需要的变量数(4条管道)
std::atomic<bool> g_isdataflowAlreadyNotified(false);  // 标志位，初始为 false(用原子变量)

std::condition_variable postprocess;

//线程正常运行的标志位
std::atomic<bool> g_isResultLDataflowReady = false;
std::atomic<bool> g_isResultRDataflowReady = false;    //目标识别矩阵的管道(老yolo端使用,只有图像识别功能的yolo)

std::atomic<bool> g_isImageLDataflowReady = false;
std::atomic<bool> g_isImageRDataflowReady = false;		 //图像数据流标志符

std::atomic<bool> g_isPredLflowReady = false;    
std::atomic<bool> g_isPredRflowReady = false;       //目标识别锚框矩阵数据流标志符
std::atomic<bool> g_isDrivableAreaLReady = false;
std::atomic<bool> g_isDrivableAreaRReady = false;  //可行驶区域矩阵数据流标志符
std::atomic<bool> g_isLanelineLReady = false;
std::atomic<bool> g_isLanelineRReady = false;  //车道线矩阵数据流标志符


bool g_isPointCloudDataflowReady = false;
bool g_isNeedWorking = false;

void writeFrameInfoPipe(HANDLE fiPipe, const Eigen::MatrixXf buffer) {

	int rows = buffer.rows();
	int cols = buffer.cols();
	std::vector<float> matdata(buffer.data(), buffer.data() + buffer.size());

	DWORD byteswritten;


	//写入行列信息

	WriteFile(fiPipe, serializeEigenMatrix(buffer).data(), 65535, &byteswritten, NULL);


}

std::vector<char> serializeEigenMatrix(const Eigen::MatrixXf& matrix) {
	// 获取矩阵的行数和列数
	int rows = matrix.rows();
	int cols = matrix.cols();

	// 将行列信息打包成字节流（4 字节行数 + 4 字节列数）
	std::vector<char> serializedData(sizeof(int) * 2); // 2 * sizeof(int) = 8 字节
	std::memcpy(serializedData.data(), &rows, sizeof(int)); // 行数
	std::memcpy(serializedData.data() + sizeof(int), &cols, sizeof(int)); // 列数

	// 将矩阵数据转换为字节流并附加到行列信息后面
	const float* matrixData = matrix.data();
	serializedData.resize(serializedData.size() + matrix.size() * sizeof(float));
	std::memcpy(serializedData.data() + sizeof(int) * 2, matrixData, matrix.size() * sizeof(float));

	return serializedData;
}

Eigen::MatrixXf removeNaNRows(const Eigen::MatrixXf& input) {
	std::vector<Eigen::Vector3f> valid_rows;

	for (int i = 0; i < input.rows(); ++i) {
		Eigen::Vector3f row = input.row(i);
		if (!std::isnan(row[0]) && !std::isnan(row[1]) && !std::isnan(row[2])) {
			valid_rows.push_back(row);
		}
	}

	// 构造新的矩阵
	Eigen::MatrixXf result(valid_rows.size(), 3);
	for (size_t i = 0; i < valid_rows.size(); ++i) {
		result.row(i) = valid_rows[i];
	}

	return result;
}

void appendRow(Eigen::MatrixXf& target, const Eigen::MatrixXf& source, int rowIndex) {
	if (rowIndex < 0 || rowIndex >= source.rows()) {
		throw std::out_of_range("rowIndex out of range");
	}

	if (target.size() == 0) {
		// target 是空矩阵，直接初始化为一行
		target = source.row(rowIndex);
		return;
	}

	if (source.cols() != target.cols()) {
		throw std::invalid_argument("Column count mismatch");
	}

	Eigen::MatrixXf result(target.rows() + 1, target.cols());
	result << target,
		source.row(rowIndex);
	target = result;  // 覆盖原矩阵
}

Eigen::MatrixXf removeColumn(const Eigen::MatrixXf& mat, int col_idx) {
	int rows = mat.rows();
	int cols = mat.cols();

	if (col_idx < 0 || col_idx >= cols) {
		std::cerr << "Invalid column index!" << std::endl;

		Eigen::MatrixXf emptyMat;

		return emptyMat;
	}

	// 创建一个新的 m x (n-1) 矩阵
	Eigen::MatrixXf newMat(rows, cols - 1);

	// 复制左侧部分 (0 到 col_idx-1)
	if (col_idx > 0)
		newMat.leftCols(col_idx) = mat.leftCols(col_idx);

	// 复制右侧部分 (col_idx+1 到 末尾)
	if (col_idx < cols - 1)
		newMat.rightCols(cols - col_idx - 1) = mat.rightCols(cols - col_idx - 1);

	// 交换数据，替换原矩阵
	return newMat;
}

cv::Mat skeletonize(const cv::Mat& binary_input) {
	/*
		对二值图像进行骨架化处理
		params:
			binary_input:二值图像输入
		return:
			skel:骨架化后的图像
	*/

	cv::Mat binary;
	binary_input.convertTo(binary, CV_8UC1);  // 确保类型正确

	double maxVal;
	cv::minMaxLoc(binary, nullptr, &maxVal);
	if (maxVal <= 1.0) {
		binary *= 255;  // 将0/1变成0/255
	}

	cv::Mat skel(binary.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp, eroded;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	cv::Mat img = binary.clone();

	while (true) {
		cv::erode(img, eroded, element);
		cv::dilate(eroded, temp, element);
		cv::subtract(img, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(img);
		if (cv::countNonZero(img) == 0)
			break;
	}

	return skel;
}

cv::Mat removeIsolatedPoints(const cv::Mat& input) {
	/*
		对二值图像进行开运算，去除孤立点
		params:
			input:二值图像输入
		return:
			output:开运算后的图像
	*/
	cv::Mat output;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(input, output, cv::MORPH_OPEN, kernel);
	return output;
}


