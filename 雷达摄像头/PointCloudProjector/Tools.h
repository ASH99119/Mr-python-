#pragma once
#ifndef TOOLS_H
#define TOOLS_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include "Windows.h"
#include <utility>


template <typename T>
class Buffer {
public:

	//双buffer
	T buffer1;
	T buffer2;

	//buffer指示器
	std::atomic<bool> using_buffer1;
	std::atomic<bool> isFull;
	std::atomic<bool> isEmpty;

	//初始化using_buffer1为true
	Buffer() : using_buffer1(true), isFull(false), isEmpty(true) {}
};

using MartixBuffer = Buffer<Eigen::MatrixXf>;
using ImageBuffer = Buffer<cv::Mat>;

//锁
extern std::mutex mtx;

//线程唤醒条件变量
extern std::condition_variable dataflow;
extern int g_cond_dataflow_counter; //dataflow条件变量唤醒计数器
extern int g_cond_dataflow_require;  //唤醒需要的变量数(4条管道)
extern std::atomic<bool> g_isdataflowAlreadyNotified;  // 标志位，初始为 false(用原子变量)

extern std::condition_variable postprocess;

//线程正常运行的标志位
extern std::atomic<bool> g_isResultLDataflowReady;
extern std::atomic<bool> g_isResultRDataflowReady;

extern std::atomic<bool> g_isImageLDataflowReady;
extern std::atomic<bool> g_isImageRDataflowReady;

extern std::atomic<bool> g_isPredLflowReady;
extern std::atomic<bool> g_isPredRflowReady;       //目标识别锚框矩阵数据流标志符
extern std::atomic<bool> g_isDrivableAreaLReady;
extern std::atomic<bool> g_isDrivableAreaRReady;  //可行驶区域矩阵数据流标志符
extern std::atomic<bool> g_isLanelineLReady;
extern std::atomic<bool> g_isLanelineRReady;  //车道线矩阵数据流标志符


//extern bool g_isPointCloudDataflowReady;
extern bool g_isNeedWorking;

//高精度计时器
class Timer {
public:
	Timer() : start_time_(std::chrono::steady_clock::now()), elapsed_time_(0) {}

	void start() {
		start_time_ = std::chrono::steady_clock::now();
	}

	void stop() {
		auto end_time = std::chrono::steady_clock::now();
		elapsed_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
	}

	void reset() {
		start_time_ = std::chrono::steady_clock::now();
		elapsed_time_ = 0;
	}

	long long elapsed() {
		return elapsed_time_;
	}

	void printElapsedTime() {
		std::cout << "Elapsed time: " << elapsed_time_ << " milliseconds\n";
	}

private:
	std::chrono::steady_clock::time_point start_time_;
	long long elapsed_time_;
};

//template<typename T>
//void readSharedVariable(T& variable, const T& buffer1, const T& buffer2, std::atomic<bool>& using_buffer1, std::mutex& mtx);
//
//template<typename T>
//void writeSharedVariable(const T& variable, T& buffer1, T& buffer2, std::atomic<bool>& using_buffer1, std::mutex& mtx);

// 模板函数，用于读写变量
template<typename T>
void readSharedVariable(T& variable, const T& buffer1, const T& buffer2, std::atomic<bool>& using_buffer1, std::mutex& mtx) {

	/*
		用于处理多线程同时读取全局变量。
		params:
			variable:读取共享变量后写入的变量
			buffer1:共享变量buffer1
			buffer2:共享变量buffer2
			using_buffer1:指示当前正在使用的buffer(默认为buffer1)
			mtx:一把锁
		return：


	*/
	{
		std::lock_guard<std::mutex> lock(mtx);  // 获取锁来读取数据

		// 根据using_buffer1来赋值给variable
		if (using_buffer1) {

			variable = buffer1;  // 读取 buffer1

		}
		else {
			variable = buffer2;  // 读取 buffer2
		}
	}

}

template<typename T>
void writeSharedVariable(const T& variable, T& buffer1, T& buffer2, std::atomic<bool>& using_buffer1, std::mutex& mtx) {

	/*
		用于处理多线程同时写入全局变量。
		params:
			variable:需要写入到共享变量的变量
			buffer1:共享变量buffer1
			buffer2:共享变量buffer2
			using_buffer1:指示当前正在使用的buffer(默认为buffer1)
			mtx:一把锁
		return：


	*/
	{
		std::lock_guard<std::mutex> lock(mtx); // 获取锁来写入数据，这个锁仅在本作用域有效

		// 检测是否在使用 buffer1
		if (using_buffer1) {
			// 写入到 buffer2
			buffer2 = variable;
			// 切换缓冲区
			using_buffer1 = false;

		}
		else {
			// 写入到 buffer1
			buffer1 = variable;
			// 切换缓冲区
			using_buffer1 = true;
		}
	}
}

template<typename T>
void writeSharedVariable(const T& variable, T& buffer1, T& buffer2, std::atomic<bool>& using_buffer1, std::atomic<bool>& isEmpty, std::atomic<bool>& isFull, std::mutex& mtx) {

	/*
		用于处理多线程同时写入全局变量。(重载)
		params:
			variable:需要写入到共享变量的变量
			buffer1:共享变量buffer1
			buffer2:共享变量buffer2
			isFull:满指示符
			isEmpty:空指示符
			using_buffer1:指示当前正在使用的buffer(默认为buffer1)
			mtx:一把锁
		return：


	*/
	{
		std::lock_guard<std::mutex> lock(mtx); // 获取锁来写入数据，这个锁仅在本作用域有效

		// 检测是否在使用 buffer1
		if (using_buffer1) {
			// 写入到 buffer2
			buffer2 = variable;
			// 切换缓冲区
			using_buffer1 = false;

		}
		else {
			// 写入到 buffer1
			buffer1 = variable;
			// 切换缓冲区
			using_buffer1 = true;
		}
		//修改队列标志

		if (!isFull) {
			//如果队列没有满
			if (isEmpty) {
				//如果是空的
				isEmpty = false;
			}
			else {
				//既不空也不满
				isFull = true;
			}
		}


	}

}

void writeFrameInfoPipe(HANDLE fiPipe, const Eigen::MatrixXf buffer);

std::vector<char> serializeEigenMatrix(const Eigen::MatrixXf& matrix);

Eigen::MatrixXf removeNaNRows(const Eigen::MatrixXf& input);

void appendRow(Eigen::MatrixXf& target, const Eigen::MatrixXf& source, int rowIndex);

Eigen::MatrixXf removeColumn(const Eigen::MatrixXf& mat, int col_idx);

cv::Mat skeletonize(const cv::Mat& binary_input);

cv::Mat removeIsolatedPoints(const cv::Mat& input);

#endif // Tools