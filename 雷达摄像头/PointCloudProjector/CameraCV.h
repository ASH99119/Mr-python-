#pragma once

#ifndef CAMERACV_H
#define CAMERACV_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Windows.h>
#include "Tools.h"

//yolo图像帧
extern ImageBuffer g_image_buffers_L;
extern ImageBuffer g_image_buffers_R;


//yolo识别结果
extern MartixBuffer g_yoloresults_buffers_L;
extern MartixBuffer g_yoloresults_buffers_R; //这个只用于老yolo端，存yolo的result，也就是识别结果矩阵

//yolop识别结果
extern MartixBuffer g_yolopPred_buffers_L;
extern MartixBuffer g_yolopPred_buffers_R; //这个只用于yolop端，存yolop的识别结果矩阵
extern MartixBuffer g_yolopDrivableArea_buffers_L;
extern MartixBuffer g_yolopDrivableArea_buffers_R; //这个只用于yolop端，存yolop的可行驶区域矩阵
extern MartixBuffer g_yolopLaneline_buffers_L;
extern MartixBuffer g_yolopLaneline_buffers_R; //这个只用于yolop端，存yolop的车道线矩阵



Eigen::MatrixXf deserialize_to_eigen_matrix_with_shape(const char* resultBuffer, size_t buffer_size);

cv::Mat decodeJpegFromByteStream(const std::vector<unsigned char>& byte_stream);

void readFramePipe(HANDLE fPipe, std::atomic<bool>& isImageDataflowReady, ImageBuffer& image_buffers);

void readResultPipe(HANDLE rPipe, std::atomic<bool>& isResultDataflowReady, MartixBuffer& yoloresults_buffers);

bool connectToPipe(HANDLE& hPipe, const char* pipeName);

#endif // CAMERACV_H

