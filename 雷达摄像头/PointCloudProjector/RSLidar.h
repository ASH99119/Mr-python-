#pragma once
#ifndef RSLIDAR_H
#define RSLIDAR_H

//rs_driver的头文件
#include <rs_driver/api/lidar_driver.hpp>
#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

#include <Eigen/Dense>
#include "Tools.h"

//用PointT作为模板类PointCloudT的模板参数
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

// 定义两个同步消息队列 用于存储指向PointCloudMsg的指针
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

//雷达点云point_cloud_c
extern MartixBuffer g_point_cloud_c_buffers_left;
extern MartixBuffer g_point_cloud_c_buffers_right;

//雷达点云point_cloud_l
extern MartixBuffer g_point_cloud_l_buffers;

//雷达点云point_cloud_p
extern MartixBuffer g_point_cloud_p_buffers_left;
extern MartixBuffer g_point_cloud_p_buffers_right;

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);

void exceptionCallback(const robosense::lidar::Error& code);

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);

Eigen::MatrixXf homogenizePoints(const Eigen::MatrixXf& points);

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf, Eigen::MatrixXf> lidarToPixel(Eigen::MatrixXf rawPointCloud, Eigen::MatrixXf lidar_to_cam, Eigen::MatrixXf cam_to_pixel);

Eigen::MatrixXf convertPointCloudToEigenMatrix(const std::shared_ptr<PointCloudMsg>& msg);

#endif // RSLIDAR_H