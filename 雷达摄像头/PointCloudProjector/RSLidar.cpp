# include "RSLidar.h"

robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

//雷达点云point_cloud_c
MartixBuffer g_point_cloud_c_buffers_left;
MartixBuffer g_point_cloud_c_buffers_right;

//雷达点云point_cloud_l
MartixBuffer g_point_cloud_l_buffers;

//雷达点云point_cloud_p
MartixBuffer g_point_cloud_p_buffers_left;
MartixBuffer g_point_cloud_p_buffers_right;

//robosense雷达驱动案例给的方法，具体在pcd那个线程里有用到
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg){

	// Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
	//       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
	stuffed_cloud_queue.push(msg);
}

void exceptionCallback(const robosense::lidar::Error& code) {


	// @brief exception callback function. The caller should register it to the lidar driver.
	//        Via this function, the driver inform the caller that something happens.
	// @param code The error code to represent the error/warning/information
	// 
	// Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
	//       so please DO NOT do time-consuming task here.
	RS_WARNING << code.toString() << RS_REND;
}

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void) {

	// @brief point cloud callback function. The caller should register it to the lidar driver.
	//        Via this function, the driver gets an free/unused point cloud message from the caller.
	// @param msg  The free/unused point cloud message.
	// Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
	//       so please DO NOT do time-consuming task here.
	std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
	if (msg.get() != NULL)
	{
		return msg;
	}

	return std::make_shared<PointCloudMsg>();
}

Eigen::MatrixXf homogenizePoints(const Eigen::MatrixXf& points) {
	/*
		用于将[x,y,z]格式的点齐次化为:[x/z,y/z,1]
		params:
			points:MatrixXf类型的eigen矩阵,其格式为：[[x1,y1,z1],[x2,y2,z2]....]N*3
		returns:
			result:MatrixXf类型的eigen矩阵,其格式为：[[x1/z1,y1/z1,1],[x2/z2,y2/z2,1]....]N*3
	*/

	// 确保输入矩阵的列数为 3
	if (points.cols() != 3) {
		throw std::invalid_argument("输入矩阵必须有 3 列。");
	}

	// 创建一个与输入矩阵大小相同的输出矩阵
	Eigen::MatrixXf result(points.rows(), 3);

	// 遍历输入矩阵的每一行
	for (int i = 0; i < points.rows(); ++i) {
		float x = points(i, 0);
		float y = points(i, 1);
		float z = points(i, 2);

		// 避免除以零的情况
		if (z != 0.0f) {
			result(i, 0) = x / z;
			result(i, 1) = y / z;
			result(i, 2) = z / z; // 这将等于 1.0
		}
		else {
			// 如果 z 为 0，则设置为 NaN（也可以其他处理方式，其实这应该不会出现0，之前会过滤一次）
			result(i, 0) = std::numeric_limits<float>::quiet_NaN();
			result(i, 1) = std::numeric_limits<float>::quiet_NaN();
			result(i, 2) = std::numeric_limits<float>::quiet_NaN();
		}
	}

	return result;
}

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf, Eigen::MatrixXf> lidarToPixel(Eigen::MatrixXf rawPointCloud, Eigen::MatrixXf lidar_to_cam, Eigen::MatrixXf cam_to_pixel) {
	/*
	 将雷达点云根据相应的标定参数投射到图像上
		params：
			point_cloud:雷达点云坐标集。格式为[(x1,y1,z1),(x2,y2,z2)....]
			cam_to_pixel:相机坐标系到像素坐标系的变换矩阵，通常是相机内参K矩阵
			lidar_to_cam:雷达坐标系到相机坐标系的变换矩阵，通常由联合标定得到
		returns(暂时设置为返回point_cloud_p):
			point_cloud_l:有效投影到图片上的点云(lidar坐标系下)。格式为[(x1,y1,z1),(x2,y2,z2)....]
			point_cloud_c:有效投影到图片上的点云(camera坐标系下)。格式为[(x1,y1,z1),(x2,y2,z2)....]
			point_cloud_p:有效投影到图片上的点云(pixel坐标系下)。格式为[(u1,v1)...]


		std::tuple<Eigen::MatrixXf, Eigen::MatrixXf, Eigen::MatrixXf>
	*/

	//取rawPointCloud行数
	int rows = rawPointCloud.rows();
	//初始化一个全为1长度为rawPointCloud行数的向量
	Eigen::VectorXf newColumn(rows);
	newColumn.setOnes();

	//取前三列作为有效投影到图片上的点云(lidar坐标系下)
	Eigen::MatrixXf point_cloud_l = rawPointCloud.leftCols(3);

	//将point_cloud_l齐次化(homogenize)，即将[(x1,y1,z1),(x2,y2,z2)...]齐次化为[(x1,y1,z1,1),(x2,y2,z2,1)...]
	//注意这里与后面的齐次化略有不同，具体原理可查询关键字“相机坐标系投影”
	Eigen::MatrixXf point_cloud_l_homogenized = point_cloud_l;
	//将矩阵point_cloud_l_homogenized增加一列内存分配
	point_cloud_l_homogenized.conservativeResize(Eigen::NoChange, point_cloud_l_homogenized.cols() + 1);
	//将矩阵point_cloud_l_的第四列填充为向量newColumn
	point_cloud_l_homogenized.col(point_cloud_l_homogenized.cols() - 1) = newColumn;

	//std::cout << point_cloud_l_homogenized << std::endl;



	//将矩阵point_cloud_l_homogenized转置以便后续计算，此时矩阵point_cloud_l_homogenized格式为4xN
	Eigen::MatrixXf point_cloud_l_homogenized_t = point_cloud_l_homogenized.transpose();
	//计算点云在相机坐标系下的位置,其格式为[(x1,y1,z1),(x2,y2,z2)....] Nx3, point_cloud_c即为point_cloud在lidar坐标系
	//注意：这个带t的矩阵是需要转置一次才是最终的Nx3或者Nx4矩阵，此时point_cloud_c_t的格式为3xN
	Eigen::MatrixXf point_cloud_c_t = lidar_to_cam * point_cloud_l_homogenized_t;
	//注意：这里直接用point_cloud_c = point_cloud_c.transpose(),会导致矩阵数据异常。他不像单独的数字变量一样,可以直接覆写。
	Eigen::MatrixXf point_cloud_c = point_cloud_c_t.transpose();

	//std::cout << point_cloud_c << std::endl;

	//计算点云在图片上的位置,得到的结果仍需进一步处理
	//此处矩阵point_cloud_p_t的格式为3xN,point_cloud_p_unhomogenized的格式为Nx3
	Eigen::MatrixXf point_cloud_p_unhomogenized_t = cam_to_pixel * point_cloud_c_t;
	Eigen::MatrixXf point_cloud_p_unhomogenized = point_cloud_p_unhomogenized_t.transpose();

	//std::cout << point_cloud_p << std::endl;

	//齐次化矩阵，此时point_cloud_p_homogenized和point_cloud_p_unhomogenized的格式均为Nx3
	Eigen::MatrixXf point_cloud_p_homogenized = homogenizePoints(point_cloud_p_unhomogenized);
	//取point_cloud_p_homogenized每行前两个值作为像素坐标系下的u，v坐标，此时point_cloud_p的格式为Nx2
	Eigen::MatrixXf point_cloud_p = point_cloud_p_homogenized.leftCols(2);


	//std::cout << point_cloud_p << std::endl;

	//可以用std::get来实现分别读取
	/*
	* 示例：
		auto result = lidarToPixel(rawPointCloud, lidar_to_cam, cam_to_pixel);

		// 使用 std::get 提取 tuple 中的元素
		Eigen::MatrixXf point_cloud_p = std::get<0>(result);
		Eigen::MatrixXf point_cloud_l = std::get<1>(result);
	*/
	return std::make_tuple(point_cloud_l, point_cloud_c, point_cloud_p);
}

Eigen::MatrixXf convertPointCloudToEigenMatrix(const std::shared_ptr<PointCloudMsg>& msg) {
	/*
		将PointCloudMsg的msg点云转化为Eigen矩阵，仅对robosense点云有效
		param：
			msg:点云
		return：
			matrix：返回的eigen矩阵
	*/

	//排除点云为空的情况
	if (!msg || msg->points.empty()) {
		throw std::runtime_error("点云消息为空或点云数据为空。");
	}

	//取点云点的数量
	size_t numPoints = msg->points.size();
	//按点云点的数量初始化矩阵
	Eigen::MatrixXf matrix(numPoints, 4); // 4 列分别表示 x, y, z, intensity

	//将点云信息写入matrix
	for (size_t i = 0; i < numPoints; ++i) {

		const PointT& point = msg->points[i];




		matrix(i, 0) = point.x;
		matrix(i, 1) = point.y;
		matrix(i, 2) = point.z;
		matrix(i, 3) = static_cast<float>(point.intensity); //强度
	}

	return matrix;
}

