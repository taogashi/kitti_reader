#include "kitti_reader/KittiReader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv)
{ 
	pcl::visualization::PCLVisualizer pcl_viewer;
	bool viewer_inited_ = false;

	cv::namedWindow("img");

	KittiReader reader;
	reader.set_cloud_dir("/home/ys/Old/ProgramData/dataset/kitti-odo/lidar/sequences/00/velodyne");
	reader.set_calib_dir("/home/ys/Old/ProgramData/dataset/kitti-odo/cali/sequences/00/calib.txt");
	reader.set_pose_dir("/home/ys/Old/ProgramData/dataset/kitti-odo/gt/poses/00.txt");
	reader.set_kitti_file_len(6);
	reader.set_start_index(0);

	boost::thread* viewer_thread = new boost::thread([&]() {
		pcl_viewer.setBackgroundColor(0, 0, 0);
		pcl_viewer.addCoordinateSystem(2.0);
		pcl_viewer.initCameraParameters();
		while (!boost::this_thread::interruption_requested() && !pcl_viewer.wasStopped()) {
		pcl_viewer.spinOnce(10);
		} });

	bool res = true;
	 while (res && !pcl_viewer.wasStopped()) {
		KittiReader::PointCloud::Ptr cloud(new KittiReader::PointCloud());
		cv::Mat img;
		res = reader.read_img(img);
		if (res) {
			cv::imshow("img", img);
		} else {
			std::cout << "failed to read img" << std::endl;
		}

		Eigen::Matrix4d pose;
		res = reader.read_pose(pose);
		if (res) {
			std::cout << pose << std::endl;
		} else {
			std::cout << "failed to read pose." << std::endl;
		}

		Eigen::Matrix4d Tr;
		Eigen::Matrix<double, 3, 4> P0;
		res = reader.read_calib(Tr, P0);
		if (res) {
			std::cout << Tr << std::endl;
		}
#if 1
		res = reader.read_cloud(cloud);
		if (res) {
			std::cout << cloud->size() << std::endl;
			pcl::visualization::PointCloudColorHandlerGenericField<KittiReader::PointT> intens(cloud, "intensity"); 
			if (!viewer_inited_) {
				pcl_viewer.addPointCloud<pcl::PointXYZI>(cloud, intens, "cloud");
				pcl_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
				viewer_inited_ = true;
			} else {
				pcl_viewer.updatePointCloud<pcl::PointXYZI>(cloud, intens, "cloud");
			}
		} else {
			std::cout << "shit, something wrong." << std::endl;
		}
#endif
		cv::waitKey(0);
	}
	std::cout << "finish cleanly." << std::endl;
	return 0;
}
