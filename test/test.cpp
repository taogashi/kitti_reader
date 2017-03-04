#include "kitti_reader/KittiReader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv)
{ 
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	KittiReader reader;
	reader.set_cloud_dir("/home/ys/Old/ProgramData/dataset/kitti-lidar/sequences/01/velodyne");
	reader.set_kitti_file_len(6);
	reader.set_start_index(10);

	bool res = true;
	do {
		KittiReader::PointCloud::Ptr cloud(new KittiReader::PointCloud());
		res = reader.read_cloud(cloud);
		if (res) {
			std::cout << cloud->size() << std::endl;
			pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud);
			viewer.showCloud(cloud);
		}
		else
			std::cout << "shit, it's empty." << std::endl;
		sleep(1);
	} while (res && !viewer.wasStopped());
	return 0;
}
