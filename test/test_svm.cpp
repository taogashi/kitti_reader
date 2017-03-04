#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/ml/svm_wrapper.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <gperftools/profiler.h>
#include "kitti_reader/KittiReader.h"

#include <pcl/filters/random_sample.h>
#include <pcl/features/normal_3d.h>

using namespace Eigen;

#define DEG_TO_RAD (1/180.0*3.1415926)
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudMono;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudColor;

inline bool simple_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                       pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
                       const std::string &id, int size)
{
    if (!viewer)
        return false;

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);

    return true;
}

int
main (int argc, char** argv)
{
    ProfilerStart("/tmp/pcl.prof");
	KittiReader reader;
	reader.set_cloud_dir("/home/ys/ProgramData/dataset/kitti-obj/training/velodyne");
	reader.set_img_dir("/home/ys/ProgramData/dataset/kitti-obj/training/image_2");
	reader.set_kitti_file_len(6);
	reader.set_start_index(10);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(2.0);
	viewer->initCameraParameters();

	KittiReader::PointCloud::Ptr cloud(new KittiReader::PointCloud());
	KittiReader::PointCloud::Ptr filtered_cloud(new KittiReader::PointCloud());
	PointCloudColor::Ptr result(new PointCloudColor());
	if (!reader.read_cloud(cloud)) {
		std::cout << "failed to read kitti." << std::endl;	
		return 1;
	}

	pcl::RandomSample<KittiReader::PointT> rs;
	rs.setSample(static_cast<unsigned int>(0.3 * cloud->points.size()));
	rs.setInputCloud(cloud);
	rs.filter(*filtered_cloud);

	pcl::NormalEstimation<KittiReader::PointT, pcl::Normal> ne;
	ne.setInputCloud(filtered_cloud);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(30);
//	ne.setRadiusSearch (1.0);
	ne.compute (*cloud_normals);

	pcl::copyPointCloud(*filtered_cloud, *result);
	for (auto& pt : result->points) {
		pt.rgba = 0xFFFF0000;
	}
	simple_vis(viewer, result, "result", 1);
	viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (result, cloud_normals, 5, 0.5, "normals");

#if 0
    pcl::SVMTrain svm;
    pcl::SVMParam svm_param;
    svm_param.svm_type = C_SVC;
    svm_param.kernel_type = LINEAR;
    svm_param.C = 1.0;

    std::vector<pcl::SVMData> training_data;
    for (auto& pt : sphere1->points) {
        pcl::SVMData svmdata;
        svmdata.label = 1.0;
        pcl::SVMDataPoint dp;
        dp.idx = 0;
        dp.value = pt.x;
        svmdata.SV.push_back(dp);
        dp.idx = 1;
        dp.value = pt.y;
        svmdata.SV.push_back(dp);
        dp.idx = 2;
        dp.value = pt.z;
        svmdata.SV.push_back(dp);
        training_data.push_back(svmdata);
    }
    for (auto& pt : sphere2->points) {
        pcl::SVMData svmdata;
        svmdata.label = 2.0;
        pcl::SVMDataPoint dp;
        dp.idx = 0;
        dp.value = pt.x;
        svmdata.SV.push_back(dp);
        dp.idx = 1;
        dp.value = pt.y;
        svmdata.SV.push_back(dp);
        dp.idx = 2;
        dp.value = pt.z;
        svmdata.SV.push_back(dp);
        training_data.push_back(svmdata);
    }
    for (auto& pt : sphere2->points) {
        pcl::SVMData svmdata;
        svmdata.label = std::numeric_limits<double>::signaling_NaN();
        pcl::SVMDataPoint dp;
        dp.idx = 0;
        dp.value = pt.x;
        svmdata.SV.push_back(dp);
        dp.idx = 1;
        dp.value = pt.y;
        svmdata.SV.push_back(dp);
        dp.idx = 2;
        dp.value = pt.z;
        svmdata.SV.push_back(dp);
        training_data.push_back(svmdata);
    }

    svm.setInputTrainingSet(training_data);
    svm.trainClassifier();

    pcl::SVMClassify classifier;
    classifier.setClassifierModel(svm.getClassifierModel());
    classifier.setInputTrainingSet(training_data);
    classifier.classification();
    std::vector< std::vector<double> > out;
    classifier.getClassificationResult(out);
    std::cout << "output: " << out.size() << std::endl;
    std::cout << "output[0]: " << out[0].size() << std::endl;
    std::cout << "output[1]: " << out[1].size() << std::endl;
#endif
	while(!viewer->wasStopped()) {
		viewer->spinOnce(50);
	}

    ProfilerStop();
    return (0);
}
