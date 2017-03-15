#include <opencv2/opencv.hpp>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <boost/array.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

class KittiReader {
	public:
//		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		using PointT = pcl::PointXYZI;
		using PointCloud = pcl::PointCloud<PointT>;

		KittiReader();
		~KittiReader();

		bool read_cloud(PointCloud::Ptr &cloud);
		bool read_img(cv::Mat &img);
		bool read_pose(Eigen::Matrix4d &pose);
		bool read_calib(Eigen::Matrix4d& Tr,
				Eigen::Matrix<double, 3, 4>& P0);

		void set_root_dir(const std::string& dir)
		{
			root_dir_ = dir;
		}

		void set_cloud_dir(const std::string &dir)
		{
			cloud_dir_ = dir;
		}

		void set_img_dir(const std::string &dir)
		{
			image_dir_ = dir;
		}

		void set_calib_dir(const std::string &dir)
		{
			calib_dir_ = dir;
		}

		// pose dir is actually a regular file
		void set_pose_dir(const std::string &dir);

		void set_kitti_file_len(int len)
		{
			if (len < 1) {
				return;
			}
			kitti_file_id_len_ = len;
		}

		void set_start_index(int start_idx);

	private:
		void reset_read_flags(void);

		std::string root_dir_;
		std::string cloud_dir_;
		std::string image_dir_;
		std::string calib_dir_;
		std::ifstream pose_file_;

		int kitti_file_id_len_;
		int frame_index_;
		bool finished_;

		bool is_cloud_read_;
		bool is_image_read_;
		bool is_calib_read_;
		bool is_pose_read_;
};

