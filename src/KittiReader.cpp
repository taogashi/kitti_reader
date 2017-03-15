#include "kitti_reader/KittiReader.h"
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

KittiReader::KittiReader():
	kitti_file_id_len_(6),
	finished_(false),
	frame_index_(0),
	is_cloud_read_(false),
	is_image_read_(false),
	is_calib_read_(false),
	is_pose_read_(false)
{
}

KittiReader::~KittiReader()
{
	pose_file_.close();
}

bool KittiReader::read_cloud(PointCloud::Ptr &cloud)
{
	if (!cloud) {
		return false;
	}

	if (cloud_dir_.empty()) {
		return false;
	}

	if (finished_) {
		return false;
	}

	if (is_cloud_read_) {
		frame_index_++;
		reset_read_flags();
	}
	is_cloud_read_ = true;

    std::stringstream ss;
    ss<<'/'<<std::setfill('0')<<std::setw(kitti_file_id_len_)<<frame_index_<<".bin";
    boost::filesystem::path file_full_path = boost::filesystem::path(root_dir_) /
		boost::filesystem::path(cloud_dir_) /
		boost::filesystem::path(ss.str());
    if (!boost::filesystem::is_regular(file_full_path)) {
        finished_ = true;
        return false;
    }

    int num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));
    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (file_full_path.string().c_str(),"rb");
	if (stream == NULL) {
		std::cerr << "failed to open file " << file_full_path.string() << std::endl;
		return false;
	}
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++) {
		PointT pt(*pr);
		pt.x = *px;
		pt.y = *py;
		pt.z = *pz;
        cloud->points.push_back(pt);
        px+=4;
        py+=4;
        pz+=4;
        pr+=4;
    }
    fclose(stream);
	free(data);
	cloud->width = cloud->size();
	cloud->height = 1;
	cloud->header.stamp = (std::chrono::system_clock::now().time_since_epoch()).count()
		* std::chrono::system_clock::period::num * 1000 / std::chrono::system_clock::period::den;
	cloud->header.seq = frame_index_;
	cloud->header.frame_id = "lidar";
	return true;
}

bool KittiReader::read_img(cv::Mat &img)
{
	if (image_dir_.empty()) {
		return false;
	}

	if (finished_) {
		return false;
	}

	if (is_image_read_) {
		frame_index_++;
		reset_read_flags();
	}
	is_image_read_ = true;

    std::stringstream ss;
    ss<<'/'<<std::setfill('0')<<std::setw(kitti_file_id_len_)<<frame_index_<<".png";
    boost::filesystem::path file_full_path = boost::filesystem::path(root_dir_) /
		boost::filesystem::path(image_dir_) /
		boost::filesystem::path(ss.str());
    if (!boost::filesystem::is_regular(file_full_path)) {
        finished_ = true;
        return false;
    }

	img = cv::imread(file_full_path.string().c_str());
	if (img.empty()) {
		return false;
	}
	return true;
}

bool KittiReader::read_pose(Eigen::Matrix4d &pose)
{
	if (!pose_file_.is_open()) {
		std::cerr << "invalid pose file" << std::endl;
		return false;
	}

	if (finished_) {
		std::cerr << "finished" << std::endl;
		return false;
	}
	
	std::string line;
	if (!std::getline(pose_file_, line)) {
		std::cerr << "end of file." << std::endl;
		return false;
	}

	if (is_pose_read_) {
		frame_index_++;
		reset_read_flags();
	}
	is_pose_read_ = true;

	std::vector<double> vals;
	std::stringstream ss(line);
	while (ss) {
		std::string s;
		if (!std::getline(ss, s, ' '))
			break;
		vals.push_back(atof(s.c_str()));
	}
	pose.block<3, 4>(0, 0) = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> >(&(vals[0]));

	pose(3, 0) = pose(3, 1) = pose(3, 2) = 0.0;
	pose(3, 3) = 1.0;
	return true;
}

bool KittiReader::read_calib(Eigen::Matrix4d& Tr,
		Eigen::Matrix<double, 3, 4>& P0)
{
	if (calib_dir_.empty())
		return false;
	if (finished_)
		return false;

	boost::filesystem::path file_path = boost::filesystem::path(root_dir_) /
		boost::filesystem::path(calib_dir_);
	boost::filesystem::path full_file_path;
	if (boost::filesystem::is_directory(file_path)) {
		if (is_calib_read_) {
			frame_index_++;
			reset_read_flags();
		}
		is_calib_read_ = true;
		std::stringstream ss;
		ss << std::setfill('0') << std::setw(kitti_file_id_len_) << frame_index_ << ".txt";
		full_file_path = file_path / boost::filesystem::path(ss.str());
		if (!boost::filesystem::is_regular_file(full_file_path)) {
			return false;
		}
	} else if (boost::filesystem::is_regular_file(file_path)) {
		full_file_path = file_path;
	}
	std::ifstream ifs(full_file_path.string());
	if (!ifs.is_open()) {
		return false;
	}
	std::string line;
	while (std::getline(ifs, line)) {
		size_t pos = line.find("P0");
		if (pos != std::string::npos) {
			std::stringstream ss(line.substr(pos + 4));
			double vals[12] = {0.0};
			for (int i = 0; i < 12; i++) {
				ss >> vals[i];
			}
			std::cout << std::endl;
			P0 = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> >(vals);
			continue;
		}
		pos = line.find("Tr");
		if (pos != std::string::npos) {
			std::stringstream ss(line.substr(pos + 4));
			double vals[16] = {0.0};
			for (int i = 0; i < 12; i++)
				ss >> vals[i];
			vals[12] = vals[13] = vals[14] = 0.0;
			vals[15] = 1.0;
			Tr = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(vals);
			continue;
		}
	}
}
#if 0
void KittiReader::read_calib(uint64_t stamp)
{
	std::stringstream ss;
	ss << calib_dir_ << '/' << std::setfill('0')
	   << std::setw(kitti_file_id_len_) << frame_index_ << ".txt";
	std::ifstream ifs(ss.str().c_str());
	if (!ifs.is_open()) {
		return;
	}
	char buf[256];
	while(ifs.getline(buf, 256)) {
		size_t pos = 0;
		if ((pos = std::string(buf).find("P2")) != std::string::npos) {
			std::stringstream line(std::string(buf).substr(pos+4));
			for (int i = 0; i < 12; i++) {
				line >> P_[i];
			}
		} else if ((pos = std::string(buf).find("R0_rect")) != std::string::npos) {
			std::stringstream line(std::string(buf).substr(pos+9));
			for (int i = 0; i < 9; i++) {
				line >> R0_rect_[i];
			}
		} else if ((pos = std::string(buf).find("Tr_velo_to_cam")) != std::string::npos) {
			std::stringstream line(std::string(buf).substr(pos+16));
			for (int i = 0; i < 12; i++) {
				line >> T_velo_to_cam_[i];
			}
		}
	}
}
#endif

// pose dir is actually a regular file
void KittiReader::set_pose_dir(const std::string &dir)
{
	boost::filesystem::path full_path = boost::filesystem::path(root_dir_) /
		boost::filesystem::path(dir);
	if (!boost::filesystem::is_regular(full_path)) {
		std::cerr << "invalid pose file: " << full_path.string()<< std::endl;
		return;
	}
	pose_file_.open(full_path.string().c_str());
}

void KittiReader::reset_read_flags(void)
{
	is_cloud_read_ = false;
	is_image_read_ = false;
	is_calib_read_ = false;
	is_pose_read_ = false;
}

void KittiReader::set_start_index(int start_idx)
{
	frame_index_ = std::max(start_idx, 0);
	pose_file_.seekg(std::ios::beg);
    for(int i=0; i < frame_index_; ++i){
        pose_file_.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
}
