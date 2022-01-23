#include <fstream>
#include <string>
#include "Timer.h"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <FilterFusion/Visualizer.h>
#include "System.h"

using namespace std;

bool LoadSensorData(const std::string& encoder_file_path, std::unordered_map<std::string, std::string>* time_data_map) {
    std::ifstream encoder_file(encoder_file_path);
    if (!encoder_file.is_open()) {
        LOG(ERROR) << "[LoadSensorData]: Failed to open encoder file.";
        return false;
    }

    std::string line_str, time_str;
    while (std::getline(encoder_file, line_str)) {
        std::stringstream ss(line_str);
        if (!std::getline(ss, time_str, ',')) {
            LOG(ERROR) << "[LoadSensorData]: Find a bad line in the encoder file.: " << line_str;
            return false;
        }
        time_data_map->emplace(time_str, line_str);
    }

    return true;
}

bool LoadGT(const std::string& encoder_file_path, std::vector<std::string>& gt_time) {
    std::ifstream encoder_file(encoder_file_path);
    if (!encoder_file.is_open()) {
        LOG(ERROR) << "[LoadSensorData]: Failed to open encoder file.";
        return false;
    }

    std::string line_str, time_str;
    gt_time.clear();
    gt_time.reserve(140000);
    while (std::getline(encoder_file, line_str)) {
        std::stringstream ss(line_str);
        if (!std::getline(ss, time_str, ',')) {
            LOG(ERROR) << "[LoadSensorData]: Find a bad line in the encoder file.: " << line_str;
            return false;
        }
        gt_time.push_back(time_str);
    }

    return true;
}

std::string searchInsert(std::vector<std::string>& nums, const std::string& target) {
    int n = nums.size();
    int left = 0;
    int right = n; // 定义target在左闭右开的区间里，[left, right)  target
    while (left < right) { // 因为left == right的时候，在[left, right)是无效的空间
        int middle = left + ((right - left) >> 1);
        if (nums[middle] > target) {
            right = middle; // target 在左区间，在[left, middle)中
        } else if (nums[middle] < target) {
            left = middle + 1; // target 在右区间，在 [middle+1, right)中
        } else { // nums[middle] == target
            return nums[middle]; // 数组中找到目标值的情况，直接返回下标
        }
    }
    return nums[right];
}


// 1. Config file.
// 2. Dataset folder.
int main(int argc, char** argv) {


    std::cerr<<"1"<<std::endl;


    ros::init(argc, argv, "msckf_viw");
    ros::NodeHandle nh("~");

    if (argc != 2) {
        LOG(ERROR) << "[main]: Please input param_file.";
        return EXIT_FAILURE;
    }


    std::cerr<<"2"<<std::endl;

    FLAGS_minloglevel = 3;
    FLAGS_logtostderr = true;
    const std::string param_file = argv[1];

    shared_ptr<System> mpSystem = std::make_shared<System>(nh, param_file);
    std::thread sync_thread(&System::sync_process, mpSystem);

    std::cerr<<"3"<<std::endl;

    ros::spin();

    std::cerr<<"4"<<std::endl;

    return EXIT_SUCCESS;

//    const std::string data_folder = "/media/qzj/Extreme\ SSD/datasets/KAIST/urban28";
//    TicToc timer("vins", true);
//    TicToc timer_wheel("wheel", true);
//
//    std::cout<< data_folder <<std::flush<<std::endl;
//
//
//    // Create FilterFusion system.
//    FilterFusion::FilterFusionSystem FilterFusion_sys(param_file);
//
//    // Load all encoder data to buffer.
//    std::unordered_map<std::string, std::string> time_encoder_map;
//    if (!LoadSensorData(data_folder + "/sensor_data/encoder.csv", &time_encoder_map)) {
//        LOG(ERROR) << "[main]: Failed to load encoder data.";
//        return EXIT_FAILURE;
//    }
//
//    // Load all gps data to buffer.
//    std::unordered_map<std::string, std::string> time_gps_map;
//    if (!LoadSensorData(data_folder + "/sensor_data/gps.csv", &time_gps_map)) {
//        LOG(ERROR) << "[main]: Failed to load gps data.";
//        return EXIT_FAILURE;
//    }
//
//    // Load all gps data to buffer.
//    std::unordered_map<std::string, std::string> time_groundtruth;
//    if (!LoadSensorData(data_folder + "/global_pose.csv", &time_groundtruth)) {
//        LOG(ERROR) << "[main]: Failed to load groundtruth data.";
//        return EXIT_FAILURE;
//    }
//    std::vector<std::string> gt_time;
//    bool first = true;
//    Eigen::Matrix3d G_R_O;
//    Eigen::Vector3d G_p_O;
//    Eigen::Matrix3d G_R_O_init;
//    Eigen::Vector3d G_p_O_init;
//
//    LoadGT(data_folder + "/global_pose.csv", gt_time);
//
//    std::ifstream file_data_stamp(data_folder + "/sensor_data/data_stamp.csv");
//    if (!file_data_stamp.is_open()) {
//        LOG(ERROR) << "[main]: Failed to open data_stamp file.";
//        return EXIT_FAILURE;
//    }
//
//    std::vector<std::string> line_data_vec;
//    line_data_vec.reserve(3);
//    std::string line_str, value_str;
//    while (std::getline(file_data_stamp, line_str)) {
//        line_data_vec.clear();
//        std::stringstream ss(line_str);
//        while (std::getline(ss, value_str, ',')) { line_data_vec.push_back(value_str); }
//
//        constexpr double kToSecond = 1e-9;
//        const std::string& time_str = line_data_vec[0];
//        const double timestamp = std::stod(time_str) * kToSecond;
//
//        const std::string& sensor_type = line_data_vec[1];
//        printf("%s\r", sensor_type.c_str());
//        if (sensor_type == "stereo") {
//            timer.tic();
//            const std::string img_file = data_folder + "/image/stereo_left/" + time_str + ".png";
//            const cv::Mat raw_image = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);
//            if (raw_image.empty()) {
//                LOG(ERROR) << "[main]: Failed to open image at time: " << time_str;
//                return EXIT_FAILURE;
//            }
//
//            // Convert raw image to color image.
//            cv::Mat color_img;
//            cv::cvtColor(raw_image, color_img, cv::COLOR_BayerRG2RGB);
//
//            // Convert raw image to gray image.
//            cv::Mat gray_img;
//            cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);
//
//            // Feed image to system.
//            FilterFusion_sys.FeedImageData(timestamp, gray_img);
//            timer.toc();
//            timer.print();
//        }
//
//        if (sensor_type == "encoder") {
//            timer_wheel.tic();
//            if (time_encoder_map.find(time_str) == time_encoder_map.end()) {
//                LOG(ERROR) << "[main]: Failed to find encoder data at time: " << time_str;
//                return EXIT_FAILURE;
//            }
//            const std::string& encoder_str = time_encoder_map.at(time_str);
//            std::stringstream enc_ss(encoder_str);
//            line_data_vec.clear();
//            while (std::getline(enc_ss, value_str, ',')) { line_data_vec.push_back(value_str); }
//
//            const double left_enc_cnt = std::stod(line_data_vec[1]);
//            const double right_enc_cnt = std::stod(line_data_vec[2]);
//
//            // Feed wheel data to system.
//            FilterFusion_sys.FeedWheelData(timestamp, left_enc_cnt, right_enc_cnt);
//            timer_wheel.toc();
//            timer_wheel.print();
//        }
//
//        if (sensor_type == "gps") {
//            if (time_gps_map.find(time_str) == time_gps_map.end()) {
//                LOG(ERROR) << "[main]: Failed to find gps data at time: " << time_str;
//                return EXIT_FAILURE;
//            }
//            const std::string& gps_str = time_gps_map.at(time_str);
//            std::stringstream gps_ss(gps_str);
//            line_data_vec.clear();
//            while (std::getline(gps_ss, value_str, ',')) { line_data_vec.push_back(value_str); }
//
//            const double lat = std::stod(line_data_vec[1]);
//            const double lon = std::stod(line_data_vec[2]);
//            const double hei = std::stod(line_data_vec[3]);
//
//            Eigen::Matrix3d cov;
//            for (size_t i = 0; i < 9; ++i) {
//                cov.data()[i] = std::stod(line_data_vec[4+i]);
//            }
//
//            // Feed gps data to system.
//            FilterFusion_sys.FeedGpsData(timestamp, lon, lat, hei, cov);
//        }
//
//        std::string nearest = searchInsert(gt_time, time_str);
////        printf("%s, %s", nearest.c_str(), time_str.c_str());
//        if (time_groundtruth.find(nearest.c_str()) == time_groundtruth.end()) {
//            LOG(ERROR) << "[main]: Failed to find groundtruth data at time: " << nearest.c_str();
//            return EXIT_FAILURE;
//        }
//        const std::string& gt_str = time_groundtruth.at(nearest);
//        std::stringstream gt_ss(gt_str);
//        line_data_vec.clear();
//        while (std::getline(gt_ss, value_str, ',')) { line_data_vec.push_back(value_str); }
//        G_R_O << std::stod(line_data_vec[1]), std::stod(line_data_vec[2]), std::stod(line_data_vec[3]),
//                std::stod(line_data_vec[5]), std::stod(line_data_vec[6]), std::stod(line_data_vec[7]),
//                std::stod(line_data_vec[9]), std::stod(line_data_vec[10]), std::stod(line_data_vec[11]);
//        G_p_O << std::stod(line_data_vec[4]), std::stod(line_data_vec[8]), std::stod(line_data_vec[12]);
//
//        if(first){
//            G_R_O_init = G_R_O;
//            G_p_O_init = G_p_O;
//            first = false;
//        }
////        G_R_O = G_R_O_init.transpose() * G_R_O;
////        G_p_O = G_R_O_init.transpose() * G_p_O - G_R_O_init.transpose() * G_p_O_init;
//        G_p_O = G_p_O - G_p_O_init;
//        FilterFusion_sys.FeedGroundTruth(timestamp, G_R_O, G_p_O);
//    }
//
//    FilterFusion_sys.FileClose();
//
//    std::cin.ignore();
//
//    return EXIT_SUCCESS;
}