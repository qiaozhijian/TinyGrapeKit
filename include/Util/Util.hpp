#ifndef UTILS_H__H
#define UTILS_H__H

#include <fstream>
#include <Eigen/Core>
#include<unordered_map>
#include <glog/logging.h>

namespace TGK {
namespace Util {

    inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d w;
        w <<  0.,   -v(2),  v(1),
              v(2),  0.,   -v(0),
             -v(1),  v(0),  0.;
        return w;
    }

    inline bool LoadSensorData(const std::string& encoder_file_path, std::unordered_map<std::string, std::string>* time_data_map) {
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

    inline bool LoadGT(const std::string& encoder_file_path, std::vector<std::string>& gt_time) {
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

//二分法查最近的数
    inline std::string searchInsert(std::vector<std::string>& nums, const std::string& target) {
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

}  // namespace Util
}  // namespace TGK

#endif