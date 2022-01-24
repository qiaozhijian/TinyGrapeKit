#include <string>
#include "Timer.h"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <FilterFusion/Visualizer.h>
#include "System.h"
#include "Util/Util.hpp"
#include "Util/file_manager.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "RunKAISTData");
    ros::NodeHandle nh("~");

    if (argc != 2) {
        LOG(ERROR) << "[main]: Please input param_file.";
        return EXIT_FAILURE;
    }

    string project_dir, bag_path;
    nh.param<std::string>("project_dir", project_dir, "blank_dir");
    nh.param<std::string>("rosbag_path", bag_path, "rosbag_path");
    LOG(INFO) << "Load project_dir " << project_dir;
    LOG(INFO) << "Load rosbag_path " << bag_path;

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = project_dir + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    TGK::FileManager::CreateDirectory(FLAGS_log_dir);

    const std::string param_file = argv[1];
    LOG(INFO) << "Load param_file " << param_file;
    shared_ptr<System> mpSystem = std::make_shared<System>(nh, param_file);

    rosbag::Bag bag;
    if (!bag_path.empty()) {
        try {
            bag.open(bag_path, rosbag::bagmode::Read);
        } catch (std::exception& ex) {
            ROS_FATAL("Unable to open rosbag [%s]", bag_path.c_str());
            return 1;
        }
    }

    rosbag::View viewFull(bag);
    auto start_sim_time = viewFull.getBeginTime();
    rosbag::View view(bag, start_sim_time);
    auto start_real_time = std::chrono::high_resolution_clock::now();
    auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",200);
    rosgraph_msgs::Clock clock_msg;

    cv::Mat image;
    double left_enc_cnt, right_enc_cnt;
    double time_img, time_encoder;
    for(const rosbag::MessageInstance& m: view)
    {
        if(!ros::ok())
            break;

        clock_msg.clock = m.getTime();
        clock_publisher.publish( clock_msg );
        // Handle IMU measurement
        nav_msgs::OdometryConstPtr encoder = m.instantiate<nav_msgs::Odometry>();
        if (encoder != nullptr && m.getTopic() == mpSystem->encoder_topic) {
//            LOG(INFO) << std::fixed << "time encoder: " << time_encoder;
            time_encoder = encoder->header.stamp.toSec();
            left_enc_cnt = encoder->twist.twist.linear.x;
            right_enc_cnt = encoder->twist.twist.linear.y;
            mpSystem->mpFilterFusion->FeedWheelData(time_encoder, left_enc_cnt, right_enc_cnt);
        }

        const sensor_msgs::ImageConstPtr &pImage = m.instantiate<sensor_msgs::Image>();
        if (pImage != nullptr && m.getTopic() == mpSystem->image_topic){
//            LOG(INFO) << std::fixed << "time_img: " << time_img;
            time_img = pImage->header.stamp.toSec();
            image = mpSystem->getImageFromMsg(pImage);
            mpSystem->mpFilterFusion->FeedImageData(time_img, image);
        }
    }

    bag.close();

    return 0;
}