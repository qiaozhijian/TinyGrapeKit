//
// Created by qzj on 2022/1/19.
//

#ifndef SRC_SYSTEM_H
#define SRC_SYSTEM_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>
#include <thread>
#include <FilterFusion/FilterFusionSystem.h>

class System{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System(ros::NodeHandle &nh, const std::string& param_file){

        cv::FileStorage cv_params(param_file, cv::FileStorage::READ);

        image_topic = cv_params["image_topic"].string();
        encoder_topic = cv_params["encoder_topic"].string();
        imu_topic = cv_params["imu_topic"].string();

        subImage = nh.subscribe<sensor_msgs::ImageConstPtr>(image_topic, 50, &System::ImageHandler, this);
        subEncoder = nh.subscribe<geometry_msgs::TwistStampedConstPtr>(encoder_topic, 50, &System::EncoderHandler, this);
        subIMU = nh.subscribe<sensor_msgs::ImuConstPtr>(imu_topic, 50, &System::IMUHandler, this);

        mpFilterFusion = std::make_shared<FilterFusion::FilterFusionSystem>(param_file);
    }

    void ImageHandler(sensor_msgs::ImageConstPtr msg){
        m_buf.lock();
        img_buf.push(msg);
        m_buf.unlock();
    }

    void EncoderHandler(geometry_msgs::TwistStampedConstPtr msg){
        m_buf.lock();
        encoder_buf.push(msg);
        m_buf.unlock();
    }

    void IMUHandler(sensor_msgs::ImuConstPtr msg){
        m_buf.lock();
        imu_buf.push(msg);
        m_buf.unlock();
    }

    void sync_process()
    {
        while(1)
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img_buf.empty())
            {
                time = img_buf.front()->header.stamp.toSec();
                header = img_buf.front()->header;
                image = getImageFromMsg(img_buf.front());
                img_buf.pop();
            }
            m_buf.unlock();

            if(!image.empty())
                mpFilterFusion->FeedImageData(time, image);


            m_buf.lock();
            if(!encoder_buf.empty())
            {
                time = encoder_buf.front()->header.stamp.toSec();
                header = encoder_buf.front()->header;
                encoder_buf.front();
//                const double left_enc_cnt = std::stod(line_data_vec[1]);
//                const double right_enc_cnt = std::stod(line_data_vec[2]);

                // Feed wheel data to system.
//                mpFilterFusion->FeedWheelData(time, left_enc_cnt, right_enc_cnt);

                encoder_buf.pop();
            }
            m_buf.unlock();

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat img = ptr->image.clone();
        return img;
    }

private:

    std::shared_ptr<FilterFusion::FilterFusionSystem> mpFilterFusion;

    std::string image_topic;
    std::string encoder_topic;
    std::string imu_topic;

    ros::Subscriber subImage;
    ros::Subscriber subEncoder;
    ros::Subscriber subIMU;

    std::mutex m_buf;
    std::queue<sensor_msgs::ImuConstPtr> imu_buf;
    std::queue<sensor_msgs::ImageConstPtr> img_buf;
    std::queue<geometry_msgs::TwistStampedConstPtr> encoder_buf;

};

#endif //SRC_SYSTEM_H
