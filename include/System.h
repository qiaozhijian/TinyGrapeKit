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
#include <nav_msgs/Odometry.h>

class System{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System(ros::NodeHandle &nh, const std::string& param_file){

        cv::FileStorage cv_params(param_file, cv::FileStorage::READ);

        image_topic = cv_params["image_topic"].string();
        encoder_topic = cv_params["encoder_topic"].string();
        imu_topic = cv_params["imu_topic"].string();

        LOG(INFO) << "Subscribe image_topic " << image_topic;
        LOG(INFO) << "Subscribe encoder_topic " << encoder_topic;
        LOG(INFO) << "Subscribe imu_topic " << imu_topic;
        subImage = nh.subscribe<sensor_msgs::ImageConstPtr>(image_topic, 50, &System::ImageHandler, this);
//        subEncoder = nh.subscribe<geometry_msgs::TwistStampedConstPtr>(encoder_topic, 50, &System::EncoderHandler, this);
        subEncoder = nh.subscribe<nav_msgs::OdometryConstPtr>(encoder_topic, 50, &System::EncoderHandler, this);
        subIMU = nh.subscribe<sensor_msgs::ImuConstPtr>(imu_topic, 50, &System::IMUHandler, this);

        mpFilterFusion = std::make_shared<FilterFusion::FilterFusionSystem>(param_file);
    }

    void ImageHandler(sensor_msgs::ImageConstPtr msg){
        m_buf.lock();
        img_buf.push(msg);
        m_buf.unlock();
    }

//    void EncoderHandler(geometry_msgs::TwistStampedConstPtr msg){
//        m_buf.lock();
//        encoder_buf.push(msg);
//        m_buf.unlock();
//    }
    void EncoderHandler(nav_msgs::OdometryConstPtr msg){
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
        cv::Mat image;
        double left_enc_cnt, right_enc_cnt;
        double time_img, time_encoder;
        bool encoder_ready = false;
        bool camera_ready = false;
        while(ros::ok())
        {
            m_buf.lock();
            if(!encoder_buf.empty())
            {
                time_encoder = encoder_buf.front()->header.stamp.toSec();
                left_enc_cnt = getleft_enc_cnt(encoder_buf.front());
                right_enc_cnt = getright_enc_cnt(encoder_buf.front());

                encoder_ready = true;
                encoder_buf.pop();
            }
            m_buf.unlock();

            if(encoder_ready){
                encoder_ready = false;
                mpFilterFusion->FeedWheelData(time_encoder, left_enc_cnt, right_enc_cnt);
            }

            m_buf.lock();
            if(!img_buf.empty())
            {
                time_img = img_buf.front()->header.stamp.toSec();
                image = getImageFromMsg(img_buf.front());
                camera_ready = true;
                img_buf.pop();
            }
            m_buf.unlock();

            if(camera_ready)
            {
                camera_ready = false;
                mpFilterFusion->FeedImageData(time_img, image);
            }

//            double left_enc_cnt = 0;
//            double right_enc_cnt = 0;
//            double time_encoder_temp = 0;
//            double time_encoder = 0;
//            m_buf.lock();
//
//            if(!encoder_buf.empty())
//            {
//
//                static int count = -1;
//                count ++;
//                static double timestamp0 = 1544590798713209835/1e9;
//                time_encoder_temp = encoder_buf.front()->header.stamp.toSec();
//
//                if(count)
//                {
//                    timestamp0 = time_encoder;
//                }
//                time_encoder = 2*time_encoder_temp-timestamp0;
//
//                std::cerr<<"time_encoder"<<time_encoder<<std::endl;
//
//                left_enc_cnt = getleft_enc_cnt(encoder_buf.front());
//                right_enc_cnt = getright_enc_cnt(encoder_buf.front());
//                encoder_buf.pop();
//            }
//            m_buf.unlock();
//
//            if(right_enc_cnt)
//                mpFilterFusion->FeedWheelData(time_encoder, left_enc_cnt, right_enc_cnt);

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
            mpFilterFusion->FileClose();

        }
        return ;
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
//            img.is_bigendian = img_msg->is_bigendian;
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

//    double getleft_enc_cnt(const geometry_msgs::TwistStampedConstPtr &encoder_msgs)
//    {
//
//        static int countl = -1;
//        countl ++;
//
//        double kl = 0.00047820240382508;
//        double kr = 0.00047768621928995;
//        double b = 1.52439;
//
//        static double timestamp0l = 1544590798713209835/1e9;
//        static double left_enc_cnt0 = 15902462;
//
//        double left_enc_cnt = 0;
//
//        geometry_msgs::TwistStamped encoder;
//        double vx = encoder_msgs->twist.linear.x;
//        double vy = encoder_msgs->twist.linear.y;
//        double vz = encoder_msgs->twist.linear.z;
//        double ax = encoder_msgs->twist.angular.x;
//        double ay = encoder_msgs->twist.angular.y;
//        double az = encoder_msgs->twist.angular.z;
//        double timestamp = encoder_msgs->header.stamp.toSec();
//
//
//        double delta_t= 2*timestamp-timestamp0l;
//
//        left_enc_cnt = left_enc_cnt0 + (4*vx*(timestamp-timestamp0l)-az*b)/(2*kl);
//
//        if(countl)
//        {
//            left_enc_cnt0 = left_enc_cnt;
//            timestamp0l = delta_t;
//        }
//
//        return left_enc_cnt;
//    }
//
//    double getright_enc_cnt(const geometry_msgs::TwistStampedConstPtr &encoder_msgs)
//    {
//        static int countr = -1;
//        countr ++;
//
//        double kl = 0.00047820240382508;
//        double kr = 0.00047768621928995;
//        double b = 1.52439;
//
//        static double timestamp0r = 1544590798713209835/1e9;
//        static double right_enc_cnt0 = 15902462;
//
//        double right_enc_cnt = 0;
//
//        geometry_msgs::TwistStamped encoder;
//        double vx = encoder_msgs->twist.linear.x;
//        double vy = encoder_msgs->twist.linear.y;
//        double vz = encoder_msgs->twist.linear.z;
//        double ax = encoder_msgs->twist.angular.x;
//        double ay = encoder_msgs->twist.angular.y;
//        double az = encoder_msgs->twist.angular.z;
//        double timestamp = encoder_msgs->header.stamp.toSec();
//
//
//        double delta_t= 2*timestamp-timestamp0r;
//
//        right_enc_cnt = right_enc_cnt0 + (4*vx*(timestamp-timestamp0r)-az*b)/(2*kl);
//
//        if(countr)
//        {
//            right_enc_cnt0 = right_enc_cnt;
//            timestamp0r = delta_t;
//        }
//
//        return right_enc_cnt;
//    }
    double getleft_enc_cnt(const nav_msgs::OdometryConstPtr &encoder_msgs)
    {
        nav_msgs::Odometry encoder;
        double left_enc_cnt = encoder_msgs->twist.twist.linear.x;
        return left_enc_cnt;
    }
    double getright_enc_cnt(const nav_msgs::OdometryConstPtr &encoder_msgs)
    {
        nav_msgs::Odometry encoder;
        double right_enc_cnt = encoder_msgs->twist.twist.linear.y;
        return right_enc_cnt;
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
//    std::queue<geometry_msgs::TwistStampedConstPtr> encoder_buf;
    std::queue<nav_msgs::OdometryConstPtr> encoder_buf;
};

#endif //SRC_SYSTEM_H
