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

//        image_topic = "/hikrobot_camera/rgb";
//        encoder_topic = "/current_speed";
//        imu_topic = "/imu/data";

        std::cerr<<"5"<<std::endl;

        subImage = nh.subscribe<sensor_msgs::ImageConstPtr>(image_topic, 50, &System::ImageHandler, this);
        subEncoder = nh.subscribe<geometry_msgs::TwistStampedConstPtr>(encoder_topic, 50, &System::EncoderHandler, this);
        subIMU = nh.subscribe<sensor_msgs::ImuConstPtr>(imu_topic, 50, &System::IMUHandler, this);

        mpFilterFusion = std::make_shared<FilterFusion::FilterFusionSystem>(param_file);

        std::cerr<<"6"<<std::endl;
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
            std_msgs::Header header_img;
            double time_img = 0;

            m_buf.lock();
            if(!img_buf.empty())
            {
                time_img = img_buf.front()->header.stamp.toSec();
                header_img = img_buf.front()->header;
                image = getImageFromMsg(img_buf.front());
                img_buf.pop();

                std::cerr<<"7"<<std::endl;
            }
            m_buf.unlock();



            if(!image.empty())
            {
                cv::imshow("image",image);
                cv::waitKey(10);
                mpFilterFusion->FeedImageData(time_img, image);
            }


            m_buf.lock();

            double left_enc_cnt = 0;
            double right_enc_cnt = 0;
            std_msgs::Header header_encoder;
            double time_encoder_temp = 0;
            double time_encoder = 0;

            if(!encoder_buf.empty())
            {

                static int count = -1;
                count ++;

                double timestamp0 = 1544590798713209835/1e9;

                time_encoder_temp = encoder_buf.front()->header.stamp.toSec();
                double delta_t = 2*(time_encoder_temp-timestamp0);


//                std::cerr<<"aaaaaa"<<std::endl;
//
//                std::cerr<<"timestamp0"<<timestamp0<<std::endl;
//
//                std::cerr<<"time_encoder_temp"<<time_encoder_temp<<std::endl;
//                std::cerr<<"delta_t"<<delta_t<<std::endl;

                std::cerr<<"8"<<std::endl;

                if(count)
                {
                    timestamp0 = time_encoder_temp;
                }
                time_encoder = timestamp0+delta_t;

                std::cerr<<"time_encoder"<<time_encoder<<std::endl;

                header_encoder = encoder_buf.front()->header;

                left_enc_cnt = getleft_enc_cnt(encoder_buf.front());
                right_enc_cnt = getright_enc_cnt(encoder_buf.front());

                encoder_buf.front();
//                const double left_enc_cnt = std::stod(line_data_vec[1]);
//                const double right_enc_cnt = std::stod(line_data_vec[2]);

                // Feed wheel data to system.
                mpFilterFusion->FeedWheelData(time_encoder, left_enc_cnt, right_enc_cnt);

                encoder_buf.pop();
            }
            m_buf.unlock();

            std::cerr<<left_enc_cnt<<std::endl;
            std::cerr<<right_enc_cnt<<std::endl;


                std::cerr<<"time_encoder"<<time_encoder<<std::endl;

            if(right_enc_cnt)
                mpFilterFusion->FeedWheelData(time_encoder, left_enc_cnt, right_enc_cnt);



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
//            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat img = ptr->image.clone();
        std::cerr<<"9"<<std::endl;
        return img;
    }

    double getleft_enc_cnt(const geometry_msgs::TwistStampedConstPtr &encoder_msgs)
    {

        static int count = -1;
        count ++;

        double kl = 0.00047820240382508;
        double kr = 0.00047768621928995;
        double b = 1.52439;

        double timestamp0 = 1544590798713209835/1e9;
        double left_enc_cnt0 = 15902462;

        double left_enc_cnt = 0;

        geometry_msgs::TwistStamped encoder;
        double vx = encoder_msgs->twist.linear.x;
        double vy = encoder_msgs->twist.linear.y;
        double vz = encoder_msgs->twist.linear.z;
        double ax = encoder_msgs->twist.angular.x;
        double ay = encoder_msgs->twist.angular.y;
        double az = encoder_msgs->twist.angular.z;
        double timestamp = encoder_msgs->header.stamp.toSec();

        double delta_t = 2*(timestamp-timestamp0);
        double delta_l = (double)(2*delta_t*vx-az*b)/(double)(2*kl);

        if(count)
        {
            left_enc_cnt0 = left_enc_cnt;
            timestamp0 = timestamp;
        }

        left_enc_cnt = left_enc_cnt0+delta_l;

        std::cerr<<"10"<<std::endl;
        std::cerr<<"left_enc_cnt"<<left_enc_cnt<<std::endl;

        return left_enc_cnt;
    }

    double getright_enc_cnt(const geometry_msgs::TwistStampedConstPtr &encoder_msgs)
    {
        static int count = -1;
        count ++;

        double kl = 0.00047820240382508;
        double kr = 0.00047768621928995;
        double b = 1.52439;

        double timestamp0 = 1544590798713209835/1e9;
        double right_enc_cnt0 = 15902462;

        double right_enc_cnt = 0;

        geometry_msgs::TwistStamped encoder;
        double vx = encoder_msgs->twist.linear.x;
        double vy = encoder_msgs->twist.linear.y;
        double vz = encoder_msgs->twist.linear.z;
        double ax = encoder_msgs->twist.angular.x;
        double ay = encoder_msgs->twist.angular.y;
        double az = encoder_msgs->twist.angular.z;
        double timestamp = encoder_msgs->header.stamp.toSec();



        double delta_t = 2*(timestamp-timestamp0);
        double delta_l = (double)(2*delta_t*vx+az*b)/(double)(2*kr);

        if(count)
        {
            right_enc_cnt0 = right_enc_cnt;
        }

        right_enc_cnt = right_enc_cnt0+delta_l;

//        std::cerr<<"11"<<std::endl;
//        std::cerr<<"right_enc_cnt0"<<right_enc_cnt0<<std::endl;
//        std::cerr<<"timestamp"<<timestamp<<std::endl;
//        std::cerr<<"count"<<count<<std::endl;
//        std::cerr<<"delta_t"<<delta_t<<std::endl;
//        std::cerr<<"vx"<<vx<<std::endl;
//        std::cerr<<"az"<<az<<std::endl;

        std::cerr<<"right_enc_cnt"<<right_enc_cnt<<std::endl;

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
    std::queue<geometry_msgs::TwistStampedConstPtr> encoder_buf;

};

#endif //SRC_SYSTEM_H
