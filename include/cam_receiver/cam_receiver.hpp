//
// Created by nvidia on 9/12/19.
//

#ifndef SRC_CAM_RECEIVER_HPP
#define SRC_CAM_RECEIVER_HPP

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <vector>
#include <string>
#include <map>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "cam_receiver/helper.hpp"

namespace CamReceiver {

    enum class CHANNEL{
        DEPTH,
        RGB
    };

    class Drain final: public nodelet::Nodelet {
    public:
        virtual void onInit() ;
        Drain();
        ~Drain() override;
        Drain& loadParameters();//load parameters from config file
        Drain& defineSubscribers();
        Drain& createDirectories();
        void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
        void rgb_callback(const sensor_msgs::Image::ConstPtr& msg);
        Drain& saveImage(cv_bridge::CvImageConstPtr, const std_msgs::Header&, CHANNEL);
        void timerCallback(const ros::TimerEvent& event);

    private:
        ros::NodeHandle nh; //node handle
        ros::NodeHandle nhp; //private node handle

        ros::Timer timer;

        std::vector<ros::Subscriber> depth_sub;
        std::vector<ros::Subscriber> rgb_sub;

        helper::counter depth_counter;
        helper::counter rgb_counter;

        static std::vector<std::string> channel_name_list;
        static std::map<std::string, std::string> channel_local_folder;
        static std::map<std::string, std::string> channel_topic_name;

        //parameters to obtain from launch parameters
        std::string base_directory;
        std::map<std::string, bool> channel_acceptance;

    };
}


#endif //SRC_CAM_RECEIVER_HPP
