//
// Created by nvidia on 9/12/19.
//

#include <cam_receiver/cam_receiver.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/filesystem.hpp>
#include <ctime>
#include <map>
#include <string>
#include <sstream>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "cam_receiver/helper.hpp"
#include <boost/thread.hpp>

PLUGINLIB_EXPORT_CLASS(CamReceiver::Drain, nodelet::Nodelet);

namespace CamReceiver {
    std::vector<std::string> Drain::channel_name_list{
            "depth", "rgb"
    };

    std::map<std::string, std::string> Drain::channel_local_folder{
            {"depth", "depth_images"},
            {"rgb",   "rgb_images"}
    };

    std::map<std::string, std::string> Drain::channel_topic_name{
            {"depth", "/camera/aligned_depth_to_color/image_raw"},
            {"rgb",   "/camera/color/image_raw"},
    };

    void Drain::onInit() {
        NODELET_INFO("Initializing Drain...");

        nh = getMTNodeHandle();
        nhp = getMTPrivateNodeHandle();

        this->loadParameters().createDirectories().defineSubscribers();

        timer = nhp.createTimer(ros::Duration(12), &Drain::timerCallback, this);
    }

    Drain& Drain::loadParameters() {
        if (!nhp.getParam("base_directory", base_directory)) {
            NODELET_WARN("base directory not found will use /home/nvidia");
            base_directory = std::string("/home/nvidia");
        } else {
            //TODO: obtain base directory from default constant when not defined
            NODELET_INFO("Images will be saved to base directory %s", base_directory.data());
        }

        // obtain channel acceptance
        bool temp;
        for (std::string &x : Drain::channel_name_list) {
            //if the channel name is defined as bool parameter, we will use that parameter
            //if not defined, we will use default FALSE
            if (nhp.getParam(x, temp)) {
                NODELET_INFO("%s parameter found = %s", x.data(), (temp?"true":"false"));
                this->channel_acceptance.emplace(x, temp);
            } else {
                //todo: change FALSE to some default constant
                NODELET_WARN("%s parameter not found, will use default false", x.data());
                this->channel_acceptance.emplace(x, false);
            }
        }

        return *this;
    }

    Drain& Drain::defineSubscribers() {

        if (channel_acceptance.at("depth")) {
            depth_sub.emplace_back(
                    nh.subscribe<sensor_msgs::Image>(Drain::channel_topic_name.at("depth"), 8, &Drain::depth_callback,
                                                     this));
        }

        if (channel_acceptance.at("rgb")) {
            rgb_sub.emplace_back(
                    nh.subscribe<sensor_msgs::Image>(Drain::channel_topic_name.at("rgb"), 8, &Drain::rgb_callback,
                                                     this));
        }

        NODELET_WARN("Drain callbacks defined fired!");
        return *this;
    }

    Drain& Drain::createDirectories() {

        std::time_t t = std::time(nullptr);
        std::tm *local_time = std::localtime(&t);

        //get the year_month_day_hour_min of this moment
        std::stringstream ss;
        ss << "/" << 1900 + (local_time->tm_year) << "_" << 1 + (local_time->tm_mon) << "_" << local_time->tm_mday
           << "_" << local_time->tm_hour << "_" << local_time->tm_min;

        //create directory
        boost::filesystem::path P{base_directory};
        P += ss.str();
        try {
            boost::filesystem::create_directories(P);
            base_directory = P.string();
        }
        catch (boost::filesystem::filesystem_error &) {
            std::cerr
                    << "Drain::createDirectories(): boost::filesystem::filesystem_error: Failed to create file directory\n"
                    << std::flush;
            abort();
        }
        NODELET_INFO("Directory created at %s", P.c_str());

        boost::filesystem::current_path(P);

        for (std::string &x : Drain::channel_name_list) {
            try {
                if (true == channel_acceptance.at(x)) {
                    boost::filesystem::create_directories(Drain::channel_local_folder.at(x));
                    NODELET_INFO("Created folder %s", Drain::channel_local_folder.at(x).data());
                } else {
                    NODELET_INFO("%s not created", Drain::channel_local_folder.at(x).data());
                }
            }
            catch(std::out_of_range& e){
                std::cerr<<"out_of_range error: "<<e.what()<<"\n"<<std::flush;
                NODELET_ERROR("The directory for channel \"%s\" will not be created", x.data());
            }

        }

        return *this;
    }

    //TODO:
    //Try to relocate the call of Drain::createDirectories() into Drain::onInit(). Information concerning the node namespaces might only be legit after onInit().
    Drain::Drain() {
        NODELET_INFO("Drain constructor called");
    }

    Drain::~Drain() {
        long int loss1 = depth_counter.getLoss();
        long int loss2 = rgb_counter.getLoss();
        unsigned long tot1 = depth_counter.getCount();
        unsigned long tot2 = rgb_counter.getCount();

        NODELET_INFO("depth loss:total = %li:%lu", loss1, tot1);
	NODELET_INFO("rgb loss:total = %li:%lu", loss2, tot2);
        NODELET_INFO("Drain destructed");
    }

    void Drain::depth_callback(const sensor_msgs::Image::ConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg);

        depth_counter.updateSeq(msg->header.seq);

        boost::async(boost::bind(&Drain::saveImage, this, cv_const_ptr, msg->header, CHANNEL::DEPTH));

        return;
        //NODELET_WARN("depth callback fired!");
    }


    void Drain::rgb_callback(const sensor_msgs::Image::ConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

        rgb_counter.updateSeq(msg->header.seq);

        boost::async(boost::bind(&Drain::saveImage, this, cv_const_ptr, msg->header, CHANNEL::RGB));

        //saveImage(cv_const_ptr, msg->header, CHANNEL::RGB);

        return;
        const std_msgs::Header &header_temp = msg->header;
        std::cout << "Header::seq=" << header_temp.seq << "\n"
                  << "Header::stamp=" << header_temp.stamp << "\n"
                  << "Header::frame_id=" << header_temp.frame_id << "\n"
                  << "encoding=" << msg->encoding << "\n"
                  << "is_bigendian=" << ((msg->is_bigendian) ? "True" : "False") << "\n"
                  << "step=" << msg->step << "\n"
                  << "height=" << msg->height << "\n"
                  << "width=" << msg->width << "\n";
        NODELET_WARN("rgb callback fired!\n");
    }

    Drain& Drain::saveImage(cv_bridge::CvImageConstPtr cv_const_ptr, const std_msgs::Header &header, CHANNEL channel_enum) {
        unsigned int seconds = header.stamp.sec;
        unsigned int nanoseconds = header.stamp.nsec;
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << seconds << "." << std::setw(9) << std::setfill('0') << nanoseconds;

        std::string myStr;

        switch (channel_enum) {

            case CHANNEL::RGB:
                //cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                myStr = this->base_directory + "/" + Drain::channel_local_folder.at("rgb") + "/" + ss.str() + ".jpg";
                break;

            case CHANNEL::DEPTH:
                //cv_const_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
                myStr = this->base_directory + "/" + Drain::channel_local_folder.at("depth") + "/" + ss.str() + ".png";
                break;

            default:
                NODELET_ERROR("save_image: channel type is neither depth nor rgb, re-check your code!");
                break;
        }

        try {
            cv::imwrite(myStr, cv_const_ptr->image);
        }
        catch (cv_bridge::Exception &e) {
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            return *this;
        }


        return *this;
    }

    void Drain::timerCallback(const ros::TimerEvent&) {
        NODELET_WARN_STREAM("Drain: Depth received "<<depth_counter.getCount()<<" frames, estimate of frame loss is "<<depth_counter.getLoss()<<" frames\n");
        NODELET_WARN_STREAM("Drain: RGB received "<<rgb_counter.getCount()<<" frames, estimate of frame loss is "<<rgb_counter.getLoss()<<" frames\n");
    }


}




