#ifndef EQUIRECTANGULAR_CONVERTOR_H_
#define EQUIRECTANGULAR_CONVERTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// opencv2
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "equirectangular_convertor/projection_mode.h"

class EquirectangularConvertor
{
public:
    EquirectangularConvertor();
    ~EquirectangularConvertor();
    void process();

private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    
    void set_projection_mode(std::string mode);
    void set_parameter(int cols,int rows);
    void make_map();
    void convert_image(cv::Mat& img);
    void convert_image_to_equirectangular(cv::Mat& img);
    void remove_rotation(cv::Mat& img);
    void display_image(std::string window_name,cv::Mat& img);
    int calc_rotation(cv::Mat& img);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber img_sub_;

    // publisher
    ros::Publisher img_pub_;

    // buffer
    int cols_;
    int rows_;
    int shift_;
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat prev_;

    // params
    const std::string INPUT_IMG_WINDOW_ = "Dualfisheye Image";
    const std::string OUTPUT_IMG_WINDOW_ = "Equirectangular Image";
    bool IS_VISUALIZE_;
    ProjectionMode projection_mode_;
};

#endif  // EQUIRECTANGULAR_CONVERTOR_H_
