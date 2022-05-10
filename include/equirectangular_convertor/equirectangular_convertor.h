#ifndef EQUIRECTANGULAR_CONVERTOR_H_
#define EQUIRECTANGULAR_CONVERTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class EquirectangularConvertor
{
public:
    EquirectangularConvertor();
    ~EquirectangularConvertor();
    void process();

private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void set_parameter(int cols,int rows);
    void make_map();
    void convert_image(cv::Mat& img);
    void convert_image_to_equirectangular(cv::Mat& img);
    void remove_rotation(cv::Mat& img);
    void display_image(std::string window_name,cv::Mat& img);
    int calc_rotation(cv::Mat& img);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber dual_sub_;
    ros::Publisher equi_pub_;

    std::string DUAL_WINDOW_ = "Dualfisheye Image";
    std::string EQUI_WINDOW_ = "Equirectangular Image";

    std::string dual_topic_name_;
    std::string equi_topic_name_;
    bool is_visualize_;
    int mode_;

    // ---------- mode ----------
    // 0: 等距離射影
    // 1: 立体射影
    // 2: 立体射影逆変換
    // 3: 正射影
    // 4: 正射影逆変換
    // --------------------------

    int cols_;
    int rows_;
    int shift_;
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat prev_;
};

#endif  // EQUIRECTANGULAR_CONVERTOR_H_
