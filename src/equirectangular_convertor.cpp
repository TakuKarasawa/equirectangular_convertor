#include "equirectangular_convertor/equirectangular_convertor.h"

EquirectangularConvertor::EquirectangularConvertor() : 
    private_nh_("~"),
    cols_(0), rows_(0), shift_(0),
    map_x_(cv::Mat()), map_y_(cv::Mat()), prev_(cv::Mat())
{
    img_sub_ = nh_.subscribe("img_in",1,&EquirectangularConvertor::image_callback,this);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("img_out",1);

    std::string mode;
    private_nh_.param("MODE",mode,{std::string("equidistance")});
    set_projection_mode(mode);

    private_nh_.param("IS_VISUALIZE",IS_VISUALIZE_,{false});
    if(IS_VISUALIZE_){
        cv::namedWindow(INPUT_IMG_WINDOW_);
        cv::namedWindow(OUTPUT_IMG_WINDOW_);
    }
}

EquirectangularConvertor::~EquirectangularConvertor() { cv::destroyAllWindows(); }

void EquirectangularConvertor::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex){
        ROS_WARN("Could not convert to image");
        return;
    }

    cv::Mat cv_img(cv_ptr->image.rows,cv_ptr->image.cols,cv_ptr->image.type());
    cv_img = cv_ptr->image;
    set_parameter(cv_ptr->image.cols,cv_ptr->image.rows);
    convert_image(cv_img);
    cv::flip(cv_img,cv_img,1);
    cv::Rect rect(cv::Point(0,0),cv::Size(cols_,640));
    cv_img = cv_img(rect);

    // TO DO
    cv::Mat converted_img(cv_ptr->image.rows,cv_ptr->image.cols,cv_ptr->image.type());
    cv::Mat black_img = cv::Mat::zeros(cv::Size(cols_,80),CV_8UC3);
    cv::Mat top_img(converted_img,cv::Rect(cv::Point(0,0),cv::Size(cv_img.cols,cv_img.rows)));
    cv::Mat bottom_img(converted_img,cv::Rect(cv::Point(0,640),cv::Size(black_img.cols,black_img.rows)));
    cv_img.copyTo(top_img);
    black_img.copyTo(bottom_img);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",converted_img).toImageMsg();
    img_msg->header = msg->header;
    img_pub_.publish(img_msg);

    if(IS_VISUALIZE_){
        display_image(INPUT_IMG_WINDOW_,cv_ptr->image);
        display_image(OUTPUT_IMG_WINDOW_,converted_img);
    }
}

void EquirectangularConvertor::set_projection_mode(std::string mode)
{
    if(mode == "equidistance") projection_mode_ = ProjectionMode::Equidistance;
    else if(mode == "stereographic") projection_mode_ = ProjectionMode::Stereographic;
    else if(mode == "inverse_stereographic") projection_mode_ = ProjectionMode::InverseStereographic;
    else if(mode == "orthographic") projection_mode_ = ProjectionMode::Orthographic;
    else if(mode == "inverse_orthographic") projection_mode_ = ProjectionMode::InverseOrthographic;
    else{
        ROS_WARN("No applicable 'projection_mode'.");
        ROS_WARN("Please select ['equidistance', 'stereographic', 'inverse_stereographic', 'orthographic', 'inverse_orthographic']");
        ROS_INFO("Set 'equidistance'");
        projection_mode_ = ProjectionMode::Equidistance;
    }
}


void EquirectangularConvertor::set_parameter(int cols,int rows)
{
    cols_ = cols;
    rows_ = rows;
    shift_ = 0;
    map_x_ = cv::Mat(cv::Size(cols_,rows_),CV_32FC1);
    map_y_ = cv::Mat(cv::Size(cols_,rows_),CV_32FC1);
    make_map();
}

void EquirectangularConvertor::make_map()
{
    float dst_x = static_cast<float>(cols_);
    float dst_y = dst_x/2;
    float src_cx1 = dst_x/4;
    float src_cy1 = dst_x/4;
    float src_cx2 = dst_x - src_cx1;
    float src_r = 0.884*dst_x/4;
    float src_rx = src_r;
    float src_ry = src_r;

    for(int y = 0; y < dst_y; y++){
        for(int x = 0; x < cols_; x++){
            float phi1 = M_PI*x/dst_y;
            float theta1 = M_PI*y/dst_y;

            float x1 = std::sin(theta1)*std::cos(phi1);
            float y1 = std::sin(theta1)*std::sin(phi1);
            float z1 = std::cos(theta1);

            float phi2 = std::acos(-x1);
            float theta2 = (y1 >= 0 ? 1 : -1)*std::acos(-z1/std::sqrt(y1*y1 + z1*z1));

            float r0;
            if(phi2 < M_PI/2){
                if(projection_mode_ == ProjectionMode::Equidistance) r0 = phi2/(M_PI/2);
                else if(projection_mode_ == ProjectionMode::Stereographic) r0 = std::tan(phi2/2);
                else if(projection_mode_ == ProjectionMode::InverseStereographic) r0 = 1 - std::tan((M_PI/2 - phi2)/2);
                else if(projection_mode_ == ProjectionMode::Orthographic) r0 = std::sin(phi2);
                else if(projection_mode_ == ProjectionMode::InverseOrthographic) r0 = 1 - std::sin(M_PI/2 - phi2);
                map_x_.at<float>(y,x) = src_rx*r0*std::cos(M_PI - theta2) + src_cx2;
                map_y_.at<float>(y,x) = src_ry*r0*std::sin(M_PI - theta2) + src_cy1;
            }
            else{
                if(projection_mode_ == ProjectionMode::Equidistance) r0 = (M_PI - phi2)/(M_PI/2);
                else if(projection_mode_ == ProjectionMode::Stereographic) r0 = std::tan((M_PI - phi2)/2);
                else if(projection_mode_ == ProjectionMode::InverseStereographic) r0 = 1 - std::tan((-M_PI/2 + phi2)/2);
                else if(projection_mode_ == ProjectionMode::Orthographic) r0 = std::cos(M_PI - phi2);
                else if(projection_mode_ == ProjectionMode::InverseOrthographic) r0 = 1 - std::sin(-M_PI/2 + phi2);
                map_x_.at<float>(y,x) = src_rx*r0*std::cos(theta2) + src_cx1;
                map_y_.at<float>(y,x) = src_ry*r0*std::sin(theta2) + src_cy1;
            }
        }
    }
}

void EquirectangularConvertor::convert_image(cv::Mat& img)
{
    convert_image_to_equirectangular(img);
    remove_rotation(img);
}

void EquirectangularConvertor::convert_image_to_equirectangular(cv::Mat& img)
{
    cv::Mat buf = cv::Mat(img.size(),img.type());
    cv::remap(img,buf,map_x_,map_y_,cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
    buf.copyTo(img);
}

void EquirectangularConvertor::remove_rotation(cv::Mat& img)
{
    shift_ += calc_rotation(img);

    if(shift_ >= cols_) shift_ -= cols_;
    if(shift_ < 0) shift_ += cols_;
    if(shift_ != 0){
        cv::Mat buf;
        cv::Rect r1(cv::Point(0,0),cv::Size(shift_,rows_));
        cv::Rect r2(cv::Point(shift_,0),cv::Size(cols_ - shift_,rows_));
        cv::hconcat(img(r2),img(r1),buf);
        buf.copyTo(img);
    }
}

int EquirectangularConvertor::calc_rotation(cv::Mat& img)
{
    const int y1 = rows_/2;
    cv::Rect r1(0,y1,cols_,1);
    cv::Mat l1(img,r1);
    cv::Mat m1;
    cv::cvtColor(l1,m1,cv::COLOR_RGB2GRAY);
    int ret = 0;

    if(prev_.cols == cols_){
        int w0 = cols_/12;
        cv::Rect rs0(0,0,w0,1);
        cv::Mat t1;
        cv::hconcat(prev_,prev_(rs0),t1);
        cv::Rect rs1(cols_ - w0,0,w0,1);
        cv::Mat b1;
        cv::hconcat(prev_(rs1),t1,b1);
        cv::Mat result1;
        cv::matchTemplate(b1,m1,result1,cv::TM_CCOEFF);
        cv::Point p1;
        double v1;
        minMaxLoc(result1,NULL,&v1,NULL,&p1);
        ret = w0 - p1.x;
    }
    m1.copyTo(prev_);

    return ret;
}

void EquirectangularConvertor::display_image(std::string window_name,cv::Mat& img)
{
    cv::imshow(window_name,img);
    cv::waitKey(1);
}

void EquirectangularConvertor::process() { ros::spin(); }