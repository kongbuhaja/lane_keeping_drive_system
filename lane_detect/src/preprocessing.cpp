#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/vision.h"

const std::string toString(const std::vector<cv::Vec4i> &v);

const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 422.037858, 0.000000, 245.895397, 0.000000, 435.589734, 163.625535, 0.000000, 0.000000, 1.000000);
const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.000000);




alcoholdriving::LineDetector::LineDetector(ros::NodeHandle nh) : nh_(nh)
{
    cam_sub_= nh_.subscribe("/usb_cam/image_raw", 1, &alcoholdriving::LineDetector::camCallback, this);
    ma = MovingAverage(MA_SIZE);
    flines.first = {.0f, .0f, .0f};
    flines.second = {float(WIDTH), 0.f, 0.f};
}

alcoholdriving::LineDetector::~LineDetector()
{
    /* TODO Destroy Objects in LineDetector*/
}

void alcoholdriving::LineDetector::preprocessing(const cv::Mat &img, cv::Mat &dst)
{
    cv::Mat undistorted_img, gray; 
    cv::undistort(img, undistorted_img, cameraMatrix, distCoeffs);
    cv::cvtColor(undistorted_img, gray, cv::COLOR_BGR2GRAY);
    dst = gray(cv::Rect(0, OFFSET, WIDTH, GAP));
}

void alcoholdriving::LineDetector::histStretching(cv::Mat &img){
    int gmin, gmax;
        gmin = img.at<uchar>(0,0);
        gmax = gmin;
        for(int y=0; y<img.rows; y++){
            for(int x=0; x<img.cols; x++){
                int value = img.at<uchar>(y,x);
                if(value<gmin) gmin=value;
                else if(value>gmax) gmax=value;
            }
        }
        for(int y=0; y<img.rows; y++){
            for(int x=0; x<img.cols; x++){
                img.at<uchar>(y,x) = (img.at<uchar>(y,x) - gmin)*255 / (gmax - gmin);
            }
        }
}

void alcoholdriving::LineDetector::binarization(const cv::Mat &src, cv::Mat &dst)
{
    cv::Mat th_img = src;
    histStretching(th_img);
    cv::threshold(th_img, th_img, 220, 255, cv::THRESH_BINARY);
    gaussianBlur(th_img,th_img);
    gaussianBlur(th_img,th_img);
    cv::threshold(th_img, th_img, 220, 255, cv::THRESH_BINARY);
    dst = th_img;
    // gray = gray + (gray - mean[0]) * 6.0;
    // cv::GaussianBlur(gray,blur, cv::Size(3,3), 0);
}

float alcoholdriving::LineDetector::hough(const cv::Mat &th_img)
{
    //ROS_ERROR("hough");
    float mpos;
    cv::Mat edge_img, roi, roi_th, canny;
    std::vector<cv::Vec4i> lines, left_lines, right_lines;
    
    cv::Canny(th_img, canny, LOW_THRESHOLD, HIGH_THRESHOLD);
    cv::HoughLinesP(canny, lines, RHO, THETA, 10, 10, 3);

    divide_lines(lines, left_lines, right_lines);
    
    get_line_pos(left_lines, true);
    get_line_pos(right_lines, false);
       
    // if(flines.first[1]==flines.second[1])
    //     mpos=((flines.first[0]+flines.second[0])/2);
    // else{
    //     mpos = (-(flines.first[2]-flines.second[2])/(flines.first[1]-flines.second[1]));
    //     if(mpos>=WIDTH)
    //         mpos=WIDTH-1;
    //     else if(mpos<0)
    //         mpos=0;
    // }
    mpos=((flines.first[0]+flines.second[0])/2);

    ma.add_sample(mpos);
    ROS_ERROR("lpos: %f, rpos: %f, mpos: %f",flines.first[0], flines.second[0], ma.get_wmm());
    return (ma.get_wmm() - WIDTH * 0.5)/(WIDTH*0.5);
}

void alcoholdriving::LineDetector::divide_lines(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines)
{
    //ROS_ERROR("divide_lines");
    std::vector<float> slopes;
    std::vector<cv::Vec4i> new_lines;
    float slope;
        
    if(lines.size()==0){
        left_lines = {};
        right_lines = {};
        return;
    }

    for(int i=0; i<lines.size(); i++){
        if((lines[i][2] - lines[i][0])==0){slope = 0.0;}
        else{slope = float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);}

        if((slope > -SLOPE_THRESHOLD) && (slope < SLOPE_THRESHOLD)){
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
    }

    for(int i=0; i<slopes.size(); i++){
        if((slopes[i] < 0) && (new_lines[i][2] < WIDTH/2)) {left_lines.push_back(new_lines[i]);}
        else if((slopes[i]) > 0 && (new_lines[i][0] > WIDTH/2)) {right_lines.push_back(new_lines[i]);}
    }
}

void alcoholdriving::LineDetector::get_line_pos(std::vector<cv::Vec4i> &lines, bool left)
{
    
    float m, b;
    get_line_params(lines, m, b);
    int pos;
    if(m==0 && b==0){
        if(left) pos=0;
        if(!left) pos=WIDTH;
    }
    else{
        int y = GAP/2;
        pos = (y-b) / m;
	if(left){
		pos = std::min(std::max(0,pos),320);
		flines.first={float(pos), m, b};
	}
       	else{
		pos = std::min(std::max(320,pos),640);
		flines.second={float(pos), m, b};
	}
    }
    //ROS_ERROR("get_line_pos pos :%.d", pos);
}

void alcoholdriving::LineDetector::get_line_params(std::vector<cv::Vec4i> &lines, float &m, float &b)
{
    
    float x_sum = 0.0;
    float y_sum = 0.0;
    float m_sum = 0.0;

    if (lines.size() == 0)
    {
        m = 0;
        b = 0;
        return;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        x_sum += lines[i][0] + lines[i][2];
        y_sum += lines[i][1] + lines[i][3];
        m_sum += float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);
    }
    float x_avg = x_sum / (lines.size() * 2);
    float y_avg = y_sum / (lines.size() * 2);
    m = m_sum / lines.size();
    b = y_avg - m * x_avg;
    //ROS_ERROR("get_line_params, m :%.2f  b :%.2f", m, b);
}

float alcoholdriving::LineDetector::run()
{
    
    cv::Mat dst, th_img;
    float error=0;
       
    preprocessing(image, dst);
    binarization(dst, th_img);
    error = hough(th_img);
    ROS_ERROR("run :%.2f", error);

    return error;
}

bool alcoholdriving::LineDetector::check()
{
    //ROS_ERROR("check");
    return image.cols == 640;
}

void alcoholdriving::LineDetector::camCallback(const sensor_msgs::Image &msg){
    //ROS_ERROR("camCallback");
    cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar*>(&msg.data[0]), msg.step);
    cv::cvtColor(src, image, cv::COLOR_RGB2BGR);
}

void alcoholdriving::LineDetector::gaussianBlur(cv::Mat &img, cv::Mat &dst){
    int fsize = 3;
    int p = fsize -2;
    int h = img.rows;
    int w = img.cols;

    cv::Mat paded_img=cv::Mat::zeros(h+p*2, w+p*2, CV_8UC1)+255;
    img.copyTo(paded_img(cv::Rect(cv::Point(p,p), cv::Point(w+p,h+p))));
    dst=cv::Mat::zeros(h,w, CV_8UC1);

    for(int y=p; y<h+p; y++){
        for(int x=p; x<w+p; x++){
           double f1 = paded_img.at<uchar>(y-1,x-1)/16.0;
           double f2 = paded_img.at<uchar>(y-1,x)/8.0;
           double f3 = paded_img.at<uchar>(y-1,x+1)/16.0;
           double f4 = paded_img.at<uchar>(y,x-1)/8.0;
           double f5 = paded_img.at<uchar>(y,x)/4.0;
           double f6 = paded_img.at<uchar>(y,x+1)/8.0;
           double f7 = paded_img.at<uchar>(y+1,x-1)/16.0;
           double f8 = paded_img.at<uchar>(y+1,x+1)/8.0;
           double f9 = paded_img.at<uchar>(y+1,x+1)/16.0;
           dst.at<uchar>(y-p,x-p) = uchar((f1+f2+f3+f4+f5+f6+f7+f8+f9));
        }
    }
}
