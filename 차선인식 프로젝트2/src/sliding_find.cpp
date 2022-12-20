#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
//#include "lane/xycar_motor.h"


class Sliding_Window{
private:
    int width, height, nwindows, margin, minpix, lane_bin_th;
    cv::Mat mtx, dist, dist_matrix, perspect_m, perspect_m_inv;
    cv::Size size;
    // std::vector<cv::vec2f> warp_src, warp_dist;
    bool calibrated;
    ros::NodeHandle nh_;
    ros::Subscriber cam_sub;

public:
    Sliding_Window(const ros::NodeHandle &nh) : nh_(nh){
        width = 640;
        height = 480;

        nwindows = 9;
        margin = 12;
        minpix = 5;
        lane_bin_th = 145;
        calibrated = true;
        mtx = (cv::Mat_<double>(3,3) << 422.037858, 0.0, 245.895397,
                                        0.0, 435.589734, 163.625535,
                                        0.0, 0.0, 1.0);
        dist = (cv::Mat_<double>(1,5) << -0.289296, 0.061035, 0.001786, 0.015238, 0.0);
        size = cv::Size(width, height);
        dist_matrix = cv::getOptimalNewCameraMatrix(mtx, dist, size, 1, size);
        
    }

    void calibrate_image(cv::Mat &img, cv::Mat &src){
        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), dist_matrix, size, CV_32FC1, map1, map2);
        cv::remap(img, src, map1, map2, CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
    }
    
    void get_Perspect_mat(std::vector<cv::Point2f> src, std::vector<cv::Point2f> dst){
        perspect_m = cv::getPerspectiveTransform(src, dst);
        perspect_m_inv = cv::getPerspectiveTransform(dst, src);
    }
    
    void warp_image(cv::Mat &img, cv::Mat &dst, cv::Size size){
        cv::warpPerspective(img, dst, perspect_m, size);
    }

    void extract_bi_windows(cv::Mat img){
        cv::Mat blur, hls, lane;
        std::vector<cv::Mat> sp_hls;
        cv::GaussianBlur(img, blur, cv::Size(5,5), 0);
        cv::cvtColor(blur, hls, cv::COLOR_BGR2HLS);
        cv::split(hls,sp_hls);
        cv::threshold(sp_hls[1], lane, lane_bin_th, 255, cv::THRESH_BINARY);
        
        std::vector<int> histogram;
        int left_current = 0;
        int right_current = 0;
        
        for(int x=0; x<lane.cols; x++){
            int sum_=0;
            for(int y=lane.rows/2; y<lane.rows; y++){
                sum_ += lane.at<ucahr>(y,x);
            }
            histogram.push_back(sum_);
        }

        for(int i=0; i<lane.cols/2; i++)
            if(histogram[i] > histogram[left_current])
                left_current = i;
        for(int i=lane.cols/2; i<lane.cols; i++)
            if(histogram[i] > histogram[right_current])
                right_current = i;
        
        //std::cout<<histogram[0]<<std::endl;
        //int midpoint = (int)histogram.cols/2;

    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sliding_windows");
    ros::NodeHandle nh;
    cv::VideoCapture cap("/home/hs/xycar_ws_cpp/src/ocv/src/hough_track.avi");
    ros::Rate rate(30);
    cv::Mat frame, img, warp_img;
    int warp_w = 320;
    int warp_h = 240;
    int warpx_margin = 20;
    int warpy_margin = 3;
    std::vector<cv::Point2f> warp_src;
    warp_src.push_back(cv::Point2f(230-warpx_margin, 300-warpy_margin));
    warp_src.push_back(cv::Point2f(45-warpx_margin, 450+warpy_margin));
    warp_src.push_back(cv::Point2f(445+warpx_margin, 300-warpy_margin));
    warp_src.push_back(cv::Point2f(610+warpx_margin, 450+warpy_margin));
    std::vector<cv::Point2f> warp_dst;
    warp_dst.push_back(cv::Point2f(0,0));
    warp_dst.push_back(cv::Point2f(0,warp_h));
    warp_dst.push_back(cv::Point2f(warp_w, 0));
    warp_dst.push_back(cv::Point2f(warp_w,warp_h));

    Sliding_Window sw(nh);

    
    if(!cap.isOpened()){
        std::cout<<"no file"<<std::endl;
        return 0;
    }
    while(true)
    {
        cap >> frame;
        sw.calibrate_image(frame, img);
        sw.get_Perspect_mat(warp_src, warp_dst);
        sw.warp_image(frame, warp_img, cv::Size(warp_w, warp_h));
        sw.extract_bi_windows(warp_img);
        //cv::imshow("video",frame);
        //cv::imshow("distort", img);
        //cv::imshow("warp_img", warp_img);
        if(cv::waitKey(1) == 27){
            break;
        }
        rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}