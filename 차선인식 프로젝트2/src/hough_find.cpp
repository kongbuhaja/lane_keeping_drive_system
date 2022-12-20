#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "lane/xycar_motor.h"
#include <nmeric>

#define PI M_PI

class MovingAverage{
private:
    int samples;
    std::vector<float> data, weights;
public:    
    MovingAverage(const int n){
        samples = n;
        for(int i=1; i<n+1; i++){
            weights.push_back(i);
        }
    }
    void add_sample(const float new_sample){
        if(data.size() == samples)
            data.erase(data.begin());
        data.push_back(new_sample);
    }
    float get_mm(){
        return accumulate(data.begin(), data.end(), float(0));
    }

    float get_wmm(){
        float s = 0;
        for(int i=0; i<data.size(); i++)
            s += data[i] * weights[i];
        return float(s) / accumulate(weights.begin(), weights.begin() + data.size(), float(0))
    }
};

class PID{
private:
    float Kp, Ki, Kd, p_error, i_error, d_error;
public:
    PID(const float kp, const float ki, const float kd){
        Kp=kp;
        Ki=ki;
        Kd=kd;
        p_error=0.0;
        i_error=0.0;
        d_error=0.0;
    }
    float pid_control(const float cte){
        d_error = cte - p_error;
        p_error = cte;
        i_error += cte;
        return Kp*p_error + Ki*i_error + Kd*d_error;
    }
};
class Line_Detector{
private:
    int width, height, offset, gap;
    ros::NodeHandle nh;
    ros::Subscriber cam_sub;
    ros::Publisher drive_pub;
    lane::xycar_motor msg_motor;
    cv::Mat image;
    PID pid(0.5, 5e-4, 0.5*0.1);
    MovingAverage mml(50);
public:
    Line_Detector(const ros::NodeHandle &nh_): nh(nh_){
        width = 640;
        height = 480;
        offset = 370;
        gap = 40;
        cam_sub = nh.subscribe("/usb_cam/image_raw", 1, &Line_Detector::cam_callback, this);
        drive_pub = nh.advertise<lane::xycar_motor.h>("go", 1);
    }

    void drive(float angle, float speed){
        msg_motor.angle = angle;
        msg_motor.speed = speed;
        drive_pub.publish(msg_motor);
    }
    void cam_callback(const sensor_msgs::Image &msg){
        cv::Mat img = cv::Mat(480, 640, CV_8UC3, const_cast<uchar *>(&msg.data[0]), msg.step);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        start(img);
    }

    void draw_lines(cv::Mat &img, std::vector<cv::Vec4i> &lines){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> ds(0, 255);
        
        for(int i=0; i<lines.size(); i++){
            cv::line(img, cv::Point(lines[i][0], lines[i][1]+offset), cv::Point(lines[i][2], lines[i][3]+offset), (ds(gen),ds(gen),ds(gen)), 2);
        }
    }
    
    void draw_rectangle(cv::Mat &img, int lpos, int rpos){
        int center = (lpos + rpos) / 2;
        cv::rectangle(img, cv::Rect(lpos - 5, 15 + offset, 10, 10), (0, 255, 0), 2);
        cv::rectangle(img, cv::Rect(rpos - 5, 15 + offset, 10, 10), (0, 255, 0), 2);
        cv::rectangle(img, cv::Rect(center - 5, 15 + offset, 10, 10), (0, 255, 0), 2);
        cv::rectangle(img, cv::Rect(315, 15 + offset, 10, 10), (0, 0, 255), 2);
    }

    void divide_left_right(std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines, std::vector<cv::Vec4i> &right_lines){
        int threshold = 10;
        std::vector<float> slopes;
        std::vector<cv::Vec4i> new_lines;
        float slope;

        for(int i=0; i<lines.size(); i++){
            if((lines[i][2] - lines[i][0])==0) {slope=0;}
            else {slope = float(lines[i][3]-lines[i][1]) / float(lines[i][2]-lines[i][0]);} 
            if((slope > -threshold) && (slope < threshold)){
                slopes.push_back(slope);
                new_lines.push_back(lines[i]);
            }
        }
        for(int j=0; j<slopes.size(); j++){
            if((slopes[j] < 0) && (new_lines[j][2] < width/2 -90)) {left_lines.push_back(new_lines[j]);}
            else if((slopes[j] > 0) && (new_lines[j][0] > width/2 +90)) {right_lines.push_back(new_lines[j]);}
      
        }
    }

    void get_line_params(std::vector<cv::Vec4i> &lines, float &m, float &b){
        float x_sum = 0.0;
        float y_sum = 0.0;
        float m_sum = 0.0;

        if(lines.size()==0){
            m=0; b=0;
            return;
        }

        for(int i=0; i<lines.size(); i++){
            x_sum += lines[i][0] + lines[i][2];
            y_sum += lines[i][1] + lines[i][3];
            m_sum += float(lines[i][3] - lines[i][1]) / float(lines[i][2] - lines[i][0]);
        }
        float x_avg = x_sum / (lines.size() * 2);
        float y_avg = y_sum / (lines.size() * 2);
        m = m_sum / lines.size();
        b = y_avg - m * x_avg;
    }

    int get_line_pos(cv::Mat &img, std::vector<cv::Vec4i> &lines, bool left=false, bool right=false){
        float m, b;
        get_line_params(lines, m, b);
        int pos;
        if(m==0 && b==0){
            if(left) pos = 0;
            if(right) pos = width; 
        }
        else{
            int y = gap/2;
            pos = (y-b) / m;
            b += offset;
            float x1 = (height - b) / float(m);
            float x2 = (height/2 - b) / float(m);
            cv::line(img, cv::Point(int(x1), height), cv::Point(int(x2), height/2), (255, 0, 0), 3);
        }
        return pos;
    }

    void process_image(cv::Mat &img, int &lpos, int &rpos){
        cv::Mat gray, blur_gray, edge_img, roi;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Vec4i> lines, left_lines, right_lines;
        int kernel_size = 5;
        cv::GaussianBlur(img, blur_gray, cv::Size(kernel_size, kernel_size), 0);
        
        int low_threshold=60;
        int high_threshold=70;

        cv::Canny(blur_gray, edge_img, low_threshold, high_threshold);
    
        roi = edge_img(cv::Rect(0, offset, width, gap));
        cv::imshow("roi",roi);
        cv::HoughLinesP(roi, lines, 1, PI/180, 30, 30, 10);
        
        if(lines.size()==0){
            lpos=0; rpos=640;
            return;
        }
        divide_left_right(lines, left_lines, right_lines);
    
        lpos = get_line_pos(img, left_lines, true, false);
        rpos = get_line_pos(img, right_lines, false, true);
        
        draw_lines(img, left_lines);
        draw_lines(img, right_lines);
        cv::line(img, cv::Point(230, 245), cv::Point(410, 235), (255,255,255), 2);
        draw_rectangle(img, lpos, rpos);
    }

    void start(cv::Mat &img){
        int lpos, rpos;
        process_image(img, lpos, rpos);
        int center = (lpos + rpos)/2;
        int error = 320 - center;
        float angle = pid.pid_control(error);
        mml.add_sample(angle);
        wwm_angle = mml.get_wmm()
        drive(angle, 2);       
        cv::imshow("output", img);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "cv_bridege_test");
    ros::NodeHandle nh_;
    std::string video_path;
    ros::param::get("~video_path", video_path);
    // cv::VideoCapture cap("/home/hs/xycar_ws_cpp/src/ocv/src/hough_track.avi");
    // cv::Mat frame;
    ros::Rate rate(30);
    //Line_Detector line_detector(nh_);
    std_msgs::String s;
    while(ros::ok()){
        if(cv::waitKey(1) == 27){
           break;
        }
        //ros::spinOnce();
    }
    cv::destroyAllWindows();
}