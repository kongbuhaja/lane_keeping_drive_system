#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "iostream"
#include "random"
#include "yaml-cpp/yaml.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "alcoholdriving/motor.h"
#include "alcoholdriving/vision.h"
#include "alcoholdriving/pid_controller.h"

#define MIN_ERROR_IGNORABLE -1
#define MAX_ERROR_IGNORABLE 1
#define MAIN_DEBUG true
class Main
{
public:
    Main(int argc, char **argv)
        : argc_(argc), argv_(argv)
    {
        
        nh_ = ros::NodeHandle();


        std::string config_path;
        nh_.getParam("config_path", config_path);
        const YAML::Node &config = YAML::LoadFile(config_path);
        xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
        xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
        xycar_speed_control_threshold_ = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();     
        acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
        deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
        motor_ptr_ = new alcoholdriving::Motor(nh_, config["XYCAR"]["START_SPEED"].as<float>());
        pid_ptr_ = new alcoholdriving::PID(config["PID"]["P_GAIN"].as<float>(),
                                           config["PID"]["I_GAIN"].as<float>(),
                                           config["PID"]["D_GAIN"].as<float>());
        line_detector_ptr_ = new alcoholdriving::LineDetector(nh_);
    }
    ~Main()
    {
        // TODO : Destroy
        delete motor_ptr_;
        delete pid_ptr_;
        delete line_detector_ptr_;
    }

    int run()
    {

        while (ros::ok())
        {
            //ROS_ERROR("1");
            ros::spinOnce();
            // Vision proccessing
            if(!line_detector_ptr_ -> check())
                continue;
            error_ = line_detector_ptr_-> run();
            //ROS_ERROR("2");
            // TODO : LiDAR

            // TOBE : IMU

            // float angle = std::max(-(float)kXycarSteeringAngleLimit,
            //                        std::min(pid_ptr_->getControlOutput(error, ~(error_ > MIN_ERROR_IGNORABLE && error_ < MAX_ERROR_IGNORABLE)),
            //                                 (float)kXycarSteeringAngleLimit));
            float angle = std::max(-(float)kXycarSteeringAngleLimit,
                                            std::min(pid_ptr_->getControlOutput(error_)*(float)kXycarSteeringAngleLimit*20,
                                            (float)kXycarSteeringAngleLimit));
            
            //ROS_ERROR("3");
            
            motor_ptr_->set_motor_control(
                /* angle */ angle,
                /* speed */ speed_control(angle));

            // Motor and steering control
            //ROS_ERROR("4");
            motor_ptr_->motor_publish();

            
        }
        //ROS_ERROR("5");
        return 0;
    }

private:
    static const int kXycarSteeringAngleLimit = 50;

    int argc_;
    char **argv_;
    bool debug_;
    float error_;     

    // Xycar Device variables
    float xycar_max_speed_;
    float xycar_min_speed_;
    float xycar_speed_control_threshold_;
    float acceleration_step_;
    float deceleration_step_;
    float xycar_initial_speed_;

    ros::NodeHandle nh_;
    // std::unique_ptr<alcoholdriving::Motor> motor_ptr_;
    // std::unique_ptr<alcoholdriving::PID> pid_ptr_;
    // std::unique_ptr<alcoholdriving::LineDetector> line_detector_ptr_;

    alcoholdriving::Motor* motor_ptr_;
    alcoholdriving::PID* pid_ptr_;
    alcoholdriving::LineDetector* line_detector_ptr_;


    inline float speed_control(float angle)
    {
        // decelerate if the angle is wide enough,
        // else accelerate.
        ROS_ERROR("angle: %f", angle);
        ROS_ERROR("speed: %f", motor_ptr_->getSpeed());
        return float(std::abs(angle) > xycar_speed_control_threshold_ ? 
        std::max(motor_ptr_->getSpeed() - deceleration_step_, xycar_min_speed_) : 
        std::min(motor_ptr_->getSpeed() + acceleration_step_, xycar_max_speed_));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "team1/main");
    return Main(argc, argv).run();
}
