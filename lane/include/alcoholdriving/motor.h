#ifndef MOTOR_H_
#define MOTOR_H_

#include "ros/ros.h"
#include "lane/xycar_motor.h"

namespace alcoholdriving
{
    class Motor
    {
    public:
        Motor(ros::NodeHandle &nh, const float initial_speed) : nh_(nh)
        {
            motor_pub_ = nh_.advertise<lane::xycar_motor>("xycar_motor", 1);
            msg_motor_.speed = initial_speed;
        }

        ~Motor() {}

        inline void motor_publish()
        {
            motor_pub_.publish(msg_motor_);
        }
        inline void set_motor_control(const float angle, const float speed)
        {
            //ROS_ERROR("angle : %.2f speed : %.2f", angle, speed);
            msg_motor_.angle = angle;
            msg_motor_.speed = speed;
        }
        inline float getAngle()
        {
            return msg_motor_.angle;
        }
        inline float getSpeed()
        {
            return msg_motor_.speed;
        }

    private:
        ros::NodeHandle &nh_;
        ros::Publisher motor_pub_;
        ros::Subscriber motor_sub_;
        lane::xycar_motor msg_motor_;
    };
}
#endif