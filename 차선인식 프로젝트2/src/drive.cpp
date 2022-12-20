#include "ros/ros.h"
#include "lane/xycar_motor.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
#include "algorithm"

namespace team1
{
/* Start degree of Lidar(Left handed) */
#define LIDAR_START_DEGREE 350

/* End degree of Lidar(Right handed) */
#define LIDAR_END_DEGREE 110

/* Maxinum degree of Lidar (Mininmum is 0)*/
#define LIDAR_MAX_DEGREE 505

/* Required minimum number of dots for an object */
#define LIDAR_MIN_SIZE 20

}
class CAR
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher motor_pub;
    ros::Subscriber motor_sub;
    lane::xycar_motor msg_motor;
    std::vector<float> lidar_points;

    void lidar_callback(sensor_msgs::LaserScan data)
    {
        lidar_points = data.ranges;
        // int s = sizeof(data.range)/sizeof(data.range[0])
        // std::copy(data.range, data.range+s, lidar_points)
    }

    void motor_callback(const lane::xycar_motor &msg)
    {
        motor_publish(msg);
    }

    void motor_publish()
    {
        motor_pub.publish(msg_motor);
    }

    bool is_lidar_on()
    {
        return ~lidar_points.size();
        // TODO remove
        // if(lidar_points.size()==0){return false;}
        // return true;
    }

    void set_motor_control(const float angle, const float speed)
    {
        msg_motor.angle = angle;
        msg_motor.speed = speed;
    }

    bool detect_object_lidar(float threshold = 0.3)
    {
        return false;
        int start = team1::LIDAR_START_DEGREE;
        int end = team1::LIDAR_END_DEGREE;
        int length = lidar_points.size();
        int front_size = length - start + end + 2;
        std::vector<float> front;
        int count = 20;
        asdasd

            for (int i = start; i < lidar_points.size(); i++)
        {
            front.push_back(lidar_points[i]);
        }
        for (int i = 0; i < end; i++)
        {
            front.push_back(lidar_points[i]);
        }

        for (int i = 0; i < front.size(); i++)
        {
            if (front[i] <= threshold)
            {
                count -= 1;
                if (count <= 0)
                {
                    std::cout << ((start + i) % team1::LIDAR_MAX_DEGREE) << std::endl;
                    return true;
                }
            }
            else
            {
                count = 20;
            }
        }
        return false;
    }

public:
    CAR(const ros::NodeHandle nh_) : nh(nh_)
    {
        lidar_sub = nh.subscribe("/scan", 1, &CAR::lidar_callback, this);
        motor_pub = nh.advertise<lane::xycar_motor>("xycar_motor", 1);
        motor_sub = nh.subscribe("go", &CAR::motor_callback, this);
    }

    void run()
    {
        while (!is_lidar_on())
        {
            ros::spinOnce();
        }
        if (detect_object_lidar())
        {
            set_motor_control(0, 0);
            motor_publish();
        }
        else
        {
            set_motor_control(40, 3);
            motor_publish();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_driver");
    ros::NodeHandle nh_;
    CAR xycar(nh_);
    while (ros::ok())
    {
        xycar.run();
        ros::spinOnce();
    }
    return 0;
}