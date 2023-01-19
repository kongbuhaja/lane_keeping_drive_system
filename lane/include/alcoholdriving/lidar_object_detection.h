#ifndef LIDAR_OBJECT_DETECTION_H_
#define LIDAR_OBJECT_DETECTION_H_
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <algorithm>

namespace alcoholdriving
{
    /* Start degree of Lidar(Left handed) */
    #define LIDAR_START_DEGREE 350

    /* End degree of Lidar(Right handed) */
    #define LIDAR_END_DEGREE 110

    /* Maxinum degree of Lidar (Mininmum is 0)*/
    #define LIDAR_MAX_DEGREE 505

    /* Required minimum number of dots for an object */
    #define LIDAR_OBJECT_MIN_SIZE 6

    /* Required minimum distance of meters of an object */
    #define LIDAR_MINIMUM_DISTANCE 0.3f

    /* Required maximum distance of meters of an object */
    #define LIDAR_MAXIMUM_DISTANCE 1.f

    #define STEERING_TOTAL_NUMBER 40
    class ObstacleLidarDetector
    {
    public:
        ObstacleLidarDetector(const ros::NodeHandle nh_);
        ~ObstacleLidarDetector();
        void run();
        float* getResultArray();

    private:
        ros::NodeHandle nh;
        ros::Subscriber lidar_sub;
        std::vector<float> lidar_points;
        float* result_array;

        void lidar_callback(sensor_msgs::LaserScan data);
        bool is_lidar_on();
        bool detect_object_lidar();
    };
}
#endif

