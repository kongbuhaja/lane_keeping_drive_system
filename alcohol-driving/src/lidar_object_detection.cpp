#include "ros/ros.h"
// #include "lane/xycar_motor.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
#include "algorithm"
#include "std_msgs/Float32.h"

namespace team1
{
// #define USE_MOTOR

/* Start degree of Lidar(Left handed) */
#define LIDAR_START_DEGREE 350

/* End degree of Lidar(Right handed) */
#define LIDAR_END_DEGREE 110

/* Maxinum degree of Lidar (Mininmum is 0)*/
#define LIDAR_MAX_DEGREE 505

/* Required minimum number of dots for an object */
#define LIDAR_OBJECT_MIN_SIZE 20

/* Required minimum distance of meters of an object */
#define LIDAR_MINIMUM_DISTANCE 0.3f

/* Required maximum distance of meters of an object */
#define LIDAR_MAXIMUM_DISTANCE 1.f

#define STEERING_TOTAL_NUMBER 40
}

using namespace team1;

class ObstacleLidarDetector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    std::vector<float> lidar_points;
    float* result_array;

    ObstacleLidarDetector() {}

    /* Callback triggered if topic sent from Lidar */
    void lidar_callback(sensor_msgs::LaserScan data)
    {
        lidar_points = data.ranges;
    }
    bool is_lidar_on()
    {
        return (bool)lidar_points.size();
    }
    bool detect_object_lidar()
    {
        /*
            data.ranges
                index : angle
                element : distance
        */
        /* counting cloud points in ranges */
        int cnt = LIDAR_START_DEGREE - LIDAR_END_DEGREE;
        float x[cnt] = {
            0,
        };
        int number_of_adjacent = 0;

        int j;
        for (int i = 0; i < cnt; ++i)
        {
            j = (i + LIDAR_START_DEGREE) % LIDAR_MAX_DEGREE;
            if (lidar_points[j] > LIDAR_MINIMUM_DISTANCE && lidar_points[j] < LIDAR_MAXIMUM_DISTANCE)
            {
                x[j] = lidar_points[j];
            }
        }

        for (int i = 0; i < cnt; ++i)
        {

            if (x[i])
            {
                ++number_of_adjacent;
            }
            else
            {
                if (number_of_adjacent > LIDAR_OBJECT_MIN_SIZE)
                {
                    // ROS_WARN("lidar : [%d]", (start + i) % LIDAR_MAX_DEGREE);
                }

                number_of_adjacent = 0;
            }
        }


        //////////////////////////////////////////
        // 240 -> 42


        //////////////////////////////////////////

        return /* returns true if object exists, or else*/;
    }

public:
    ObstacleLidarDetector(const ros::NodeHandle nh_) : nh(nh_) {
        delete result_array;
        result_array = new float[STEERING_TOTAL_NUMBER];
    }
    ~ObstacleLidarDetector(){
        delete result_array;
    }
    void run()
    {
        if (is_lidar_on())
        {
            if (detect_object_lidar())
            {
            }
            else
            {
            }
        }
    }

    
    // TODO 
    float& getResultArray(){
        return *result_array;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_driver");
    ros::NodeHandle nh_;
    ObstacleLidarDetector xycar(nh_);
    ros::Rate rate(30);
    ROS_WARN("Start Lidar");
    while (ros::ok())
    {
        xycar.run();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}