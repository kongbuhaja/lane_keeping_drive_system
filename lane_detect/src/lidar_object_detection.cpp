#include "alcoholdriving/lidar_object_detection.h"



namespace alcoholdriving
{
    
    ObstacleLidarDetector::ObstacleLidarDetector(const ros::NodeHandle nh_) : nh(nh_)
    {
        result_array = new float[STEERING_TOTAL_NUMBER];
        lidar_sub = nh.subscribe("/scan", 1, &ObstacleLidarDetector::lidar_callback, this);
    }

    ObstacleLidarDetector::~ObstacleLidarDetector()
    {
        delete result_array;
    }

    void ObstacleLidarDetector::run()
    {
        if (is_lidar_on())
        {
            if (detect_object_lidar())
            {
                    /* ToDo */
            }
            else
            {
            }
        }
    }
    // TODO
    float* ObstacleLidarDetector::getResultArray()
    {
        return result_array;
    }
    /* Callback triggered if topic sent from Lidar */
    void ObstacleLidarDetector::lidar_callback(sensor_msgs::LaserScan data)
    {
        lidar_points = data.ranges;
    }
    bool ObstacleLidarDetector::is_lidar_on()
    {
        return lidar_points.size() != 0;
    }
    bool ObstacleLidarDetector::detect_object_lidar()
    {
        /*
            data.ranges
            index : angle
            element : distance
        */
        /* counting cloud points in ranges */
        int cnt = LIDAR_START_DEGREE - LIDAR_END_DEGREE;
        float* x = new float[cnt]{
            0,
        };
        int number_of_adjacent = 0;
        int j;

        /* [!] Source code verification required [!] */
        for (int i = 0; i < cnt; i++)
        {
            j = (i + LIDAR_START_DEGREE) % LIDAR_MAX_DEGREE;
            if ((lidar_points[j] > LIDAR_MINIMUM_DISTANCE) && (lidar_points[j] < LIDAR_MAXIMUM_DISTANCE))
            {
                x[i] = lidar_points[j];
            }
        }

        float distance_avg;
        for (int i = 0, k = 0; (i+(LIDAR_OBJECT_MIN_SIZE-1)) < cnt; i += LIDAR_OBJECT_MIN_SIZE, k++)
        {
            if (x[i] && x[i+1] && x[i+2] && x[i+3] && x[i+4] && x[i+5])
            {
                distance_avg = x[i];
                distance_avg += x[i+1];
                distance_avg += x[i+2];
                distance_avg += x[i+3];
                distance_avg += x[i+4];
                distance_avg += x[i+5];
                distance_avg = distance_avg / (float)LIDAR_OBJECT_MIN_SIZE;

                result_array[k] = distance_avg;
                ++number_of_adjacent;
		        //ROS_ERROR("result_array[%d]=[%f]", k, result_array[k]);
            }
            else
            {
                result_array[k] = 0.0f;
            }
        }
        return number_of_adjacent != 0;
    }
}  // namespace alcoholdriving

