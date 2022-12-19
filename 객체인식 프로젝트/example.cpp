#include <ros/ros.h>
#include <xycar_msgs/xycar_motor.h>
#include <yolov3_trt_ros/BoundingBox.h>
#include <yolov3_trt_ros/BoundingBoxes.h>

// make your own callback function
void yoloCallback();

// left, right drive function
void drive_normal(std::string direction, float time, PID& pid);

// Global variable
int sleep_rate = 12;        // while loop will iterate "sleep_rate" times in a second (ros::Rate)

int main(int argc, char** argv) {

    ros::init(argc, argv, "yolov3_trt_ros");
    ros::Subscriber sub_yolo = nh.subscribe("/yolov3_trt_ros/detections", 1, yoloCallback);
    PID pid(kp, ki, kd);

    while (ros::ok()) {
        ros::spinOnce();
        if (frame.cols != 640) {
            continue;
        }

        // Start Drive
        // Left
        if (obj_id == 0) {
            drive_normal("left", 2.5, pid);
        }
        // Right
        else if (obj_id == 1) {
            drive_normal("right", 2.5, pid);
        }
        // Stop sign
        else if (obj_id == 2) {
            // make your own stop function
            drive_stop();
        }
        // Crosswalk sign
        else if (obj_id == 3) {
            // make your own cross walk function
            find_cross_walk();
        }
        // U-turn sign
        else if (obj_id == 4) {
            // make your own u-turn function
            find_u_turn();
        }
        // Traffic light
        else if (obj_id == 5) {
            // make your own traffic right function
            find_traffic_light();
        }
    }

    return 0;
}

void yoloCallback() {
    // make your own callback function
}

void drive_normal(std::string direction, float time, PID& pid) {
    ros::spinOnce();
    // Set the rate variable
    ros::Rate rate(sleep_rate);

    float max_cnt;
    int cnt = 0;

    if (time == 0) {        // time 0 means straight drive.
        max_cnt = 1;
    }
    else {
        // Set the maximun number of iteration in while loop.
        max_cnt = static_cast<float>(sleep_rate) * time;
    }

    while (static_cast<float>(cnt) < max_cnt) {
        std::vector<float> pos_and_slope = process_image();
        lpos = static_cast<int>(pos_and_slope[0]), rpos = static_cast<int>(pos_and_slope[1]);
        l_slope = pos_and_slope[2], r_slope = pos_and_slope[3];

        // Left or Right
        if (direction == "left") {
            rpos = lpos + 470;
        }
        else if (direction == "right") {
            lpos = rpos - 470;
        }

        mpos = (lpos + rpos) * 0.5;
        ma.add_sample(mpos);
        ma_pos = ma.get_wmm();

        cte = ma_pos - Width * 0.5;
        steer_angle = pid.pid_control(cte);

        steer_angle = std::max(-50.0f, std::min(steer_angle, 50.0f));
        drive(pub, steer_angle, speed);
        prev_angle = steer_angle;

        cnt++;
        rate.sleep();
    }
}