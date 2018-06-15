#include "Serial.h"

void serialCallback(const std_msgs::Int16MultiArray& msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "serial");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    ros::Subscriber led_num_sub
        = nh.subscribe("buff/serial", 1, serialCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
