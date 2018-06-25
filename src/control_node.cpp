#include "state_machine.h"

ControlSM csm;

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO("Led:");
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setLed(i, msg.data[i]);
        ROS_INFO_STREAM(msg.data[i]);
    }
}

void mnistNumCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO("Mnist:");
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        ROS_INFO_STREAM(msg.data[i]);
    }
}

void fireNumCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO("Fire:");
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        ROS_INFO_STREAM(msg.data[i]);
    }
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber mnist_num_sub
        = nh.subscribe("buff/mnist_num", 1, mnistNumCallback);
    ros::Subscriber fire_num_sub
        = nh.subscribe("buff/fire_num", 1, fireNumCallback);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        csm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
