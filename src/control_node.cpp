#include "state_machine.h"

ControlSM csm;

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Led:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setLed(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void mnistNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Mnist:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void fireNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Fire:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void csmTimerCallback(const ros::TimerEvent&)
{
    csm.run();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ROS_INFO("Start!");
    ros::NodeHandle nh;
    ros::Timer csm_timer = nh.createTimer(ros::Duration(0.1), csmTimerCallback);

    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber mnist_num_sub
        = nh.subscribe("buff/mnist_num", 1, mnistNumCallback);
    ros::Subscriber fire_num_sub
        = nh.subscribe("buff/fire_num", 1, fireNumCallback);

    ros::spin();

    return 0;
}
