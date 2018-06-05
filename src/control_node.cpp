#include "state_machine.h"

ControlSM csm;

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Led: " << endl;
    for (uint i=0; i<msg.data.size(); ++i) {
        csm.setLed(i, msg.data[i]);
        cout << msg.data[i] << endl;
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);

    ros::Rate loop_rate(100);

    while(ros::ok()) {
        csm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Finish!");
    return 0;
}
