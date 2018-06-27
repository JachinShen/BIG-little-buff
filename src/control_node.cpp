#include "state_machine.h"

static ros::Publisher aim_num_pub;
static ros::Publisher led_id_pub;
static ros::Publisher mnist_ctr_pub;
static ros::Publisher fire_ctr_pub;
static ControlSM csm;

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Led:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setLed(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;

    static std_msgs::Int16MultiArray led_id_msg;
    led_id_msg.data.clear();
    led_id_msg.data.push_back(csm.getLedNow());
    led_id_pub.publish(led_id_msg);
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

void fireRectCallback(const std_msgs::Int16MultiArray& msg)
{
    //cout << "Fire:";
    for (uint i = 0; i < msg.data.size()/4; ++i) {
        //csm.setSudoku(i, msg.data[i]);
        //cout << " " << msg.data[i];
    }
    //cout << endl;
}

void tickCallback(const std_msgs::Bool& msg)
{
    static std_msgs::Bool fire_ctr_msg;
    static std_msgs::Bool mnist_ctr_msg;
    if (csm.tick(msg.data)) {
        fire_ctr_msg.data = true;
        mnist_ctr_msg.data = true;
        fire_ctr_pub.publish(fire_ctr_msg);
        mnist_ctr_pub.publish(mnist_ctr_msg);
    }
}

void csmTimerCallback(const ros::TimerEvent&)
{
    //csm.run();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::Timer csm_timer = nh.createTimer(ros::Duration(0.1), csmTimerCallback);

    ROS_INFO("Start!");
    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber mnist_num_sub
        = nh.subscribe("buff/mnist_num", 1, mnistNumCallback);
    ros::Subscriber fire_num_sub
        = nh.subscribe("buff/fire_num", 1, fireNumCallback);
    ros::Subscriber fire_rect_sub
        = nh.subscribe("buff/fire_rect", 1, fireRectCallback);
    ros::Subscriber tick_sub
        = nh.subscribe("buff/tick", 1, tickCallback);
    aim_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_rect", 1);
    fire_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/fire_ctr", 1);
    mnist_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/mnist_ctr", 1);
    led_id_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/led_id", 1);

    ros::spin();

    return 0;
}
