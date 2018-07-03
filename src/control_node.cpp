#include "state_machine.h"
#include "OpenUart.h"

static ros::Publisher aim_num_pub;
static ros::Publisher sudoku_ctr_pub;
static ros::Publisher led_ctr_pub;
static ros::Publisher mnist_ctr_pub;
static ros::Publisher mnist_id_pub;
static ros::Publisher fire_ctr_pub;
static ControlSM csm;

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 5) {
        ROS_ERROR("Control Led Number");
        return;
    }
    csm.setLed(msg.data);

    ROS_INFO_STREAM("Led: " << msg.data[0] << msg.data[1]
            << msg.data[2] << msg.data[3] << msg.data[4]);
}

void mnistNumCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 20) {
        ROS_ERROR("Control Mnist");
        return;
    }
    csm.setSudoku(msg.data);
    ROS_INFO_STREAM("Mnist: " << msg.data[0] << msg.data[1]
            << msg.data[2] << msg.data[3] << msg.data[4]
            << msg.data[5] << msg.data[6] << msg.data[7]
            << msg.data[8] << msg.data[9]);
    ROS_INFO_STREAM("Mnist Possibility: " << msg.data[10] << msg.data[11]
            << msg.data[12] << msg.data[13] << msg.data[14]
            << msg.data[15] << msg.data[16] << msg.data[17]
            << msg.data[18] << msg.data[19]);
}

void fireNumCallback(const std_msgs::Int16MultiArray& msg)
{
    //cout << "Fire:";
    //for (uint i = 0; i < msg.data.size(); ++i) {
        //csm.setSudoku(i, msg.data[i]);
        //cout << " " << msg.data[i];
    //}
    //cout << endl;
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
    csm.tick(msg.data);
}

void sudokuRectCallback(const std_msgs::Int16MultiArray&)
{
    csm.setSudokuFound();
}

void csmTimerCallback(const ros::TimerEvent&)
{
    csm.run();
    csm.publishSudokuLedMnist(sudoku_ctr_pub, led_ctr_pub, mnist_ctr_pub);
    csm.publishMnist(mnist_id_pub);
    if (csm.getBlockIdNow() > 0) {
        if (csm.serial_send) {
            csm.serial_send = false;
            char block_id = csm.getBlockIdNow();
            ROS_INFO_STREAM("UART Write: " << (int)block_id);
            int n = ::write(fd, &block_id,1);
            if (n<0) {
                ROS_ERROR("UART Write");
            } else {
                char buffer[20];
                ::read(fd, buffer, 20);
                ROS_INFO_STREAM("Get UART:" << buffer);
            }
        }
    }
    //static std_msgs::Bool sudoku_ctr_msg;
    //static std_msgs::Bool led_ctr_msg;
    //static std_msgs::Bool mnist_ctr_msg;
    //static std_msgs::Bool fire_ctr_msg;
    //if (csm.getPublishRun()) {
        //sudoku_ctr_msg.data = csm.getSudokuRun();
        //sudoku_ctr_pub.publish(sudoku_ctr_msg);
        //led_ctr_msg.data = csm.getLedRun();
        //led_ctr_pub.publish(led_ctr_msg);
        //mnist_ctr_msg.data = csm.getMnistRun();
        //mnist_ctr_pub.publish(mnist_ctr_msg);
    //}
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::Timer csm_timer = nh.createTimer(ros::Duration(0.01), csmTimerCallback);

    ROS_INFO("Control Start!");
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
    ros::Subscriber sudoku_rect_sub
        = nh.subscribe("buff/sudoku_rect", 1, sudokuRectCallback);
    aim_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_rect", 1);
    sudoku_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/sudoku_ctr", 1);
    led_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/led_ctr", 1);
    mnist_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/mnist_ctr", 1);
    mnist_id_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_id", 1);
    fire_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/fire_ctr", 1);

    csm.init();
    ros::spin();

    return 0;
}
