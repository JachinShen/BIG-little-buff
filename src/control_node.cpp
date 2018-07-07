#include "state_machine.h"
#include "Serial.h"

static ros::Publisher aim_num_pub;
static ros::Publisher sudoku_ctr_pub;
static ros::Publisher led_ctr_pub;
static ros::Publisher mnist_ctr_pub;
static ros::Publisher aim_ready_pub;
static ros::Publisher fire_ctr_pub;
static ControlSM csm;
static Serial serial;

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
    if (msg.data.size() != 20) {
        ROS_ERROR("Control Mnist");
        return;
    }
    csm.setSudoku(msg.data);
    ROS_INFO_STREAM("Fire: " << msg.data[0] << msg.data[1]
            << msg.data[2] << msg.data[3] << msg.data[4]
            << msg.data[5] << msg.data[6] << msg.data[7]
            << msg.data[8] << msg.data[9]);
    ROS_INFO_STREAM("Fire Possibility: " << msg.data[10] << msg.data[11]
            << msg.data[12] << msg.data[13] << msg.data[14]
            << msg.data[15] << msg.data[16] << msg.data[17]
            << msg.data[18] << msg.data[19]);
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

void aimPosCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 3) {
        ROS_ERROR("Control Aim Pos");
        return;
    }
    int target_x = msg.data[0], target_y = msg.data[1];
    int is_found = msg.data[2];
    static int slow_ctr=0;
    if (is_found == -1) {
        ROS_INFO("Demarcate Complete");
        //serial.sendTarget(320, 240, 3);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        serial.sendTarget(target_x, target_y, 14);
        csm.setDemarcateComplete();
    } else {
        serial.sendTarget(target_x, target_y, 10 + is_found);
        //if (is_found == 4) {
            //serial.sendTarget(320, 240, 3);
            //serial.sendTarget(target_x, target_y, is_found);
            //return;
        //}
        ////if (slow_ctr > 10) {
            ////slow_ctr = 0;    
            //serial.sendTarget(target_x, target_y, is_found);
        ////} else {
            ////++slow_ctr;
        ////}
    }
}

void csmTimerCallback(const ros::TimerEvent&)
{
    csm.run();
    csm.publishSudokuLedMnistFire(sudoku_ctr_pub, led_ctr_pub, mnist_ctr_pub, fire_ctr_pub);
    char block_id;
    block_id = csm.sendBlockID();
    if (block_id > 0) {
        ROS_INFO_STREAM("Send Block Id: " << (int)block_id);
        serial.sendString(&block_id, 1);
    } else if (block_id == 0) {
        ROS_INFO("Demarcate");
        //serial.receive();
        //if (serial.buffMode() == 1 
                //|| serial.buffMode() == 2) {
            ROS_INFO("Enter Buff Mode!");
            static std_msgs::Bool aim_ready_msg;
            aim_ready_msg.data = true;
            aim_ready_pub.publish(aim_ready_msg);
        //} else {
            //ROS_INFO("Wait for STM");
        //}
    }
    //csm.publishMnist(mnist_id_pub);
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
    ros::Subscriber aim_pos_sub
        = nh.subscribe("buff/aim_pos", 100, aimPosCallback);
    aim_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_rect", 1);
    sudoku_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/sudoku_ctr", 1);
    led_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/led_ctr", 1);
    mnist_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/mnist_ctr", 1);
    aim_ready_pub
        = nh.advertise<std_msgs::Bool>("buff/aim_ready", 1);
    fire_ctr_pub
        = nh.advertise<std_msgs::Bool>("buff/fire_ctr", 1);

    if (argv[1] == NULL) {
        ROS_ERROR("Control argv[1] == NULL");
        serial.init("/dev/ttyUSB0");
    } else {
        serial.init(argv[1]);
    }
    csm.init();
    ros::spin();

    return 0;
}
