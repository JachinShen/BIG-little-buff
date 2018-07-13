#include "sudoku/BlockSplit.h"

static     cv_bridge::CvImageConstPtr cv_ptr;
static ros::Publisher led_rect_pub;
static ros::Publisher sudoku_rect_pub;
static BlockSplit     block_split;
static bool           sudoku_run;

void sudokuParamCallback(const std_msgs::Int16MultiArray& msg)
{
    block_split.setParam(msg.data[0], msg.data[1]);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("Sudoku Image Call");
    if (!sudoku_run) {
        ROS_INFO("Ignore Sudoku!");
        return;
    }

    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    static Mat gray, binary;
    Rect led_rect, sudoku_rect;

    gray = cv_ptr->image.clone();
    if (gray.empty())
        return;

    if (block_split.processMnist(gray, led_rect, sudoku_rect)) {
        //ROS_INFO_STREAM("Led Rect: " << led_rect);
        std_msgs::Int16MultiArray led_rect_msg;
        led_rect_msg.data.push_back(led_rect.x);
        led_rect_msg.data.push_back(led_rect.y);
        led_rect_msg.data.push_back(led_rect.width);
        led_rect_msg.data.push_back(led_rect.height);
        led_rect_pub.publish(led_rect_msg);

        //ROS_INFO_STREAM("Sudoku Rect: " << sudoku_rect);
        std_msgs::Int16MultiArray sudoku_rect_msg;
        sudoku_rect_msg.data.push_back(sudoku_rect.x);
        sudoku_rect_msg.data.push_back(sudoku_rect.y);
        sudoku_rect_msg.data.push_back(sudoku_rect.width);
        sudoku_rect_msg.data.push_back(sudoku_rect.height);
        sudoku_rect_pub.publish(sudoku_rect_msg);

        sudoku_run = false;
    } else {
        ROS_INFO("No Sudoku Found!");
    }
}

void sudokuCtrCallback(const std_msgs::Bool& msg)
{
    sudoku_run = msg.data;
    ROS_INFO_STREAM("Get Sudoku Ctr: " << sudoku_run);
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sudoku");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.1), waitkeyTimerCallback);

    ROS_INFO("Sudoku Start!");

    led_rect_pub    = nh.advertise<std_msgs::Int16MultiArray>("buff/led_rect",    1);
    sudoku_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_rect", 1, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/gray", 1, imageCallback);
    ros::Subscriber sudoku_param_sub = nh.subscribe("buff/sudoku_param", 1, sudokuParamCallback);
    ros::Subscriber sudoku_ctr_sub = nh.subscribe("buff/sudoku_ctr", 1, sudokuCtrCallback);

    sudoku_run = false;
    block_split.init();
    ros::spin();

    return 0;
}
