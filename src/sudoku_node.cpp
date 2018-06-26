#include "sudoku/BlockSplit.h"

static ros::Publisher led_rect_pub;
static ros::Publisher sudoku_rect_pub;
static BlockSplit     block_split;

void sudokuParamCallback(const std_msgs::Int16MultiArray& msg)
{
    block_split.setParam(msg.data[0], msg.data[1]);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static Mat img, gray, binary;
    Rect led_rect, sudoku_rect;

    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty())
        return;

    if (block_split.process(img, led_rect, sudoku_rect)) {
        ROS_INFO_STREAM("Led Rect: " << led_rect);
        std_msgs::Int16MultiArray led_rect_msg;
        led_rect_msg.data.push_back(led_rect.x);
        led_rect_msg.data.push_back(led_rect.y);
        led_rect_msg.data.push_back(led_rect.width);
        led_rect_msg.data.push_back(led_rect.height);
        led_rect_pub.publish(led_rect_msg);

        ROS_INFO_STREAM("Sudoku Rect: " << sudoku_rect);
        std_msgs::Int16MultiArray sudoku_rect_msg;
        sudoku_rect_msg.data.push_back(sudoku_rect.x);
        sudoku_rect_msg.data.push_back(sudoku_rect.y);
        sudoku_rect_msg.data.push_back(sudoku_rect.width);
        sudoku_rect_msg.data.push_back(sudoku_rect.height);
        sudoku_rect_pub.publish(sudoku_rect_msg);
    } else {
        ROS_INFO("No Sudoku Found!");
    }
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

    ROS_INFO("Start!");

    led_rect_pub    = nh.advertise<std_msgs::Int16MultiArray>("buff/led_rect",    1);
    sudoku_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_rect", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber sudoku_param_sub = nh.subscribe("buff/sudoku_param", 1, sudokuParamCallback);

    block_split.init();
    ros::spin();

    return 0;
}
