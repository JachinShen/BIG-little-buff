#include "Headers.h"

static ros::Publisher led_param_pub;
static ros::Publisher sudoku_param_pub;
static std_msgs::Int16MultiArray led_param_msg;
static std_msgs::Int16MultiArray sudoku_param_msg;
int LED_RED_THRESHOLD;
int SUDOKU_GRAY_THRES;
int SUDOKU_AREA_MIN;
int SUDOKU_AREA_MAX;
int SUDOKU_HW_RATIO_MIN;
int SUDOKU_AREA_RATIO;

void ledRedThresOnChange(int pos)
{
    LED_RED_THRESHOLD = pos;
    led_param_msg.data.clear();
    led_param_msg.data.push_back(LED_RED_THRESHOLD);
    led_param_pub.publish(led_param_msg);
}

void sudokuGrayThresOnChange(int pos)
{
    SUDOKU_GRAY_THRES = pos;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(1);
    sudoku_param_msg.data.push_back(SUDOKU_GRAY_THRES);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuAreaMinThresOnChange(int pos)
{
    SUDOKU_AREA_MIN = pos;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(2);
    sudoku_param_msg.data.push_back(SUDOKU_AREA_MIN);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuAreaMaxThresOnChange(int pos)
{
    SUDOKU_AREA_MAX = pos;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(3);
    sudoku_param_msg.data.push_back(SUDOKU_AREA_MAX);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuHWRationMinThresOnChange(int pos)
{
    SUDOKU_HW_RATIO_MIN = pos;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(4);
    sudoku_param_msg.data.push_back(SUDOKU_HW_RATIO_MIN);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuAreaRatioThresOnChange(int pos)
{
    SUDOKU_AREA_RATIO = pos;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(5);
    sudoku_param_msg.data.push_back(SUDOKU_AREA_RATIO);
    sudoku_param_pub.publish(sudoku_param_msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ROS_INFO("Start!");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    led_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_param", 1);
    sudoku_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_param", 1);

    LED_RED_THRESHOLD = 80;
    SUDOKU_GRAY_THRES = 200;
    SUDOKU_AREA_MIN = 3000;
    SUDOKU_AREA_MAX = 10000;
    SUDOKU_HW_RATIO_MIN = 20; // 2.0
    SUDOKU_AREA_RATIO = 6;    // 0.6

    namedWindow("params");
    createTrackbar("led red threshold", "params", &LED_RED_THRESHOLD, 255, (cv::TrackbarCallback)ledRedThresOnChange);
    createTrackbar("sudoku gray thres", "params", &SUDOKU_GRAY_THRES, 255, (cv::TrackbarCallback)sudokuGrayThresOnChange);
    createTrackbar("sudoku area min", "params", &SUDOKU_AREA_MIN, 5000, (cv::TrackbarCallback)sudokuAreaMinThresOnChange);
    createTrackbar("sudoku area max", "params", &SUDOKU_AREA_MAX, 10000, (cv::TrackbarCallback)sudokuAreaMaxThresOnChange);
    createTrackbar("sudoku hw ratio min", "params", &SUDOKU_HW_RATIO_MIN, 100, (cv::TrackbarCallback)sudokuHWRationMinThresOnChange);
    createTrackbar("sudoku area ration", "params", &SUDOKU_AREA_RATIO, 10, (cv::TrackbarCallback)sudokuAreaRatioThresOnChange);

    while (ros::ok()) {
        waitKey(0);
    }

    return 0;
}
