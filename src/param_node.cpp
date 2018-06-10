#include "Headers.h"

static ros::Publisher led_param_pub;
static ros::Publisher sudoku_param_pub;
static std_msgs::Int16MultiArray led_param_msg;
static std_msgs::Int16MultiArray sudoku_param_msg;

static int LED_RED_THRESHOLD;
static int SUDOKU_GRAY_THRES;
static int SUDOKU_AREA_MIN;
static int SUDOKU_AREA_MAX;
static int SUDOKU_HW_RATIO_MAX;
static int SUDOKU_AREA_RATIO;

void advertiseLedParam(int value)
{
    led_param_msg.data.clear();
    led_param_msg.data.push_back(value);
    led_param_pub.publish(led_param_msg);
}

void ledRedThresOnChange(int pos)
{
    LED_RED_THRESHOLD = pos;
    advertiseLedParam(pos);
}

void advertiseSudokuParam(int index, int value)
{
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(index);
    sudoku_param_msg.data.push_back(value);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuGrayThresOnChange(int pos)
{
    SUDOKU_GRAY_THRES = pos;
    advertiseSudokuParam(1, pos);
}

void sudokuAreaMinThresOnChange(int pos)
{
    SUDOKU_AREA_MIN = pos;
    advertiseSudokuParam(2, pos);
}

void sudokuAreaMaxThresOnChange(int pos)
{
    SUDOKU_AREA_MAX = pos;
    advertiseSudokuParam(3, pos);
}

void sudokuHWRatioMinThresOnChange(int pos)
{
    SUDOKU_HW_RATIO_MAX = pos;
    advertiseSudokuParam(4, pos);
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
    ros::init(argc, argv, "param");
    ROS_INFO("Start!");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    led_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_param", 1);
    sudoku_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_param", 1);

    LED_RED_THRESHOLD = 80;
    SUDOKU_GRAY_THRES = 123;
    SUDOKU_AREA_MIN = 1800;
    SUDOKU_AREA_MAX = 5000;
    SUDOKU_HW_RATIO_MAX = 20; // 2.0
    SUDOKU_AREA_RATIO = 6;    // 0.6

    namedWindow("params");
    createTrackbar("led red threshold", "params", &LED_RED_THRESHOLD, 255, (cv::TrackbarCallback)ledRedThresOnChange);
    createTrackbar("sudoku gray thres", "params", &SUDOKU_GRAY_THRES, 255, (cv::TrackbarCallback)sudokuGrayThresOnChange);
    createTrackbar("sudoku area min", "params", &SUDOKU_AREA_MIN, 5000, (cv::TrackbarCallback)sudokuAreaMinThresOnChange);
    createTrackbar("sudoku area max", "params", &SUDOKU_AREA_MAX, 10000, (cv::TrackbarCallback)sudokuAreaMaxThresOnChange);
    createTrackbar("sudoku hw ratio max", "params", &SUDOKU_HW_RATIO_MAX, 100, (cv::TrackbarCallback)sudokuHWRatioMinThresOnChange);
    createTrackbar("sudoku area ration", "params", &SUDOKU_AREA_RATIO, 10, (cv::TrackbarCallback)sudokuAreaRatioThresOnChange);

    advertiseLedParam(LED_RED_THRESHOLD);
    advertiseSudokuParam(1, SUDOKU_GRAY_THRES);
    advertiseSudokuParam(2, SUDOKU_AREA_MIN);
    advertiseSudokuParam(3, SUDOKU_AREA_MAX);
    advertiseSudokuParam(4, SUDOKU_HW_RATIO_MAX);
    advertiseSudokuParam(5, SUDOKU_AREA_RATIO);

    while (ros::ok()) {
        waitKey(0);
    }

    return 0;
}
