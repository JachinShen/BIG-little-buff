#include "Headers.h"
#include "sudoku/BlockSplit.h"

static ros::Publisher led_param_pub;
static ros::Publisher sudoku_param_pub;
static std_msgs::Int16MultiArray led_param_msg;
static std_msgs::Int16MultiArray sudoku_param_msg;

static int LED_RED_THRESHOLD;
static int LED_GRAY_THRESHOLD;

static int SUDOKU_PARAM[BlockSplit::PARAM_SIZE] = {
    170,
    500,
    2000,
    30,
    100,
    70
};

string sudokuParamEnumToStr(int index)
{
    static string sudoku_param_enum_to_str[BlockSplit::PARAM_SIZE] = {
        "Sudoku Gray Threshold",
        "Sudoku Area Min",
        "Sudoku Area Max",
        "Sudoku HW Ratio Min",
        "Sudoku HW Ratio Max",
        "Sudoku Area Ratio"
    };
    return sudoku_param_enum_to_str[index];
}

int sudokuParamMax(int index)
{
    static int sudoku_param_max[BlockSplit::PARAM_SIZE] = {
        255,
        5000,
        10000,
        100,
        100,
        100
    };
    return sudoku_param_max[index];
}

void advertiseParam(int index, int value, ros::Publisher& pub, std_msgs::Int16MultiArray& msg)
{
    msg.data.clear();
    msg.data.push_back(index);
    msg.data.push_back(value);
    pub.publish(msg);
}

void advertiseLedParam(int index, int value)
{
    advertiseParam(index, value, led_param_pub, led_param_msg);
}

void ledRedThresOnChange(int pos)
{
    LED_RED_THRESHOLD = pos;
    advertiseLedParam(1, pos);
}

void ledGrayThresOnChange(int pos)
{
    LED_GRAY_THRESHOLD = pos;
    advertiseLedParam(2, pos);
}

void advertiseSudokuParam(int index, int value)
{
    //cout << "Index: " << index
    //<< "Value: " << value << endl;
    sudoku_param_msg.data.clear();
    sudoku_param_msg.data.push_back(index);
    sudoku_param_msg.data.push_back(value);
    sudoku_param_pub.publish(sudoku_param_msg);
}

void sudokuOnChange(int pos, void* id)
{
    int* value = (int*)id;
    *value = pos;
    advertiseSudokuParam(value - SUDOKU_PARAM, pos);
}

void updateAllParam()
{
    advertiseLedParam(1, LED_RED_THRESHOLD);
    advertiseLedParam(2, LED_GRAY_THRESHOLD);
    for (int i = 0; i < BlockSplit::PARAM_SIZE; ++i) {
        advertiseSudokuParam(i, SUDOKU_PARAM[i]);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "param");
    ROS_INFO("Start!");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    led_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_param", 1);
    sudoku_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_param", 1);

    LED_RED_THRESHOLD = 80;
    LED_GRAY_THRESHOLD = 80;

    namedWindow("params");
    createTrackbar("led red threshold", "params", &LED_RED_THRESHOLD, 255, (cv::TrackbarCallback)ledRedThresOnChange);
    createTrackbar("led gray threshold", "params", &LED_GRAY_THRESHOLD, 255, (cv::TrackbarCallback)ledGrayThresOnChange);

    for (int i = 0; i < BlockSplit::PARAM_SIZE; ++i) {
        createTrackbar(sudokuParamEnumToStr(i), "params",
            SUDOKU_PARAM + i, sudokuParamMax(i),
            (cv::TrackbarCallback)sudokuOnChange,
            SUDOKU_PARAM + i);
    }
    updateAllParam();

    while (ros::ok()) {
        if (waitKey(0) >= 0) {
            cout << "Update Param!" << endl;
            updateAllParam();
        }
    }

    return 0;
}
