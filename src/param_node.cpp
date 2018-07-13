#include "Headers.h"
#include "sudoku/BlockSplit.h"
#include "sudoku/LedSolver.h"

static ros::Publisher            led_param_pub;
static ros::Publisher            sudoku_param_pub;
static ros::Publisher            aim_param_pub;
static std_msgs::Int16MultiArray led_param_msg;
static std_msgs::Int16MultiArray sudoku_param_msg;
static std_msgs::Int16MultiArray aim_param_msg;

static int SUDOKU_PARAM[BlockSplit::PARAM_SIZE] = {
    35,
    300,
    1000,
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

static int LED_PARAM[LedSolver::PARAM_SIZE] = {
    30,
    100,
    40,
    1000,
    130,
    1400,
    250,
    5
};

string ledParamEnumToStr(int index)
{
    static string led_param_enum_to_str[LedSolver::PARAM_SIZE] = {
        "led red threshold",
        "led gray threshold",
        "led bound area min",
        "led bound area max",
        "led hw ratio Min",
        "led hw ratio Max",
        "led hw ratio for digit one",
        "led rotation degree"
    };
    return led_param_enum_to_str[index];
}

int ledParamMax(int index)
{
    static int led_param_max[LedSolver::PARAM_SIZE] = {
        255,
        255,
        1000,
        3000,
        250,
        2000,
        500,
        10
    };
    return led_param_max[index];
}

static int AIM_PARAM[BlockSplit::PARAM_SIZE] = {
    140,
    700,
    2000,
    30,
    100,
    70
};

string aimParamEnumToStr(int index)
{
    static string aim_param_enum_to_str[BlockSplit::PARAM_SIZE] = {
        "Aim Gray Threshold",
        "Aim Area Min",
        "Aim Area Max",
        "Aim HW Ratio Min",
        "Aim HW Ratio Max",
        "Aim Area Ratio"
    };
    return aim_param_enum_to_str[index];
}

int aimParamMax(int index)
{
    static int aim_param_max[BlockSplit::PARAM_SIZE] = {
        255,
        5000,
        10000,
        100,
        100,
        100
    };
    return aim_param_max[index];
}

void advertiseParam(int index, int value, ros::Publisher& pub, std_msgs::Int16MultiArray& msg)
{
    msg.data.clear();
    msg.data.push_back(index);
    msg.data.push_back(value);
    pub.publish(msg);
}

void sudokuOnChange(int pos, void* id)
{
    int* value = (int*)id;
    *value = pos;
    advertiseParam(value - SUDOKU_PARAM, pos, sudoku_param_pub, sudoku_param_msg);
}

void ledOnChange(int pos, void* id)
{
    int* value = (int*)id;
    *value = pos;
    advertiseParam(value - LED_PARAM, pos, led_param_pub, led_param_msg);
}

void aimOnChange(int pos, void* id)
{
    int* value = (int*)id;
    *value = pos;
    advertiseParam(value - AIM_PARAM, pos, aim_param_pub, aim_param_msg);
}

void updateAllParam()
{
    static int sudoku_publish_id = 0;
    static int led_publish_id    = 0;
    static int aim_publish_id    = 0;

    advertiseParam(sudoku_publish_id, SUDOKU_PARAM[sudoku_publish_id], sudoku_param_pub, sudoku_param_msg);
    advertiseParam(led_publish_id, LED_PARAM[led_publish_id], led_param_pub, led_param_msg);
    advertiseParam(aim_publish_id, AIM_PARAM[aim_publish_id], aim_param_pub, aim_param_msg);

    sudoku_publish_id = (++sudoku_publish_id) % BlockSplit::PARAM_SIZE;
    led_publish_id    = (++led_publish_id)    % LedSolver::PARAM_SIZE;
    aim_publish_id    = (++aim_publish_id)    % BlockSplit::PARAM_SIZE;
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

void updateTimerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Update All Param");
    updateAllParam();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "param");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.05), waitkeyTimerCallback);
    ros::Timer update_timer  = nh.createTimer(ros::Duration(0.3),  updateTimerCallback);

    ROS_INFO("Start!");
    led_param_pub    = nh.advertise<std_msgs::Int16MultiArray>("buff/led_param",    1);
    sudoku_param_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_param", 1);
    aim_param_pub    = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_param",    1);

    namedWindow("params");

    for (int i = 0; i < BlockSplit::PARAM_SIZE; ++i) {
        createTrackbar(sudokuParamEnumToStr(i), "params",
            SUDOKU_PARAM + i, sudokuParamMax(i),
            (cv::TrackbarCallback)sudokuOnChange,
            SUDOKU_PARAM + i);
    }

    for (int i = 0; i < LedSolver::PARAM_SIZE; ++i) {
        createTrackbar(ledParamEnumToStr(i), "params",
            LED_PARAM + i, ledParamMax(i),
            (cv::TrackbarCallback)ledOnChange,
            LED_PARAM + i);
    }

    for (int i = 0; i < BlockSplit::PARAM_SIZE; ++i) {
        createTrackbar(aimParamEnumToStr(i), "params",
            AIM_PARAM + i, aimParamMax(i),
            (cv::TrackbarCallback)aimOnChange,
            AIM_PARAM + i);
    }
    for (int i=0; i < BlockSplit::PARAM_SIZE || i < LedSolver::PARAM_SIZE; ++i) {
        updateAllParam();
    }

    ros::spin();

    return 0;
}
