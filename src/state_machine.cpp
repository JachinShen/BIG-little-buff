#include "state_machine.h"

ControlSM::ControlSM()
{
}

ControlSM::~ControlSM()
{
}

void ControlSM::transferState(State s)
{
    state = s;
    ROS_INFO_STREAM("Transfer to state: " << state_to_str[s]);
    publish_run = true;
    freshCtr();
}

void ControlSM::init()
{
    state            = WAIT;
    publish_run      = true;
    led_remain       = false;
    rising_edge      = false;
    falling_edge     = false;
    sudoku_found     = false;
    mnist_id_publish = false;
    freshCtr();

    for (int i = 0; i < 5; ++i)
        led[i] = -1;

    for (int i = 0; i < 5; ++i)
        sudoku[i] = -1;

    state_to_str[WAIT]       = "WAIT";
    state_to_str[READY]      = "READY";
    state_to_str[LED_ONE]    = "LED_ONE";
    state_to_str[ONE_TWO]    = "ONE_TWO";
    state_to_str[LED_TWO]    = "LED_TWO";
    state_to_str[TWO_THREE]  = "TWO_THREE";
    state_to_str[LED_THREE]  = "LED_THREE";
    state_to_str[THREE_FOUR] = "THREE_FOUR";
    state_to_str[LED_FOUR]   = "LED_FOUR";
    state_to_str[FOUR_FIVE]  = "FOUR_FIVE";
    state_to_str[LED_FIVE]   = "LED_FIVE";

}

void ControlSM::run()
{
    switch(state) {
        case WAIT:
            if (sudoku_found) {
                transferState(READY);
            }
            break;
        case READY:
            if (led[0] != -1) {
                ROS_INFO_STREAM("Ready Sudoku Confirm: " << sudoku_confirm[led[0]]);
                if (sudoku[led[0]] != -1 && sudoku_confirm[led[0]] > 50) {
                    transferState(LED_ONE);
                }
            }
            break;
        case LED_ONE:
            if (rising_edge) {
                rising_edge = false;
                memcpy(led_last, led, sizeof(led));
                transferState(ONE_TWO);
            }
            break;
        case ONE_TWO:
            if (falling_edge) {
                falling_edge = false;
                if (led_remain && sudoku_confirm[led[1]] > 50) {
                    transferState(LED_TWO);
                } else if (led[0] != -1 && sudoku_confirm[led[0]] > 50) {
                    transferState(LED_ONE);
                } else {
                    transferState(WAIT);
                }
            }
            break;
        default: transferState(WAIT); break;
    }
}

void ControlSM::setLed(vector<int16_t> data)
{
    if (data.size() != 5)
        return;
    for (uint i=0; i<5; ++i) {
        //led_last[i] = led[i];
        led[i] = data[i];
    }
    checkLed();
}

void ControlSM::setSudoku(vector<int16_t> data)
{
    if (data.size() != 20)
        return;
    for (uint i=0; i<10; ++i) {
        sudoku_last[i] = sudoku[i];
        sudoku[i] = data[i];
    }
    for (uint i=0; i<10; ++i) {
        sudoku_confirm[i] = data[i+10];
    }
}

void ControlSM::setSudokuFound()
{
    sudoku_found = true;
}

void ControlSM::checkLed()
{
    int led_diff = 0;
    for (int i = 0; i < 5; ++i) {
        led_diff += (led_last[i] != led[i]);
    }
    ROS_INFO_STREAM("Led Diff: " << led_diff);
    led_remain = (led_diff <= 1);
}

void ControlSM::publishSudokuLedMnist(ros::Publisher& sudoku_pub,
        ros::Publisher& led_pub, ros::Publisher& mnist_pub)
{
    if (!publish_run) {
        return;
    }
    static std_msgs::Bool sudoku_msg, led_msg, mnist_msg;
    sudoku_msg.data = sudoku_run;
    led_msg.data    = led_run;
    mnist_msg.data  = mnist_run;
    sudoku_pub.publish(sudoku_msg);
    led_pub   .publish(led_msg);
    mnist_pub .publish(mnist_msg);
    publish_run = false;
}

void ControlSM::publishMnist(ros::Publisher& mnist_pub)
{
    if (!mnist_id_publish)
        return;
    static std_msgs::Int16MultiArray mnist_msg;
    mnist_msg.data.clear();
    if (state >= LED_ONE) {
        mnist_msg.data.push_back(sudoku[getLedNow()]);
        ROS_INFO_STREAM("Publish Mnist Id: " << sudoku[getLedNow()]);
    } else {
        mnist_msg.data.push_back(-1);
        ROS_INFO_STREAM("No target");
    }
    mnist_id_publish = false;
    mnist_pub.publish(mnist_msg);
}

int ControlSM::getLedNow()
{
    if (state <= READY)
        return 0;
    return led[state / 2 - 1];
}

void ControlSM::freshCtr()
{
    switch (state) {
    case WAIT:
        mnist_id_publish = true;
        sudoku_run = true;
        sudoku_found = led_run = mnist_run = tick_run = false;
        break;
    case READY:
        mnist_id_publish = true;
        led_run = mnist_run = tick_run = true;
        sudoku_run = false;
        break;
    case LED_ONE:
        mnist_id_publish = true;
        sudoku_run = led_run = mnist_run = false; break;
    case ONE_TWO:
        sudoku_run = led_run = mnist_run = true; break;
    case LED_TWO:
        sudoku_run = led_run = mnist_run = false; break;
    case TWO_THREE:
        sudoku_run = led_run = mnist_run = true; break;
    case LED_THREE:
        sudoku_run = led_run = mnist_run = false; break;
    case THREE_FOUR:
        sudoku_run = led_run = mnist_run = true; break;
    case LED_FOUR:
        sudoku_run = led_run = mnist_run = false; break;
    case FOUR_FIVE:
        sudoku_run = led_run = mnist_run = true; break;
    case LED_FIVE:
        sudoku_run = led_run = mnist_run = false; break;
    default:
        state = WAIT;
        sudoku_run = led_run = mnist_run = true; break;
    }
}

void ControlSM::tick(bool msg_data)
{
    static bool on_change = false;
    if (on_change == false && msg_data == true) {
        on_change    = true;
        rising_edge  = true;
        ROS_INFO("Rising Edge!");
        return;
    }
    if (on_change == true && msg_data == false) {
        on_change    = false;
        falling_edge = true;
        ROS_INFO("Falling Edge!");
        return;
    }
}

void ControlSM::transferNext()
{
    transferState((State)((state + 1) % STATE_SIZE));
}

bool ControlSM::sudokuChange()
{
    int change_cnt = 0;
    for (int i = 0; i < 9; ++i) {
        if (sudoku[i] != sudoku_last[i]) {
            ++change_cnt;
        }
    }
    return change_cnt > 3;
}
