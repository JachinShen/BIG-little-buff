#include "state_machine.h"

ControlSM::ControlSM()
{
}

ControlSM::~ControlSM()
{
}

void ControlSM::transferState(State s)
{
    ROS_INFO_STREAM("Transfer to state: "<< s);
    state = s;
}

void ControlSM::init()
{
    state = WAIT;
    sudoku_run = true;
    mnist_run  = true;
    fire_run   = true;
    led_run    = true;
    on_change  = false;
    led_remain = false;

    for (int i = 0; i < 6; ++i)
        led[i] = -1;

    for (int i = 0; i < 10; ++i)
        sudoku[i] = -1;
}

void ControlSM::setLed(int index, int data)
{
    if (led[index] == data) {
        if (led_valid[index] != data) {
            led_last_valid[index] = led_valid[index];
        }
        led_valid[index] = data;
    }

    led_last[index] = led[index];
    led[index] = data;
}

void ControlSM::setSudoku(int index, int data)
{
    sudoku_last[index] = sudoku[index];
    sudoku[index] = data;
}

void ControlSM::checkLed()
{
    int led_diff = 0;
    for (int i=0; i<5; ++i) {
        led_diff += (led_last_valid[i] != led_valid[i]);
    }
    led_remain = (led_diff <= 1);
}

int ControlSM::getLedNow()
{
    return led_valid[state];
}

bool ControlSM::tick(bool msg_data)
{
    if (on_change == false && msg_data == true) {
        on_change = true;
        return true;
    }

    if (on_change == true && msg_data == false) {
        on_change = false;
        ROS_INFO("Tick!");

        if (led_remain) {
            transferNext();
        } else {
            transferState(WAIT);
        }
        return true;
    }

    return false;
}

void ControlSM::transferNext()
{
    if (state == LED_FIVE) {
        transferState(WAIT);
        return;
    }

    if (sudoku[state - WAIT] != -1) {
        transferState((State)(state + 1));
    }

    //if (state == WAIT) {
        //if (sudoku[0] != -1) {
            //transferState(LED_ONE);
        //}
    //} else if (state == LED_ONE) {
        //if (sudoku[1] != -1) {
            //transferState(LED_TWO);
        //}
    //} else if (state == LED_TWO) {
        //if (sudoku[2] != -1) {
            //transferState(LED_THREE);
        //}
    //} else if (state == LED_THREE) {
        //if (sudoku[3] != -1) {
            //transferState(LED_FOUR);
        //}
    //} else if (state == LED_FOUR) {
        //if (sudoku[4] != -1) {
            //transferState(LED_FIVE);
        //}
    //} else if (state == LED_FIVE) {
        //transferState(WAIT);
    //}
}

bool ControlSM::sudokuChange()
{
    int change_cnt = 0;
    for(int i=0; i<9; ++i) {
        if (sudoku[i] != sudoku_last[i]) {
            ++change_cnt;
        }
    }
    return change_cnt > 3;
}
