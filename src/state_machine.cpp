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
    for (int i = 0; i < 6; ++i)
        led[i] = -1;

    for (int i = 0; i < 10; ++i)
        sudoku[i] = -1;
}

void ControlSM::setLed(int index, int data)
{
    led[index] = data;
}

void ControlSM::setSudoku(int index, int data)
{
    sudoku[index] = data;
}

void ControlSM::run()
{
    if (!sudokuChange()) {
        return;
    }
    ROS_INFO("Tick!");

    if (state == WAIT) {
        if (sudoku[0] != -1) {
            transferState(LED_ONE);
        }
    } else if (state == LED_ONE) {
        if (sudoku[1] != -1) {
            transferState(LED_ONE);
        }
    } else if (state == LED_TWO) {
        if (sudoku[2] != -1) {
            transferState(LED_ONE);
        }
    } else if (state == LED_THREE) {
        if (sudoku[3] != -1) {
            transferState(LED_ONE);
        }
    } else if (state == LED_FOUR) {
        if (sudoku[4] != -1) {
            transferState(LED_ONE);
        }
    } else if (state == LED_FIVE) {
        transferState(WAIT);
    }
}

bool ControlSM::sudokuChange()
{
    int change_cnt = 0;
    for(int i=0; i<9; ++i) {
        if (sudoku[i] != sudoku_last[i]) {
            ++change_cnt;
        }
    }
    memcpy(sudoku_last, sudoku, sizeof(sudoku_last));
    return change_cnt > 7;
}
