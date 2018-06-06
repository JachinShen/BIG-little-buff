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
    if (state == WAIT) {

    } else if (state == LED_ONE) {

    } else if (state == LED_TWO) {

    } else if (state == LED_THREE) {

    } else if (state == LED_FOUR) {

    } else if (state == LED_FIVE) {
    }
}
