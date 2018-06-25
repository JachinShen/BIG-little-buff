#include "Headers.h"

class ControlSM {
public:
    ControlSM();
    ~ControlSM();
    void init();
    void run();
    void setLed(int index, int data);
    void setSudoku(int index, int data);

private:
    enum State {
        WAIT,
        LED_ONE,
        LED_TWO,
        LED_THREE,
        LED_FOUR,
        LED_FIVE
    } state;

    int led[5];
    int sudoku[9];
    int sudoku_last[9];

    void transferState(State s);
    bool sudokuChange();
};
