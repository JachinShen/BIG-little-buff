#include "Headers.h"

class ControlSM {
public:
    ControlSM();
    ~ControlSM();
    void init();
    void tick(bool msg_data);
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

    bool on_change;

    int led[5];
    int sudoku[9];
    int sudoku_last[9];


    bool sudoku_run;
    bool mnist_run;
    bool fire_run;
    bool led_run;

    void transferState(State s);
    bool sudokuChange();
};
