#include "Headers.h"

class ControlSM {
public:
    ControlSM();
    ~ControlSM();
    void init();
    bool tick(bool msg_data);
    void setLed(int index, int data);
    void setSudoku(int index, int data);
    void checkLed();
    int  getLedNow();

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
    int led_valid[5];
    int led_last[5];
    int led_last_valid[5];
    int sudoku[5];
    int sudoku_last[5];

    bool led_remain;

    bool sudoku_run;
    bool mnist_run;
    bool fire_run;
    bool led_run;

    void transferState(State s);
    void transferNext();
    bool sudokuChange();
};
