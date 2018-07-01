#include "Headers.h"

class ControlSM {
public:
    ControlSM();
    ~ControlSM();
    void init();
    void run();
    void tick(bool msg_data);
    void publishSudokuLedMnist(ros::Publisher&, ros::Publisher&, ros::Publisher&);
    void publishMnist(ros::Publisher&);
    void setLed(vector<int16_t> data);
    void setSudoku(vector<int16_t> data);
    void setSudokuFound();
    void checkLed();
    int  getLedNow();

private:
    enum State {
        WAIT,
        READY,
        LED_ONE,
        ONE_TWO,
        LED_TWO,
        TWO_THREE,
        LED_THREE,
        THREE_FOUR,
        LED_FOUR,
        FOUR_FIVE,
        LED_FIVE,
        STATE_SIZE
    } state;

    string state_to_str[STATE_SIZE];

    //bool on_change;
    bool rising_edge;
    bool falling_edge;
    bool sudoku_found;

    int led[5];
    int led_last[5];
    int sudoku[10];
    int sudoku_confirm[10];
    int sudoku_last[10];

    bool led_remain;

    bool tick_run;
    bool publish_run;
    bool sudoku_run;
    bool mnist_run;
    bool fire_run;
    bool led_run;

    bool mnist_id_publish;

    void transferState(State s);
    void transferNext();
    bool sudokuChange();
    void freshCtr();
};
