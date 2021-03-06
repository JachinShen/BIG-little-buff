#include "Headers.h"

class ControlSM {
public:
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
    enum BuffType {
        MNIST,
        FIRE
    } buff_type;
    bool wait_for_demarcate;

    ControlSM();
    ~ControlSM();
    void transferState(State s);
    void init();
    void run();
    void tick(bool msg_data);
    void publishSudokuLedMnistFire(ros::Publisher&, ros::Publisher&, ros::Publisher&, ros::Publisher&);
    void publishMnist(ros::Publisher&);
    void setLed(vector<int16_t> data);
    void setSudoku(vector<int16_t> data);
    void setSudokuFound();
    void setDemarcateComplete();
    void checkLed();
    int getLedNow();
    int getBlockIdNow();
    int sendBlockID();
    bool isWait() {return state == WAIT;}
    void setBuffType(BuffType type) {buff_type = type;}
    void setDownline() {wait_for_downline = false;}
    //bool serial_send;

private:
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
    bool sudoku_fresh;
    bool led_fresh;
    bool serial_send;
    bool wait_for_downline;

    void transferNext();
    bool sudokuChange();
    void freshCtr();

    inline void activateSudoku() {sudoku_run = true;}
    inline void pauseSudoku() {sudoku_run = false;}
    inline void activateLedMnistFire() {
        led_run = true;
        if (buff_type == MNIST) 
            mnist_run = true;
        else if (buff_type == FIRE)
            fire_run = true;
    }
    inline void pauseLedMnistFire() {led_run = mnist_run = fire_run = false;}
    inline void activateTick() {tick_run = true;}
    inline void pauseTick() {tick_run = false;}
    inline void activateSerial() {ROS_INFO("Activate Serial"); serial_send = true;}
    inline void resetSudokuFound() {sudoku_found = false;}
    inline void needFreshLedSudoku() {sudoku_fresh = led_fresh = false;}
};
