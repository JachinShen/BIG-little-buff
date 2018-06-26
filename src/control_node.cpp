#include "state_machine.h"

static ros::Publisher aim_num_pub;
static ControlSM csm;
static cv_bridge::CvImageConstPtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void sudokuRectCallback(const std_msgs::Int16MultiArray& msg)
{
    static Mat img, roi, roi_last, roi_diff;
    img = cv_ptr->image;
    if (img.empty()) {
        ROS_ERROR("Empty Image");
        return;
    }

    Rect sudoku_rect = Rect(
            msg.data[0], msg.data[1],
            msg.data[2], msg.data[3]);
    roi = img(sudoku_rect);
    cvtColor(roi, roi, CV_BGR2GRAY);
    threshold(roi, roi, 128, 255, CV_THRESH_BINARY);

    if (!roi_last.empty()) {
        imshow("roi", roi);
        imshow("roi last", roi_last);
        resize(roi, roi, roi_last.size());
        bitwise_xor(roi, roi_last, roi_diff);
        imshow("roi diff", roi_diff);
        waitKey(1);
        ROS_INFO_STREAM("Difference: " << (float)countNonZero(roi_diff) / roi_diff.cols / roi_diff.rows);
    }

    roi_last = roi.clone();

}

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Led:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setLed(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void mnistNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Mnist:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void fireNumCallback(const std_msgs::Int16MultiArray& msg)
{
    cout << "Fire:";
    for (uint i = 0; i < msg.data.size(); ++i) {
        csm.setSudoku(i, msg.data[i]);
        cout << " " << msg.data[i];
    }
    cout << endl;
}

void fireRectCallback(const std_msgs::Int16MultiArray& msg)
{
    //cout << "Fire:";
    for (uint i = 0; i < msg.data.size()/4; ++i) {
        //csm.setSudoku(i, msg.data[i]);
        //cout << " " << msg.data[i];
    }
    //cout << endl;
}

void csmTimerCallback(const ros::TimerEvent&)
{
    csm.run();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "control");
    ROS_INFO("Start!");
    ros::NodeHandle nh;
    ros::Timer csm_timer = nh.createTimer(ros::Duration(0.1), csmTimerCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber sudoku_rect_sub
        = nh.subscribe("buff/sudoku_rect", 1, sudokuRectCallback);
    ros::Subscriber mnist_num_sub
        = nh.subscribe("buff/mnist_num", 1, mnistNumCallback);
    ros::Subscriber fire_num_sub
        = nh.subscribe("buff/fire_num", 1, fireNumCallback);
    ros::Subscriber fire_rect_sub
        = nh.subscribe("buff/fire_rect", 1, fireRectCallback);
    aim_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_rect", 1);

    ros::spin();

    return 0;
}
