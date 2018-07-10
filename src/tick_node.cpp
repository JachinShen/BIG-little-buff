#include "Headers.h"

static ros::Publisher tick_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static Rect sudoku_rect;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    static Mat img, roi, roi_last, roi_diff;
    static std_msgs::Bool tick_msg;
    img = cv_ptr->image.clone();
    if (img.empty()) {
        ROS_ERROR("Empty Image");
        return;
    }

    //Mat kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
    roi = img(sudoku_rect);
    cvtColor(roi, roi, CV_BGR2GRAY);
    threshold(roi, roi, 100, 255, CV_THRESH_BINARY);
    //erode(roi, roi, kernel);

    if (!roi_last.empty()) {
        resize(roi, roi, roi_last.size());
        bitwise_xor(roi, roi_last, roi_diff);
        //erode(roi_diff, roi_diff, kernel);

        //imshow("roi", roi);
        //imshow("roi last", roi_last);
        imshow("roi diff", roi_diff);

        float difference = (float) countNonZero(roi_diff) / (roi_diff.cols * roi_diff.rows);
        //ROS_INFO_STREAM("Difference: " << difference);

        //TODO: Find the falling edge more smartly.
        //tick_msg.data = difference > 0.08;
        tick_msg.data = false;
        tick_pub.publish(tick_msg);
    }

    roi_last = roi.clone();

}

void sudokuRectCallback(const std_msgs::Int16MultiArray& msg)
{
    sudoku_rect = Rect(
            msg.data[0], msg.data[1],
            msg.data[2], msg.data[3]);
}

void mnistNumCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 20) {
        ROS_ERROR("Tick Wrong Size!");
        return;
    }
    static int mnist_num[10] = {0};
    //static int mnist_num_confirm[10] = {0};
    static int mnist_num_last[10] = {0};
    //static int mnist_num_confirm[10] = {0};
    for (int i=0; i<10; ++i) {
        if (msg.data[i+10] > 60) {
            mnist_num[i] = msg.data[i];
        }
    }
    int diff_ctr = 0;
    for (int i=0; i<10; ++i) {
        if (mnist_num[i] != mnist_num_last[i]) {
            ++diff_ctr;
        }
    }
    static std_msgs::Bool tick_msg;
    tick_msg.data = diff_ctr > 3;
    tick_pub.publish(tick_msg);
    memcpy(mnist_num_last, mnist_num, sizeof(mnist_num));
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

void hitTimerCallback(const ros::TimerEvent&)
{
    static std_msgs::Bool tick_msg;
    tick_msg.data = true;
    tick_pub.publish(tick_msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tick");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.1), waitkeyTimerCallback);
    //ros::Timer hit_timer = nh.createTimer(ros::Duration(1), hitTimerCallback);

    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub
        //= it.subscribe("camera/image", 1, imageCallback);
    //ros::Subscriber sudoku_rect_sub
        //= nh.subscribe("buff/sudoku_rect", 1, sudokuRectCallback);
    ros::Subscriber mnist_num_sub
        = nh.subscribe("buff/mnist_num", 1, mnistNumCallback);

    tick_pub = nh.advertise<std_msgs::Bool>("buff/tick", 1);

    ros::spin();
}
