#include "Headers.h"

static ros::Publisher tick_pub;
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
    static std_msgs::Bool tick_msg;
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
        resize(roi, roi, roi_last.size());
        bitwise_xor(roi, roi_last, roi_diff);

        imshow("roi", roi);
        imshow("roi last", roi_last);
        imshow("roi diff", roi_diff);
        waitKey(1);

        float difference = (float) countNonZero(roi_diff / (roi_diff.cols * roi_diff.rows));
        ROS_INFO_STREAM("Difference: " << difference);

        tick_msg.data = difference > 0.1;
        tick_pub.publish(tick_msg);
    }

    roi_last = roi.clone();

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tick");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber sudoku_rect_sub
        = nh.subscribe("buff/sudoku_rect", 1, sudokuRectCallback);

    tick_pub = nh.advertise<std_msgs::Bool>("buff/tick", 1);

    ros::spin();
}
