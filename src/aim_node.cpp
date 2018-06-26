#include "kcftracker.hpp"
#include "sudoku/BlockSplit.h"
#include "Headers.h"

static Rect aim_rect;

void aimRectCallback(const std_msgs::Int16MultiArray& msg)
{
    aim_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

int main(int argc, char* argv[])
{
    ROS_INFO("Start!");
    if (argv[1] == NULL) {
        ROS_ERROR("argv[1]=NULL, no video source\n");
        return 1;
    }

    ros::init(argc, argv, "aim");
    ros::NodeHandle nh;

    ros::Subscriber aim_sub = nh.subscribe("buff/aim_rect", 1, aimRectCallback);

    Mat frame;
    Rect led_rect, sudoku_rect;
    enum {VIDEO_FILE, VIDEO_CAMERA} video_type;
    KCFTracker tracker(false, true, false, false);
    while(nh.ok()) {
        cv::VideoCapture cap;
        if ('0' <= argv[1][0] && argv[1][0] <= '9') {
            video_type = VIDEO_CAMERA;
            cap.open(argv[1][0] - '0');
        } else {
            video_type = VIDEO_FILE;
            cap.open(argv[1]);
        }

        if (!cap.isOpened()) {
            ROS_INFO("can not opencv video device");
            return 1;
        }
        ROS_INFO("open successfully");

        while(cap.read(frame) && ros::ok()) {
            if (frame.empty())
                continue;
            resize(frame, frame, Size(640, 480));
            if (frame.channels() != 1) {
                cvtColor(frame, frame, CV_BGR2GRAY);
            }
            imshow("src", frame);
            waitKey(0);
        }
    }

    return 0;
}
