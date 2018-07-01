#include "kcftracker.hpp"
#include "sudoku/BlockSplit.h"
#include "Headers.h"

static Mat frame;
static Rect aim_rect;
static KCFTracker tracker(false, true, false, false);

void aimRectCallback(const std_msgs::Int16MultiArray& msg)
{
    aim_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    ROS_INFO_STREAM("Aim Rect: " << aim_rect);
    tracker.init(aim_rect, frame);
}

int main(int argc, char* argv[])
{
    ROS_INFO("Aim Start!");
    if (argv[1] == NULL) {
        ROS_ERROR("argv[1]=NULL, no video source\n");
        return 1;
    }

    ros::init(argc, argv, "aim");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber aim_sub = nh.subscribe("buff/aim_rect", 1, aimRectCallback);

    Rect led_rect, sudoku_rect;
    enum {VIDEO_FILE, VIDEO_CAMERA} video_type;
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
            ros::spinOnce();
            if (aim_rect.area() != 0) {
                aim_rect = tracker.update(frame);
                rectangle(frame, aim_rect, 255, 4);
            }
            imshow("aim src", frame);
            waitKey(1);
            loop_rate.sleep();
        }
    }

    return 0;
}
