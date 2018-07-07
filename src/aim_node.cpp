#include "GlobalCamera.h"
#include "Headers.h"
#include "kcftracker.hpp"
#include "sudoku/BlockSplit.h"

static Mat frame;
static Rect tl_aim_rect;
static Rect center_aim_rect;
static Rect rb_aim_rect;
static KCFTracker tl_tracker(false, true, false, false);
static KCFTracker center_tracker(false, true, false, false);
static KCFTracker rb_tracker(false, true, false, false);
static cv_bridge::CvImageConstPtr cv_ptr;
static bool aim_ready_run;
static BlockSplit block_split;
static enum TargetPos { TOP_LEFT,
    CENTER,
    RIGHT_BOTTOM } target_pos;
static ros::Publisher aim_pos_pub;
static ros::Subscriber aim_param_sub;

void process()
{
    if (!aim_ready_run)
        return;
    static Mat gray, binary;
    Rect led_rect, sudoku_rect;
    if (frame.channels() != 1)
        cvtColor(frame, gray, CV_BGR2GRAY);
    else
        gray = frame;
    if (block_split.process(gray, led_rect, sudoku_rect)) {
        tl_aim_rect = Rect(sudoku_rect.x, sudoku_rect.y, sudoku_rect.width / 3, sudoku_rect.height / 3);
        center_aim_rect = Rect(sudoku_rect.x + sudoku_rect.width / 3, sudoku_rect.y + sudoku_rect.height / 3,
            sudoku_rect.width / 3, sudoku_rect.height / 3);
        rb_aim_rect = Rect(sudoku_rect.x + sudoku_rect.width * 2 / 3, sudoku_rect.y + sudoku_rect.height * 2 / 3,
            sudoku_rect.width / 3, sudoku_rect.height / 3);
        ROS_INFO_STREAM("target : " << center_aim_rect);
        tl_tracker.init(tl_aim_rect, gray);
        center_tracker.init(center_aim_rect, gray);
        rb_tracker.init(rb_aim_rect, gray);
    }
}

void demarcate()
{
    if (center_aim_rect.area() != 0) {
        switch (target_pos) {
        case TOP_LEFT:
            tl_aim_rect = tl_tracker.update(frame);
            rectangle(frame, tl_aim_rect, 255, 4);
        case CENTER:
            center_aim_rect = center_tracker.update(frame);
            rectangle(frame, center_aim_rect, 255, 4);
        case RIGHT_BOTTOM:
            rb_aim_rect = rb_tracker.update(frame);
            rectangle(frame, rb_aim_rect, 255, 4);
            break;
        default:
            target_pos = TOP_LEFT;
        }
        static std_msgs::Int16MultiArray aim_pos_msg;
        aim_pos_msg.data.clear();
        switch (target_pos) {
        case TOP_LEFT:
            aim_pos_msg.data.push_back(tl_aim_rect.x + tl_aim_rect.width / 2);
            aim_pos_msg.data.push_back(tl_aim_rect.y + tl_aim_rect.height / 2);
            break;
        case CENTER:
            aim_pos_msg.data.push_back(center_aim_rect.x + center_aim_rect.width / 2);
            aim_pos_msg.data.push_back(center_aim_rect.y + center_aim_rect.height / 2);
            break;
        case RIGHT_BOTTOM:
            aim_pos_msg.data.push_back(rb_aim_rect.x + rb_aim_rect.width / 2);
            aim_pos_msg.data.push_back(rb_aim_rect.y + rb_aim_rect.height / 2);
            break;
        }
        if (320 - 1 <= aim_pos_msg.data[0] && aim_pos_msg.data[0] <= 320 + 1 ){
                //&& 240-20 <= aim_pos_msg.data[1] && aim_pos_msg.data[1] <= 240 + 20) {
            target_pos = (TargetPos)((target_pos + 1));
            if (target_pos >= 3) {
                aim_pos_msg.data.push_back(-1);
            } else {
                //aim_pos_msg.data.push_back(4);
                aim_pos_msg.data.push_back(int(target_pos) + 1);
            }
        } else {
            aim_pos_msg.data.push_back(int(target_pos) + 1);
            //aim_pos_msg.data.push_back(3);
        }
        aim_pos_pub.publish(aim_pos_msg);
    } else {
        process();
    }
}

void aimParamCallback(const std_msgs::Int16MultiArray& msg)
{
    block_split.setParam(msg.data[0], msg.data[1]);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void aimReadyCallback(const std_msgs::Bool& msg)
{
    ROS_INFO("Aim Ready Call!");
    aim_ready_run = msg.data;
    target_pos = TOP_LEFT;
    process();
    //aim_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    //ROS_INFO_STREAM("Aim Rect: " << aim_rect);

    //if (aim_rect.area() == 0)
    //return;

    //static Mat car_image;
    //cvtColor(cv_ptr->image.clone(), car_image, CV_BGR2GRAY);
    //tracker.init(aim_rect, car_image);
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

    ros::Subscriber aim_sub = nh.subscribe("buff/aim_ready", 1, aimReadyCallback);
    aim_param_sub = nh.subscribe("buff/aim_param", 1, aimParamCallback);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);
    aim_pos_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_pos", 100);

    Rect led_rect, sudoku_rect;
    block_split.init();
    aim_ready_run = true;
    while (nh.ok()) {
        enum { VIDEO_FILE, VIDEO_CAMERA } video_type;
        cv::VideoCapture cap;
        GlobalCamera global_cap;
        aim_ready_run = false;
        if ('0' <= argv[1][0] && argv[1][0] <= '9') {
            video_type = VIDEO_CAMERA;
            //cap.open(argv[1][0] - '0');
            if (global_cap.init() < 0) {
                ROS_ERROR("Global Shutter Camera Init Failed!");
                continue;
            }
        } else {
            video_type = VIDEO_FILE;
            cap.open(argv[1]);
            if (!cap.isOpened()) {
                ROS_INFO("can not opencv video device");
                return 1;
            }
            ROS_INFO("open successfully");
        }

        if (video_type == VIDEO_CAMERA) {
            if (! global_cap.read(frame)) {
                ROS_ERROR("Global Shutter Camera Read Img!");
                continue;
            }
            if (frame.empty()) {
                ROS_ERROR("Global Shutter Camera Read Img Empty!");
                continue;
            }
        }

        if (video_type == VIDEO_CAMERA) {
            while (ros::ok()) {
                ros::spinOnce();
                if (target_pos >= 3) {
                    loop_rate.sleep();
                    continue;
                }
                if (!global_cap.read(frame))
                    continue;
                if (frame.empty())
                    continue;
                demarcate();
                imshow("aim src", frame);
                waitKey(1);
            }
        } else if (video_type == VIDEO_FILE) {
            while (target_pos <= 2 && cap.read(frame) && ros::ok()) {
                if (frame.empty())
                    continue;
                resize(frame, frame, Size(640, 480));
                if (frame.channels() != 1) {
                    cvtColor(frame, frame, CV_BGR2GRAY);
                }
                ros::spinOnce();
                demarcate();
                imshow("aim src", frame);
                waitKey(1);
                //loop_rate.sleep();
            }
        }
    }

    return 0;
}
