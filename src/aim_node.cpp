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
static bool aim_ready_run;
static BlockSplit block_split;
static enum TargetPos { TOP_LEFT,
    CENTER,
    RIGHT_BOTTOM,
    POS_SIZE} target_pos;
static ros::Publisher aim_pos_pub;
static ros::Subscriber aim_param_sub;
static int offset_y;
static int offset_x;

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
        case POS_SIZE: ROS_INFO("Exit");
            return; break;
        default:
            target_pos = TOP_LEFT;
        }
        static std_msgs::Int16MultiArray aim_pos_msg;
        aim_pos_msg.data.clear();
        switch (target_pos) {
        case TOP_LEFT:
            aim_pos_msg.data.push_back(tl_aim_rect.x + tl_aim_rect.width / 2 + 10);
            aim_pos_msg.data.push_back(tl_aim_rect.y + tl_aim_rect.height / 2);
            break;
        case CENTER:
            aim_pos_msg.data.push_back(center_aim_rect.x + center_aim_rect.width / 2 - 15);
            aim_pos_msg.data.push_back(center_aim_rect.y + center_aim_rect.height / 2);
            break;
        case RIGHT_BOTTOM:
            aim_pos_msg.data.push_back(rb_aim_rect.x + rb_aim_rect.width / 2);
            aim_pos_msg.data.push_back(rb_aim_rect.y + rb_aim_rect.height / 2);
            break;
        case POS_SIZE: return;
        }
        aim_pos_msg.data[0] -= (offset_x - 100);
        aim_pos_msg.data[1] -= offset_y;
        //ROS_INFO_STREAM("Offset y" << offset_y);
        if (320 - 3 <= aim_pos_msg.data[0] && aim_pos_msg.data[0] <= 320 + 3
                && 240-3 <= aim_pos_msg.data[1] && aim_pos_msg.data[1] <= 240 + 3) {
            target_pos = (TargetPos)((target_pos + 1));
            if (target_pos >= POS_SIZE) {
                aim_pos_msg.data.push_back(-1);
                aim_ready_run = false;
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

void aimReadyCallback(const std_msgs::Bool& msg)
{
    ROS_INFO("Aim Ready Call!");
    aim_ready_run = msg.data;
    if (aim_ready_run) {
        target_pos = TOP_LEFT;
        center_aim_rect = Rect(0, 0, 0, 0);
        process();
    } else {
        target_pos = POS_SIZE;
        center_aim_rect = Rect(0, 0, 0, 0);
    }
    //aim_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    //ROS_INFO_STREAM("Aim Rect: " << aim_rect);

    //if (aim_rect.area() == 0)
    //return;

    //static Mat car_image;
    //cvtColor(cv_ptr->image.clone(), car_image, CV_BGR2GRAY);
    //tracker.init(aim_rect, car_image);
}

string getFilename()
{
#if PLATFORM == PC
    fstream fin("/home/jachinshen/Projects/lunar_ws/src/buff/out2.txt");
#elif PLATFORM == MANIFOLD
    fstream fin("/home/ubuntu/lunar_ws/src/buff/out2.txt");
#endif
    int video_cnt;
    fin>>video_cnt;
    stringstream video_ss;
    video_ss<<video_cnt;

    fin.close();
    std::string file_name=video_ss.str();
#if PLATFORM == PC
    fstream fout("/home/jachinshen/Projects/lunar_ws/src/buff/out2.txt");
#elif PLATFORM == MANIFOLD
    fstream fout("/home/ubuntu/lunar_ws/src/buff/out2.txt");
#endif
    video_cnt++;
    fout<<video_cnt;
    fout.close();

#if PLATFORM == PC
    file_name = "/home/jachinshen/record_vice"+file_name+".avi";
#elif PLATFORM == MANIFOLD
    file_name = "/home/ubuntu/record_vice"+file_name+".avi";
#endif
    return file_name;
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
    aim_pos_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_pos", 100);

    Rect led_rect, sudoku_rect;
    block_split.init();
    aim_ready_run = true;
    center_aim_rect = Rect(0, 0, 0, 0);
    offset_y = 53;
    offset_x = 91;
    while (nh.ok()) {
        enum { VIDEO_FILE, VIDEO_CAMERA } video_type;
        cv::VideoCapture cap;
        GlobalCamera global_cap;
#if RECORD == RECORD_ON
        cv::VideoWriter g_writer;
        g_writer.open(getFilename(), CV_FOURCC('P','I','M','1'),30,cv::Size(640,480));
#endif
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
                if (target_pos == POS_SIZE) {
                    loop_rate.sleep();
                    continue;
                }
                if (!global_cap.read(frame))
                    continue;
                if (frame.empty())
                    continue;
                if (frame.channels() != 1) {
                    cvtColor(frame, frame, CV_BGR2GRAY);
                }
                demarcate();
                imshow("AimSrc", frame);
                createTrackbar("offset y", "AimSrc", &offset_y, 100);
                createTrackbar("offset x", "AimSrc", &offset_x, 200);
#if RECORD == RECORD_ON
                g_writer.write(frame);
#endif
                waitKey(1);
            }
        } else if (video_type == VIDEO_FILE) {
            while (ros::ok()) {
                ros::spinOnce();
                if (target_pos == POS_SIZE)
                    continue;
                if (!cap.read(frame))
                    continue;
                if (frame.empty())
                    continue;
                resize(frame, frame, Size(640, 480));
                if (frame.channels() != 1) {
                    cvtColor(frame, frame, CV_BGR2GRAY);
                }
                demarcate();
                imshow("AimSrc", frame);
                createTrackbar("offset y", "AimSrc", &offset_y, 100);
                createTrackbar("offset x", "AimSrc", &offset_x, 200);
                char key_press = waitKey(1);
                if (key_press == 'c') {
                    target_pos = POS_SIZE;
                }
                //loop_rate.sleep();
            }
        }
    }

    return 0;
}
