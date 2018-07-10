#include "sudoku/LedSolver.h"

static ros::Publisher             led_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static Rect                       led_rect;
static LedSolver                  led_solver;
static bool                       led_run;

void process()
{
    if (!led_run)
        return;

    if (led_rect.area() == 0)
        return;

    static std_msgs::Int16MultiArray led_num_msg;
    static Mat                       img;
    static Mat                       led_roi;

    img = cv_ptr->image.clone();
    if (img.empty()) {
        ROS_ERROR("Empty Image");
        return;
    }

    ROS_INFO_STREAM("Led Rect: " << led_rect);
    led_roi = Mat(img, led_rect);
    if (led_solver.process(led_roi)) {
        led_num_msg.data.clear();
        for (uint i = 0; i < 5; ++i)
            led_num_msg.data.push_back(led_solver.getResult(i));
        led_num_pub.publish(led_num_msg);
    } else {
        led_num_msg.data.clear();
        for (uint i = 0; i < 5; ++i)
            led_num_msg.data.push_back(-1);
        led_num_pub.publish(led_num_msg);
    }

    if (led_solver.confirmLed()) {
        led_run = false;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("Led Solver Image Call Back");
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    process();
    //if (!led_run)
        //return;
    //static std_msgs::Int16MultiArray led_num_msg;
    //led_num_msg.data.clear();
    //for (uint i = 0; i < 5; ++i)
        //led_num_msg.data.push_back(i+1);
    //led_num_pub.publish(led_num_msg);
    //led_run = false;
}

void ledRectCallback(const std_msgs::Int16MultiArray& msg)
{
    led_rect = Rect(msg.data[0], msg.data[1],
        msg.data[2], msg.data[3]);
}

void ledParamCallback(const std_msgs::Int16MultiArray& msg)
{
    //ROS_INFO_STREAM("Get Param" << msg.data[0]);
    led_solver.setParam(msg.data[0], msg.data[1]);
}

void ledCtrCallback(const std_msgs::Bool& msg)
{
    led_run = msg.data;
    process();
    //if (!led_run)
        //return;
    //static std_msgs::Int16MultiArray led_num_msg;
    //led_num_msg.data.clear();
    //for (uint i = 0; i < 5; ++i)
        //led_num_msg.data.push_back(i+1);
    //led_num_pub.publish(led_num_msg);
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "led");
    ROS_INFO("Led Start!");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.1), waitkeyTimerCallback);

    led_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/led_num", 1);

    //led_solver.init("./src/buff/svm/SVM_DATA_NUM.xml");
    led_solver.init();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);

    ros::Subscriber led_rect_sub
        = nh.subscribe("buff/led_rect", 1, ledRectCallback);
    ros::Subscriber led_param_sub
        = nh.subscribe("buff/led_param", 1, ledParamCallback);
    ros::Subscriber led_ctr_sub
        = nh.subscribe("buff/led_ctr", 1, ledCtrCallback);

    led_run = true;
    ros::spin();

    return 0;
}
