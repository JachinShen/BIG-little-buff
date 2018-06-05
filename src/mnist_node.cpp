#include "Headers.h"

static ros::Publisher mnist_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void handwriteRectsCallback(const std_msgs::Int16MultiArray& msg)
{
    static std_msgs::Int16MultiArray mnist_num_msg;
    static Mat img;
    static Mat mnist_roi;
    static Rect mnist_rect;

    mnist_rect = Rect(msg.data[0], msg.data[1],
            msg.data[2], msg.data[3]);

    img = cv_ptr->image;
    if (img.empty()) {
        cout << "Empty Image" << endl;
        return;
    }

    mnist_roi = Mat(img, mnist_rect);
}

void mnistParamCallback(const std_msgs::Int16MultiArray& msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "led");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    mnist_num_pub 
        = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_num", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub 
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber led_rect_sub 
        = nh.subscribe("buff/handwrite_rects", 1, handwriteRectsCallback);
    ros::Subscriber led_param_sub
        = nh.subscribe("buff/mnist_param", 1, mnistParamCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
