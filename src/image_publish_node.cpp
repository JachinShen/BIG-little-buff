#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sstream> // for converting the command line parameter to integer

#include <sys/time.h>
timeval timestart;

using namespace cv;

int main(int argc, char** argv)
{
    //Check if video source has been passed as a parameter
    ROS_INFO("Start!");
    if (argv[1] == NULL) {
        ROS_ERROR("argv[1]=NULL, no video source\n");
        return 1;
    }

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1, true);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(30);
    enum {VIDEO_FILE, VIDEO_CAMERA} video_type;
    while (nh.ok()) {
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


        while (cap.read(frame) && ros::ok()) {

            resize(frame, frame, Size(640, 480));
            imshow("src", frame);

            // Check if grabbed frame is actually full with some content
            if (!frame.empty()) {
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                pub.publish(msg);
            }

            if (video_type == VIDEO_FILE)
                waitKey(0);
            else if (video_type == VIDEO_CAMERA)
                waitKey(1);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
