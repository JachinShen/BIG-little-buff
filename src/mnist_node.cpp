#include "sudoku/DnnClassifier.h"

static     ros::Publisher             mnist_num_pub;
static     ros::Publisher             aim_rect_pub;
static     cv_bridge::CvImageConstPtr cv_ptr;
static     DnnClassifier              mnist_classifier;
static     bool                       mnist_run;
static     Rect                       sudoku_rect;
static     vector<Rect>               mnist_rect;

void process() {
    if (!mnist_run) {
        //ROS_INFO("Ignore Mnist!");
        return;
    }

    if (sudoku_rect.area() == 0)
        return;

    static Mat img, img_roi, gray, binary;
    static vector<Mat> mnist_roi;
    vector<vector<Point> > contours;

    img = cv_ptr->image.clone();
    if (img.empty()) {
        ROS_ERROR("Empty Image");
        return;
    }
    //ROS_INFO_STREAM("Sudoku Rect: " << sudoku_rect);
    img_roi = img(sudoku_rect);
    //cvtColor(img_roi, gray, CV_BGR2GRAY);
    gray = img_roi;
    threshold(gray, binary, 128, 255, CV_THRESH_BINARY);

    findContours(binary.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    mnist_rect.clear();
    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 500)
            continue;
        mnist_rect.push_back(bound);
    }

    sort(mnist_rect.begin(), mnist_rect.end(), compareRect);
    mnist_roi.clear();
    binary = ~binary;
    for (uint i = 0; i < mnist_rect.size(); ++i) {
        Mat roi = (binary)(mnist_rect[i]);
        //copyMakeBorder(roi, roi, 15, 15, 15, 15, BORDER_CONSTANT);
        resize(roi, roi, Size(28, 28));
        mnist_roi.push_back(roi);
    }

    mnist_classifier.process(mnist_roi);

    static std_msgs::Int16MultiArray mnist_num_msg;
    mnist_num_msg.data.clear();
    for (uint i = 0; i < 10; ++i) {
        mnist_num_msg.data.push_back(mnist_classifier.getNumberBlockID(i));
    }
    for (uint i = 0; i < 10; ++i) {
        mnist_num_msg.data.push_back(mnist_classifier.confirmNumber(i));
    }
    mnist_num_pub.publish(mnist_num_msg);

#if DRAW == SHOW_ALL
    for (uint i = 0; i < mnist_rect.size(); ++i) {
        rectangle(img_roi, mnist_rect[i], Scalar(255, 0, 0), 2);
    }
    imshow("Mnist", img_roi);
#endif

    if (mnist_classifier.confirm()) {
        mnist_run = false;
    }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    process();
}

void handwriteRectsCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 4) {
        ROS_ERROR("Mnist Rect");
        return;
    }
    sudoku_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

void mnistParamCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO_STREAM("Mnist Param: " << msg.data[0]);
}

void mnistCtrCallback(const std_msgs::Bool& msg)
{
    mnist_run = msg.data;
    process();
}

void mnistIdCallback(const std_msgs::Int16MultiArray& msg)
{
    static std_msgs::Int16MultiArray aim_rect_msg;
    static Rect aim_rect;
    ROS_INFO_STREAM("Mnist Get Id: " << msg.data[0]);
    aim_rect_msg.data.clear();
    if (msg.data[0] >= 1) {
        aim_rect = mnist_rect[msg.data[0]-1];
        aim_rect.x += sudoku_rect.x;
        aim_rect.y += sudoku_rect.y;
        aim_rect_msg.data.push_back(aim_rect.x);
        aim_rect_msg.data.push_back(aim_rect.y);
        aim_rect_msg.data.push_back(aim_rect.width);
        aim_rect_msg.data.push_back(aim_rect.height);
    } else {
        aim_rect_msg.data.push_back(0);
        aim_rect_msg.data.push_back(0);
        aim_rect_msg.data.push_back(0);
        aim_rect_msg.data.push_back(0);
    }
    aim_rect_pub.publish(aim_rect_msg);
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mnist");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.1), waitkeyTimerCallback);

    ROS_INFO("Mnist Start!");
    mnist_num_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_num", 1, true);
    aim_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/aim_rect", 1, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/gray", 1, imageCallback);
    ros::Subscriber sudoku_rect_sub
        = nh.subscribe("buff/sudoku_rect", 1, handwriteRectsCallback);
    ros::Subscriber mnist_param_sub
        = nh.subscribe("buff/mnist_param", 1, mnistParamCallback);
    ros::Subscriber mnist_ctr_sub
        = nh.subscribe("buff/mnist_ctr", 1, mnistCtrCallback);
    ros::Subscriber mnist_id_sub
        = nh.subscribe("buff/mnist_id", 1, mnistIdCallback);

    string model_file   = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.prototxt";
    string trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.caffemodel";
    string mean_file    = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file   = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/synset_words.txt";

    mnist_classifier.init(model_file, trained_file, mean_file, label_file);

    mnist_run = false;
    ros::spin();

    return 0;
}
