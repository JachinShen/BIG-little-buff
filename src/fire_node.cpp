#include "sudoku/DnnClassifier.h"

static ros::Publisher             fire_num_pub;
//static ros::Publisher             fire_rect_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static DnnClassifier              fire_classifier;
static bool                       fire_run;
static Rect                       sudoku_rect;

void process()
{
    if (!fire_run) {
        //ROS_INFO("Ignore Fire!");
        return;
    }

    if (sudoku_rect.area() == 0)
        return;

    static Mat img, gray, binary, sudoku_roi;
    static vector<Mat> fire_roi;
    vector<Rect>               fire_rect;
    vector<vector<Point> > contours;

    img = cv_ptr->image;
    if (img.empty()) {
        ROS_ERROR("Empty Image");
        return;
    }

    sudoku_roi = img(sudoku_rect);
    //cvtColor(sudoku_roi, gray, CV_BGR2GRAY);
    gray = sudoku_roi;
    threshold(gray, binary, 180, 255, CV_THRESH_BINARY);

    findContours(binary.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 500)
            continue;
        fire_rect.push_back(bound);
    }
    sort(fire_rect.begin(), fire_rect.end(), compareRect);

    fire_roi.clear();
    for (uint i=0; i<fire_rect.size(); ++i) {
        Mat roi = (binary(fire_rect[i]));
        int left_right_gap = (roi.rows - roi.cols) / 2 + 5;
        if (left_right_gap <= 0)
            continue;
        copyMakeBorder(roi, roi, 5, 5, left_right_gap, left_right_gap, BORDER_CONSTANT);
        resize(roi, roi, Size(28, 28));
        fire_roi.push_back(roi);
    }
    fire_classifier.process(fire_roi);

    static std_msgs::Int16MultiArray fire_num_msg;
    fire_num_msg.data.clear();
    //int block_id;
    for (uint i = 0; i < 10; ++i) {
        //block_id = fire_classifier.getNumberBlockID(i);
        fire_num_msg.data.push_back(fire_classifier.getNumberBlockID(i));
    }
    for (uint i = 0; i < 10; ++i) {
        fire_num_msg.data.push_back(fire_classifier.confirmNumber(i));
    }
    fire_num_pub.publish(fire_num_msg);
#if DRAW == SHOW_ALL
    //imshow("sudoku roi", sudoku_roi);
    for (uint i = 0; i < fire_rect.size(); ++i) {
        rectangle(binary, fire_rect[i], Scalar(255, 0, 0), 2);
    }
    imshow("Fire", binary);
#endif

    if (fire_classifier.confirm()) {
        fire_run = false;
        ROS_INFO("Set Fire Stop!");
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

void fireRectsCallback(const std_msgs::Int16MultiArray& msg)
{
    if (msg.data.size() != 4) {
        ROS_ERROR("Fire Rect");
        return;
    }
    sudoku_rect = Rect(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

void fireParamCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO_STREAM("Fire Param: " << msg.data[0]);
}

void fireCtrCallback(const std_msgs::Bool& msg)
{
    fire_run = msg.data;
    ROS_INFO("Fire Ctr Call");
    process();
}

void waitkeyTimerCallback(const ros::TimerEvent&)
{
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "fire");
    ROS_INFO("Fire Start!");
    ros::NodeHandle nh;
    ros::Timer waitkey_timer = nh.createTimer(ros::Duration(0.1), waitkeyTimerCallback);

    fire_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/fire_num", 1, true);
    //fire_rect_pub
        //= nh.advertise<std_msgs::Int16MultiArray>("buff/fire_rect", 1, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/gray", 1, imageCallback);
    ros::Subscriber sudoku_rect_sub
        = nh.subscribe("buff/sudoku_rect", 1, fireRectsCallback);
    //ros::Subscriber led_num_sub
        //= nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber fire_param_sub
        = nh.subscribe("buff/fire_param", 1, fireParamCallback);
    ros::Subscriber fire_ctr_sub
        = nh.subscribe("buff/fire_ctr", 1, fireCtrCallback);

    string model_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.prototxt";
    string trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.caffemodel";
    string mean_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/synset_words.txt";

    fire_classifier.init(model_file, trained_file, mean_file, label_file);
    fire_run = false;

    ros::spin();

    return 0;
}
