#include "sudoku/DnnClassifier.h"

static ros::Publisher             fire_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static DnnClassifier              fire_classifier;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void fireRectsCallback(const std_msgs::Int16MultiArray& msg)
{
    static Mat img, gray, binary;
    static vector<Mat> fire_roi;
    //static vector<Rect> fire_rect;

    img = cv_ptr->image;
    if (img.empty()) {
        cout << "Empty Image" << endl;
        return;
    }
    cvtColor(img, gray, CV_BGR2GRAY);
    threshold(gray, binary, 180, 255, CV_THRESH_BINARY);

    fire_roi.clear();

    for (uint i = 0; i < msg.data.size(); i += 4) {
        Mat roi = (Mat(binary,
            Rect(msg.data[i], msg.data[i + 1],
                msg.data[i + 2], msg.data[i + 3]))
                       .clone());
        int left_right_gap = (roi.rows - roi.cols) / 2 + 5;
        copyMakeBorder(roi, roi, 5, 5, left_right_gap, left_right_gap, BORDER_CONSTANT);
        resize(roi, roi, Size(28, 28));
        fire_roi.push_back(roi);
    }

    fire_classifier.process(fire_roi);

    imshow("roi 1", fire_roi[0]);
    imshow("roi 2", fire_roi[1]);
    imshow("roi 3", fire_roi[2]);
    imshow("roi 4", fire_roi[3]);
    imshow("roi 5", fire_roi[4]);
    imshow("roi 6", fire_roi[5]);
    imshow("roi 7", fire_roi[6]);
    imshow("roi 8", fire_roi[7]);
    imshow("roi 9", fire_roi[8]);
    waitKey(1);
}

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    static std_msgs::Int16MultiArray fire_num_msg;
    fire_num_msg.data.clear();
    for (uint i = 0; i < msg.data.size(); ++i)
        fire_num_msg.data.push_back(fire_classifier.getNumberBlockID(msg.data[i]));
    fire_num_pub.publish(fire_num_msg);
}

void fireParamCallback(const std_msgs::Int16MultiArray& msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "fire");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    fire_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/fire_num", 1, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber fire_rects_sub
        = nh.subscribe("buff/fire_rects", 1, fireRectsCallback);
    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber fire_param_sub
        = nh.subscribe("buff/fire_param", 1, fireParamCallback);

    string model_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.prototxt";
    string trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.caffemodel";
    string mean_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/synset_words.txt";
    //string low_trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/lenet_iter_10000.caffemodel";

    fire_classifier.init(model_file, trained_file, mean_file, label_file);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
