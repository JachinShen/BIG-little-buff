#include "sudoku/DnnClassifier.h"

static ros::Publisher             mnist_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static DnnClassifier                      mnist_classifier;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void handwriteRectsCallback(const std_msgs::Int16MultiArray& msg)
{
    static Mat img, gray, binary;
    static vector<Mat> mnist_roi;
    //static vector<Rect> mnist_rect;

    img = cv_ptr->image;
    if (img.empty()) {
        cout << "Empty Image" << endl;
        return;
    }
    cvtColor(img, gray, CV_BGR2GRAY);
    threshold(gray, binary, 128, 255, CV_THRESH_BINARY_INV);

    mnist_roi.clear();

    for (uint i = 0; i < msg.data.size(); i += 4) {
        Mat roi = (Mat(binary,
            Rect(msg.data[i] + 10, msg.data[i + 1] + 5,
                msg.data[i + 2] - 20, msg.data[i + 3] - 10))
                       .clone());
        //copyMakeBorder(roi, roi, 15, 15, 15, 15, BORDER_CONSTANT);
        resize(roi, roi, Size(28, 28));
        mnist_roi.push_back(roi);
    }

    mnist_classifier.process(mnist_roi);

    imshow("roi 1", mnist_roi[0]);
    imshow("roi 2", mnist_roi[1]);
    imshow("roi 3", mnist_roi[2]);
    imshow("roi 4", mnist_roi[3]);
    imshow("roi 5", mnist_roi[4]);
    imshow("roi 6", mnist_roi[5]);
    imshow("roi 7", mnist_roi[6]);
    imshow("roi 8", mnist_roi[7]);
    imshow("roi 9", mnist_roi[8]);
    waitKey(1);
}

void ledNumCallback(const std_msgs::Int16MultiArray& msg)
{
    static std_msgs::Int16MultiArray mnist_num_msg;
    mnist_num_msg.data.clear();
    for (uint i = 0; i < msg.data.size(); ++i)
        mnist_num_msg.data.push_back(mnist_classifier.getNumberBlockID(msg.data[i]));
    mnist_num_pub.publish(mnist_num_msg);
}

void mnistParamCallback(const std_msgs::Int16MultiArray& msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mnist");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    mnist_num_pub
        = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_num", 1, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber handwrite_rects_sub
        = nh.subscribe("buff/handwrite_rects", 1, handwriteRectsCallback);
    ros::Subscriber led_num_sub
        = nh.subscribe("buff/led_num", 1, ledNumCallback);
    ros::Subscriber mnist_param_sub
        = nh.subscribe("buff/mnist_param", 1, mnistParamCallback);

    string model_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.prototxt";
    string trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.caffemodel";
    string mean_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/synset_words.txt";
    //string low_trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/lenet_iter_10000.caffemodel";

    mnist_classifier.init(model_file, trained_file, mean_file, label_file);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
