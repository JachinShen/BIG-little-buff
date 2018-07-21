//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include "Headers.h"
#include "sudoku/BlockSplit.h"
#include "sudoku/DnnClassifier.h"
#include "sudoku/LedSolver.h"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <fstream>
#include <sstream> // for converting the command line parameter to integer

#include <sys/time.h>
timeval timestart;
static Mat img, gray;
static ros::Publisher sudoku_rect_pub;
static ros::Publisher led_num_pub;
static ros::Publisher mnist_num_pub;
static ros::Publisher             fire_num_pub;
static BlockSplit block_split;
static bool sudoku_run;
static LedSolver led_solver;
static bool led_run;
static Rect led_rect;
static DnnClassifier mnist_classifier;
static bool mnist_run;
static Rect sudoku_rect;
static DnnClassifier              fire_classifier;
static bool                       fire_run;

using namespace cv;

void initImageProcess()
{
    block_split.init();
    sudoku_run = true;
    led_solver.init();
    led_run = true;

#if PLATFORM == PC
    string model_file   = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.prototxt";
    string trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mnist_model.caffemodel";
    string mean_file    = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file   = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/synset_words.txt";
    string fire_model_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.prototxt";
    string fire_trained_file = "/home/jachinshen/Projects/lunar_ws/src/buff/caffemodels/fire_model.caffemodel";
#elif PLATFORM == MANIFOLD
    string model_file   = "/home/ubuntu/lunar_ws/src/buff/caffemodels/mnist_model.prototxt";
    string trained_file = "/home/ubuntu/lunar_ws/src/buff/caffemodels/mnist_model.caffemodel";
    string mean_file    = "/home/ubuntu/lunar_ws/src/buff/caffemodels/mean.binaryproto";
    string label_file   = "/home/ubuntu/lunar_ws/src/buff/caffemodels/synset_words.txt";
    string fire_model_file = "/home/ubuntu/lunar_ws/src/buff/caffemodels/fire_model.prototxt";
    string fire_trained_file = "/home/ubuntu/lunar_ws/src/buff/caffemodels/fire_model.caffemodel";
#endif

    mnist_classifier.init(model_file, trained_file, mean_file, label_file);
    mnist_run = true;
    fire_classifier.init(fire_model_file, fire_trained_file, mean_file, label_file);
    fire_run = false;
}

void sudokuProcess()
{
    if (!sudoku_run)
        return;

    static Mat binary;

    if (block_split.process(gray, led_rect, sudoku_rect)) {
        ROS_INFO_STREAM("Led Rect: " << led_rect);
        ROS_INFO_STREAM("Sudoku Rect: " << sudoku_rect);
        std_msgs::Int16MultiArray sudoku_rect_msg;
        sudoku_rect_msg.data.push_back(1);
        sudoku_rect_pub.publish(sudoku_rect_msg);
        sudoku_run = false;
    } else {
        ROS_INFO("No Sudoku Found!");
    }
}

void sudokuParamCallback(const std_msgs::Int16MultiArray& msg)
{
    block_split.setParam(msg.data[0], msg.data[1]);
}

void sudokuCtrCallback(const std_msgs::Bool& msg)
{
    sudoku_run = msg.data;
    ROS_INFO_STREAM("Get Sudoku Ctr: " << sudoku_run);
}

void ledProcess()
{
    if (!led_run)
        return;

    if (led_rect.area() == 0)
        return;

    static Mat led_roi;

    led_roi = Mat(img, led_rect);
    Rect bound_all_rect;
    static int led_miss_ctr = 0;
    static std_msgs::Int16MultiArray led_num_msg;
    if (led_solver.process(led_roi, bound_all_rect)) {
        led_miss_ctr = 0;
        //ROS_INFO_STREAM("Bound All Rect: " << bound_all_rect);
        led_num_msg.data.clear();
        for (uint i = 0; i < 5; ++i)
            led_num_msg.data.push_back(led_solver.getResult(i));
        led_num_pub.publish(led_num_msg);
        if (led_solver.confirmLed()) {
            led_run = false;
        }
    } else {
        ++led_miss_ctr;
        if (led_miss_ctr >= 5) {
            led_miss_ctr = 0;
            led_num_msg.data.clear();
            for (uint i = 0; i < 5; ++i)
                led_num_msg.data.push_back(led_solver.getResult(i));
            led_num_pub.publish(led_num_msg);
            //led_run = false;
        }
    }
}

void ledParamCallback(const std_msgs::Int16MultiArray& msg)
{
    //ROS_INFO_STREAM("Get Param" << msg.data[0]);
    led_solver.setParam(msg.data[0], msg.data[1]);
}

void ledCtrCallback(const std_msgs::Bool& msg)
{
    led_run = msg.data;
    ledProcess();
}

void mnistProcess()
{
    if (!mnist_run) {
        //ROS_INFO("Ignore Mnist!");
        return;
    }

    if (sudoku_rect.area() == 0)
        return;

    static Mat gray_roi, binary;
    static vector<Mat> mnist_roi;
    vector<vector<Point> > contours;
    vector<Rect> mnist_rect;

    //img_roi = img(sudoku_rect);
    //cvtColor(img_roi, gray, CV_BGR2GRAY);
    gray_roi = gray(sudoku_rect);
    threshold(gray_roi, binary, 70, 255, CV_THRESH_BINARY);

    findContours(binary.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    mnist_rect.clear();
    int MAX_AREA = sudoku_rect.area() / 9 * 2;
    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 500 || bound.area() > MAX_AREA)
            continue;
        mnist_rect.push_back(bound);
        if (mnist_rect.size() == 9) {
            break;
        }
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

    //TODO: Try to process the 9 blocks as a batch.
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
    //fire_num_pub.publish(mnist_num_msg);

#if DRAW == SHOW_ALL
    for (uint i = 0; i < mnist_rect.size(); ++i) {
        rectangle(gray_roi, mnist_rect[i], Scalar(255, 0, 0), 2);
    }
    imshow("Mnist", gray_roi);
#endif

}

void mnistParamCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO_STREAM("Mnist Param: " << msg.data[0]);
}

void mnistCtrCallback(const std_msgs::Bool& msg)
{
    mnist_run = msg.data;
    mnistProcess();
}

void fireProcess()
{
    if (!fire_run) {
        //ROS_INFO("Ignore Fire!");
        return;
    }

    if (sudoku_rect.area() == 0)
        return;

    static Mat binary, sudoku_roi, fire_binary;
    static vector<Mat> fire_roi;
    vector<Rect>               fire_rect;
    vector<vector<Point> > contours;

    sudoku_roi = gray(sudoku_rect);
    //cvtColor(sudoku_roi, gray, CV_BGR2GRAY);
    threshold(sudoku_roi, binary, 60, 255, CV_THRESH_BINARY);
    fire_binary = sudoku_roi;
    //threshold(sudoku_roi, fire_binary, 140, 255, CV_THRESH_TRUNC);

    findContours(binary.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int MAX_AREA = sudoku_rect.area() / 9;
    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 200 || bound.area() > MAX_AREA)
            continue;
        fire_rect.push_back(bound);
    }
    if (fire_rect.size() != 9) {
        return;
    }
    sort(fire_rect.begin(), fire_rect.end(), compareRect);

    //static char window_name[15] = "Fire roi x";
    fire_roi.clear();
    for (uint i=0; i<fire_rect.size(); ++i) {
        Mat roi = (fire_binary(fire_rect[i]));
        int left_right_gap = (roi.rows - roi.cols) / 2 + 3;
        if (left_right_gap > 0)
            copyMakeBorder(roi, roi, 3, 3, left_right_gap, left_right_gap, BORDER_CONSTANT);
        else
            copyMakeBorder(roi, roi, 3, 3, 3, 3, BORDER_CONSTANT);
        resize(roi, roi, Size(28, 28));
        //window_name[10] = i + '0';
        //imshow(window_name, roi);
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
    imshow("Fire recog", fire_binary);
#endif
}

void fireParamCallback(const std_msgs::Int16MultiArray& msg)
{
    ROS_INFO_STREAM("Fire Param: " << msg.data[0]);
}

void fireCtrCallback(const std_msgs::Bool& msg)
{
    fire_run = msg.data;
    ROS_INFO("Fire Ctr Call");
    fireProcess();
}

string getFilename()
{
#if PLATFORM == PC
    fstream fin("/home/jachinshen/Projects/lunar_ws/src/buff/out.txt");
#elif PLATFORM == MANIFOLD
    fstream fin("/home/ubuntu/lunar_ws/src/buff/out.txt");
#endif
    int video_cnt;
    fin>>video_cnt;
    stringstream video_ss;
    video_ss<<video_cnt;

    fin.close();
    std::string file_name=video_ss.str();
#if PLATFORM == PC
    fstream fout("/home/jachinshen/Projects/lunar_ws/src/buff/out.txt");
#elif PLATFORM == MANIFOLD
    fstream fout("/home/ubuntu/lunar_ws/src/buff/out.txt");
#endif
    video_cnt++;
    fout<<video_cnt;
    fout.close();

#if PLATFORM == PC
    file_name = "/home/jachinshen/record"+file_name+".avi";
#elif PLATFORM == MANIFOLD
    file_name = "/home/ubuntu/record"+file_name+".avi";
#endif
    return file_name;
}



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
    //image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub = it.advertise("camera/image", 1, true);
    //image_transport::Publisher gray_pub = it.advertise("camera/gray", 1, true);
    sudoku_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/sudoku_rect", 1, true);
    led_num_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_num", 1);
    mnist_num_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_num", 1, true);
    fire_num_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/fire_num", 1, true);
    ros::Subscriber sudoku_param_sub = nh.subscribe("buff/sudoku_param", 1, sudokuParamCallback);
    ros::Subscriber sudoku_ctr_sub = nh.subscribe("buff/sudoku_ctr", 1, sudokuCtrCallback);
    ros::Subscriber led_param_sub = nh.subscribe("buff/led_param", 1, ledParamCallback);
    ros::Subscriber led_ctr_sub = nh.subscribe("buff/led_ctr", 1, ledCtrCallback);
    ros::Subscriber mnist_param_sub = nh.subscribe("buff/mnist_param", 1, mnistParamCallback);
    ros::Subscriber mnist_ctr_sub = nh.subscribe("buff/mnist_ctr", 1, mnistCtrCallback);
    ros::Subscriber fire_param_sub = nh.subscribe("buff/fire_param", 1, fireParamCallback);
    ros::Subscriber fire_ctr_sub = nh.subscribe("buff/fire_ctr", 1, fireCtrCallback);

    //cv::Mat frame, gray;
    //sensor_msgs::ImagePtr msg, gray_msg;
    ros::Rate loop_rate(30);
    enum { VIDEO_FILE,
        VIDEO_CAMERA } video_type;
    for (int i=0; i<30; ++i) {
        ros::spinOnce();
    }
    while (nh.ok()) {
        cv::VideoCapture cap;
#if RECORD == RECORD_ON
        cv::VideoWriter g_writer;
        g_writer.open(getFilename(), CV_FOURCC('P','I','M','1'),30,cv::Size(640,480));
#endif
        if ('0' <= argv[1][0] && argv[1][0] <= '9') {
            video_type = VIDEO_CAMERA;
            cap.open(argv[1][0] - '0');
        } else {
            video_type = VIDEO_FILE;
            cap.open(argv[1]);
            cap.set(CAP_PROP_EXPOSURE, 160);
        }
        if (!cap.isOpened()) {
            ROS_INFO("can not opencv video device");
            return 1;
        }
        ROS_INFO("open successfully");

        initImageProcess();

#if OPENMP_SWITCH == OPENMP_STOP
        while (cap.read(img) && ros::ok()) {
            ros::spinOnce();
            // Check if grabbed frame is actually full with some content
            if (!img.empty()) {
                cv::resize(img, img, Size(640, 480));
                ROS_INFO("Process Start");
                cv::cvtColor(img, gray, CV_BGR2GRAY);
                sudokuProcess();
                ledProcess();
                mnistProcess();
                fireProcess();
                ROS_INFO("Process End");
                //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                //gray_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
                //pub.publish(msg);
                //gray_pub.publish(gray_msg);
                //ROS_INFO("Publish");

#if RECORD == RECORD_ON
                g_writer.write(img);
#endif
                imshow("src", img);
                if (video_type == VIDEO_CAMERA)
                    waitKey(1);
                else if (video_type == VIDEO_FILE)
                    waitKey(0);
            }
        }
#elif OPENMP_SWITCH == OPENMP_RUN
        cv::Mat frame1, frame2;
        cap.read(frame2);
        while(ros::ok()) {
            ros::spinOnce();
# pragma omp parallel sections
            {
# pragma omp section
                {
                    cap.read(frame1);
#if RECORD == RECORD_ON
                    g_writer.write(frame1);
#endif
                }
# pragma omp section
                {
                    ROS_INFO("Process Start");
                    img = frame2;
                    cv::cvtColor(img, gray, CV_BGR2GRAY);
                    sudokuProcess();
                    ledProcess();
                    //mnistProcess();
                    fireProcess();
                    ROS_INFO("Process End");
                }
            }
# pragma omp barrier
            imshow("frame", frame1);
            waitKey(1);
            /************************/
            ros::spinOnce();
# pragma omp parallel sections
            {
# pragma omp section
                {
                    cap.read(frame2);
#if RECORD == RECORD_ON
                    g_writer.write(frame2);
#endif
                }
# pragma omp section
                {
                    ROS_INFO("Process Start");
                    img = frame1;
                    cv::cvtColor(img, gray, CV_BGR2GRAY);
                    sudokuProcess();
                    ledProcess();
                    //mnistProcess();
                    fireProcess();
                    ROS_INFO("Process End");
                }
            }
# pragma omp barrier
            imshow("frame", frame1);
            waitKey(1);
        }
#endif

//#if RECORD == RECORD_ON
        //g_writer.close();
//#endif
    }
}
