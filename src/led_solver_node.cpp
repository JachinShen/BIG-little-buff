/*********************************************
Sudoku Node: find the nine blocks
Copyright 2018 JachinShen

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************/

#include "sudoku/LedSolver.h"
#include <pthread.h>

static ros::Publisher led_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("Image Call!");
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

void ledCallback(const std_msgs::Int16MultiArray& msg)
{
    static sudoku::LedSolver led_solver("./src/buff/svm/SVM_DATA_NUM.xml");
    static Rect led_rect;
    static Mat led_roi;
    static Mat img;

    led_rect = Rect(msg.data[0], msg.data[1],
            msg.data[2], msg.data[3]);
    img = cv_ptr->image;
    if (img.empty()) {
        cout << "Empty Image" << endl;
        return;
    }

    led_roi = Mat(img, led_rect);
    led_solver.process(led_roi);
    imshow("led img", led_roi);
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "led");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    led_num_pub 
        = nh.advertise<std_msgs::Int16MultiArray>("buff/led_num", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub 
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber led_rect_sub 
        = nh.subscribe("buff/led_rect", 1, ledCallback);

    ros::spin();

    ROS_INFO("End!");
    return 0;
}
