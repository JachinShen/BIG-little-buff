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

static ros::Publisher led_num_pub;
static cv_bridge::CvImageConstPtr cv_ptr;
static sudoku::LedSolver led_solver;

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

void ledRectCallback(const std_msgs::Int16MultiArray& msg)
{
    static std_msgs::Int16MultiArray led_num_msg;
    static Mat img;
    static Mat led_roi;
    static Rect led_rect;

    led_rect = Rect(msg.data[0], msg.data[1],
            msg.data[2], msg.data[3]);

    img = cv_ptr->image;
    if (img.empty()) {
        cout << "Empty Image" << endl;
        return;
    }

    led_roi = Mat(img, led_rect);
    if (led_solver.process(led_roi))
    {
        led_num_msg.data.clear();
        for (uint i=0; i<5; ++i)
            led_num_msg.data.push_back(led_solver.getResult(i));
        led_num_pub.publish(led_num_msg);
    }

    waitKey(1);
}

void ledParamCallback(const std_msgs::Int16MultiArray& msg)
{
    led_solver.setRedThreshold(msg.data[0]);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "led");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    led_num_pub 
        = nh.advertise<std_msgs::Int16MultiArray>("buff/led_num", 1);

    led_solver.init("./src/buff/svm/SVM_DATA_NUM.xml");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub 
        = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber led_rect_sub 
        = nh.subscribe("buff/led_rect", 1, ledRectCallback);
    ros::Subscriber led_param_sub
        = nh.subscribe("buff/led_param", 1, ledParamCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
