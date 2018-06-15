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

#include "sudoku/BlockSplit.h"

static ros::Publisher led_rect_pub;
static ros::Publisher mnist_rects_pub;
static ros::Publisher fire_rects_pub;
static BlockSplit block_split;

void sudokuParamCallback(const std_msgs::Int16MultiArray& msg)
{
    block_split.setParam(msg.data[0], msg.data[1]);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static Mat img, gray, binary;
    Rect led_rect;
    Rect sudoku_rect;
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty())
        return;
    if (block_split.process(img, led_rect, sudoku_rect)) {
        cout << "Led Rect: " << led_rect << endl;
        std_msgs::Int16MultiArray led_rect_msg;
        led_rect_msg.data.push_back(led_rect.x);
        led_rect_msg.data.push_back(led_rect.y);
        led_rect_msg.data.push_back(led_rect.width);
        led_rect_msg.data.push_back(led_rect.height);
        led_rect_pub.publish(led_rect_msg);

        cout << "Sudoku Rects: " << endl;
        std_msgs::Int16MultiArray sudoku_rect_msg;
        sudoku_rect_msg.data.push_back(sudoku_rect.x);
        sudoku_rect_msg.data.push_back(sudoku_rect.y);
        sudoku_rect_msg.data.push_back(sudoku_rect.width);
        sudoku_rect_msg.data.push_back(sudoku_rect.height);
        fire_rects_pub.publish(sudoku_rect_msg);
        mnist_rects_pub.publish(sudoku_rect_msg);
    } else {
        ROS_INFO("No sudoku Found!");
    }
    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sudoku");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    block_split.init();

    led_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_rect", 1);
    mnist_rects_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/mnist_rects", 1);
    fire_rects_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/fire_rects", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber sudoku_param_sub = nh.subscribe("buff/sudoku_param", 1, sudokuParamCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
