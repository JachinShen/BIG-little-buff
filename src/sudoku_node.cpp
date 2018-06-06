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

#include "sudoku/ImageProcess.h"

static ros::Publisher led_rect_pub;
static ros::Publisher handwrite_rects_pub;
static ImageProcess image_process;

void sudokuParamCallback(const std_msgs::Int16MultiArray& msg)
{
    image_process.setParam(msg.data[0], msg.data[1]);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static Mat img, gray, binary;
    Rect led_rect;
    vector<Rect> handwrite_rects;
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty())
        return;
    if (image_process.process(img, led_rect, handwrite_rects)) {
        cout << "Led Rect: " << led_rect << endl;
        std_msgs::Int16MultiArray led_rect_msg;
        led_rect_msg.data.push_back(led_rect.x);
        led_rect_msg.data.push_back(led_rect.y);
        led_rect_msg.data.push_back(led_rect.width);
        led_rect_msg.data.push_back(led_rect.height);
        led_rect_pub.publish(led_rect_msg);

        cout << "Hand Write Rects: " << handwrite_rects[0] << endl;
        std_msgs::Int16MultiArray handwrite_rects_msg;
        for (uint i = 0; i < handwrite_rects.size(); ++i) {
            handwrite_rects_msg.data.push_back(handwrite_rects[i].x);
            handwrite_rects_msg.data.push_back(handwrite_rects[i].y);
            handwrite_rects_msg.data.push_back(handwrite_rects[i].width);
            handwrite_rects_msg.data.push_back(handwrite_rects[i].height);
        }
        handwrite_rects_pub.publish(handwrite_rects_msg);

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

    image_process.init();

    led_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_rect", 1);
    handwrite_rects_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/handwrite_rects", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber sudoku_param_sub = nh.subscribe("buff/sudoku_param", 1, sudokuParamCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
