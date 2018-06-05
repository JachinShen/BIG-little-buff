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
//#include "uart_node/sudoku_mode.h"

static ros::Publisher led_rect_pub;
static ros::Publisher handwrite_rects_pub;
//static ros::Rate loop_rate(30);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("Image Call!");
    static sudoku::ImageProcess image_process;
    static Mat img, gray, binary;
    Rect led_rect;
    vector<Rect> handwrite_rects;
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty())
        return;
    if (image_process.process(img, led_rect, handwrite_rects)) {
        cout << "Led Rect: " << led_rect << endl;
        std_msgs::Int16MultiArray led_msg;
        led_msg.data.push_back(led_rect.x);
        led_msg.data.push_back(led_rect.y);
        led_msg.data.push_back(led_rect.width);
        led_msg.data.push_back(led_rect.height);
        led_rect_pub.publish(led_msg);

        cout << "Hand Write Rects: " << handwrite_rects[0] << endl;
        std_msgs::Int16MultiArray handwrite_msg;
        for (uint i = 0; i < handwrite_rects.size(); ++i) {
            handwrite_msg.data.push_back(handwrite_rects[i].x);
            handwrite_msg.data.push_back(handwrite_rects[i].y);
            handwrite_msg.data.push_back(handwrite_rects[i].width);
            handwrite_msg.data.push_back(handwrite_rects[i].height);
        }
        handwrite_rects_pub.publish(handwrite_msg);

        imshow("sudoku img", img);
    } else {
        ROS_INFO("No sudoku Found!");
    }
        waitKey(0);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sudoku");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    led_rect_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/led_rect", 4);
    handwrite_rects_pub = nh.advertise<std_msgs::Int16MultiArray>("buff/handwrite_rects", 4*9);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    cv::namedWindow("sudoku parameters");

    ros::spin();

    ROS_INFO("End!");
    return 0;
}
