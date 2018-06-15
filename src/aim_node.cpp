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

#include "kcftracker.hpp"
#include "Headers.h"

void aimNumCallback(const std_msgs::Int16MultiArray& msg)
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "aim");
    ROS_INFO("Start!");
    ros::NodeHandle nh;

    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::Subscriber aim_sub = nh.subscribe("buff/aim_num", 1, aimNumCallback);

    ros::spin();

    ROS_INFO("Finish!");
    return 0;
}
