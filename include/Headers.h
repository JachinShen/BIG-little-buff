#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
using namespace cv;

#include <algorithm>
#include <errno.h>
#include <fcntl.h> /* File control definitions */
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

using std::copy;
using std::cout;
using std::endl;
using std::sort;
using std::string;
using std::vector;

bool compareRect(const Rect& r1, const Rect& r2);
