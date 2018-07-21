#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>
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
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>

#define PC           0
#define MANIFOLD     1

//#define VIDEO_FILE   0
//#define VIDEO_CAMERA 1

#define NO_SHOW      0
#define SHOW_ALL     1

#define RECORD_OFF   0
#define RECORD_ON    1

#define OPENMP_STOP  0
#define OPENMP_RUN   1

#define PI 3.14159265358979323

#if defined __arm__
#   define PLATFORM MANIFOLD
#else
#   define PLATFORM PC
#endif

#define DRAW          SHOW_ALL
#define RECORD        RECORD_ON
#define OPENMP_SWITCH OPENMP_STOP


using std::copy;
using std::cout;
using std::endl;
using std::sort;
using std::string;
using std::vector;

bool compareRect(const Rect& r1, const Rect& r2);
bool compareRectX(const Rect& r1, const Rect& r2);
