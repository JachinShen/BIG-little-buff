#include "sudoku/LedSolver.h"

LedSolver::LedSolver()
{
    //hog = NULL;
}

//void LedSolver::init(const char* file)
void LedSolver::init()
{
    //svm    = SVM::create();
    //svm    = svm->load(file);
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    //hog    = new cv::HOGDescriptor(cvSize(28, 28), cvSize(14, 14), cvSize(7, 7), cvSize(7, 7), 9);

    for (int i = 0; i < 5; ++i)
        results[i] = -1;

    param[RED_THRESHOLD] = 128;

    param[GRAY_THRESHOLD] = 128;
    param[BOUND_AREA_MAX] = 30;
    param[BOUND_AREA_MAX] = 100;
    param[HW_RATIO_MIN] = 130;
    param[HW_RATIO_MAX] = 1000;
    param[HW_RATIO_FOR_DIGIT_ONE] = 250;
    param[ROTATION_DEGREE] = 5;
}

LedSolver::~LedSolver()
{
    //if (hog != NULL)
    //delete hog;
}

void LedSolver::setParam(int index, int value)
{
    if (0 <= index && index < PARAM_SIZE) {
        param[index] = value;
        return;
    } else {
        cout << "Set Param Error!" << endl;
    }
}

void LedSolver::getRed(Mat& led_roi, Mat& led_roi_binary)
{
    static Mat bgr_split[3];
    //static Mat led_roi_red;
    static Mat led_roi_gray;

    cvtColor(led_roi, led_roi_gray, COLOR_BGR2GRAY);
    threshold(led_roi_gray, led_roi_gray, param[GRAY_THRESHOLD], 255, THRESH_BINARY);

    //split(led_roi, bgr_split);
    //led_roi_red = 2 * bgr_split[2] - bgr_split[1] - bgr_split[0];
    //threshold(led_roi_red, led_roi_red, param[RED_THRESHOLD], 255, THRESH_BINARY);

    //led_roi = led_roi_red & led_roi_gray;
    led_roi = led_roi_gray;
    //led_roi_binary = led_roi_gray;

    //erode(led_roi, led_roi_binary, kernel);
    //dilate(led_roi, led_roi_binary, kernel);
    dilate(led_roi, led_roi_binary, kernel);
    //imshow("original binary:", led_roi);
#if DRAW == SHOW_ALL
    imshow("Led Red Binary dilated: ", led_roi_binary);
#endif
}

bool LedSolver::process(Mat& led_roi, Rect& bound_all_rect)
{
    static Mat led_roi_binary;
#if DRAW == SHOW_ALL
    static Mat draw;
#endif
    static vector<vector<Point> > contours;
    vector<Rect> digits;

#if DRAW == SHOW_ALL
    draw = led_roi.clone();
#endif

    getRed(led_roi, led_roi_binary);
    findContours(led_roi_binary.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    digits.clear();
    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        //if (bound.x < 10 || bound.x + bound.width > led_roi.cols - 10
        //|| bound.y < 10 || bound.y + bound.height > led_roi.rows)
        //continue;
        if (bound.area() < param[BOUND_AREA_MIN] || bound.area() > param[BOUND_AREA_MAX])
            continue;
        ROS_INFO_STREAM("Led Area: " << bound.area());
        float hw_ratio = (float)bound.height / bound.width;
        if (hw_ratio < 1.0)
            continue;
        //hw_ratio = 1.0 / hw_ratio;
        //cout << "HW ratio: " << hw_ratio << endl;
        ROS_INFO_STREAM("HW ration: " << hw_ratio);
        if (hw_ratio < param[HW_RATIO_MIN] / 100.0 || hw_ratio > param[HW_RATIO_MAX] / 100.0)
            continue;
        digits.push_back(bound);
    }

    sort(digits.begin(), digits.end(), compareRectX);

    if (digits.size() < 5 && !digits.empty()) {
        int curSize = digits.size(), maxwidth = digits.front().width, maxheight = digits.front().height;
        for (int i = 0; i < curSize - 1 && digits.size() < 5; i++) {
            maxwidth = max(maxwidth, digits[i].width);
            maxheight = max(maxheight, digits[i].height);
            if (digits[i].x + digits[i].width * 2 >= digits[i + 1].x) {
                continue;
            } else {
                int newx = (digits[i + 1].x + digits[i].x) / 2, newy = (digits[i].y + digits[i + 1].y) / 2,
                    newwidth = max(digits[i].width, digits[i + 1].width), newheight = max(digits[i].height, digits[i + 1].height);
                if ((float)digits[i].height / digits[i].width > param[HW_RATIO_FOR_DIGIT_ONE] / 100.0) {
                    newx = (digits[i + 1].x + digits[i].x) / 2 + digits[i].width - digits[i + 1].width;
                    newx = (digits[i].x + digits[i].width + digits[i + 1].x + digits[i + 1].width) / 2 - digits[i + 1].width;
                }
                if ((float)digits[i + 1].height / digits[i + 1].width > param[HW_RATIO_FOR_DIGIT_ONE] / 100.0) {
                    newx = (digits[i].x + digits[i + 1].x) / 2;
                }
                Rect bound = Rect(newx, newy, newwidth, newheight);
                ROS_INFO_STREAM("Led Guess Rect: " << bound);
                digits.push_back(bound);
            }
        }
        sort(digits.begin(), digits.end(), compareRectX);
        if (digits.size() < 5 && !digits.empty()) {
            bool left = true, right = true;
            while (left && digits.size() < 5) {
                int newx = digits.front().x - maxwidth * 4 / 3 - 3, newy = digits.front().y, newwidth = maxwidth + 3, newheight = digits.front().height;
                Rect bound = Rect(newx, newy, newwidth, newheight);
                Mat roi = (led_roi_binary)(bound).clone();
                if (predictCross(roi) == -1)
                    left = false;
                if (left)
                    digits.insert(digits.begin(), bound);
            }
            while (right && digits.size() < 5) {
                int newx = digits.back().x + maxwidth * 4 / 3 - 3, newy = digits.back().y, newwidth = maxwidth + 3, newheight = digits.back().height;
                Rect bound = Rect(newx, newy, newwidth, newheight);
                Mat roi = (led_roi_binary)(bound).clone();
                if (predictCross(roi) == -1)
                    right = false;
                if (right)
                    digits.push_back(bound);
            }
        }
        sort(digits.begin(), digits.end(), compareRectX);
    }

    if (digits.size() != 5) {
        cout << "digit size error, current size:" << digits.size() << endl;
        ROS_INFO("Clear vector");
        digits.clear(); // add for secure, otherwise munmap_chunk() error will be raised if there are too many elements in the vector (about 30)
        return false;
    }

    for (uint i = 0; i < digits.size(); ++i) {
        float hw_ratio = (float)digits[i].height / digits[i].width;
        if (hw_ratio < 1.0)
            hw_ratio = 1.0 / hw_ratio;
        Mat roi = (led_roi_binary)(digits[i]).clone();
        if (hw_ratio > param[HW_RATIO_FOR_DIGIT_ONE] / 100.0) {
            int segment1 = scanSegmentY(roi, roi.rows / 3, 0, roi.cols);
            int segment2 = scanSegmentY(roi, roi.rows * 2 / 3, 0, roi.cols);
            if (segment1 > 1 && segment2 > 1)
                results[i] = 1;
            else
                results[i] = -1;
            continue;
        }

        Point center = Point(digits[i].width / 2, digits[i].height / 2);
        Mat M2 = getRotationMatrix2D(center, param[ROTATION_DEGREE], 1);
        warpAffine(roi, roi, M2, roi.size(), 1, 0, 0);
        results[i] = predictCross(roi);
    }

    bound_all_rect = Rect(digits[0].tl() - Point(10, 10),
        digits[4].br() + Point(10, 10));

#if DRAW == SHOW_ALL
    for (uint i = 0; i < digits.size(); ++i) {
        rectangle(draw, digits[i], Scalar(255, 0, 0), 2);
    }
    imshow("draw", draw);
    //imshow("Cross Led Red Binary: ", led_roi);
#endif
    return true;
}

int LedSolver::scanSegmentX(Mat& roi, int line_x, int y_begin, int y_end)
{
    int hit_ctr = 0;
    for (int i = y_begin; i < y_end; ++i) {
        uchar* pixel = roi.ptr<uchar>(i) + line_x;
        if (*pixel == 255 || *pixel == 128) {
            *pixel = 128;
            ++hit_ctr;
        }
    }
    return hit_ctr;
}

int LedSolver::scanSegmentY(Mat& roi, int line_y, int x_begin, int x_end)
{
    uchar* pixel = roi.ptr<uchar>(line_y) + x_begin;
    int hit_ctr = 0;
    for (int i = x_begin; i < x_end; ++i, ++pixel) {
        if (*pixel == 255 || *pixel == 128) {
            *pixel = 128;
            ++hit_ctr;
        }
    }
    return hit_ctr;
}

int LedSolver::predictCross(Mat& roi)
{
#define SEGMENT_A 0x01
#define SEGMENT_B 0x02
#define SEGMENT_C 0x04
#define SEGMENT_D 0x08
#define SEGMENT_E 0x10
#define SEGMENT_F 0x20
#define SEGMENT_G 0x40
#define SEGMENT_THRES 0

    int mid_x = roi.cols / 2;
    int one_sixth_x = roi.cols / 6;
    int one_third_y = roi.rows / 3;
    int two_thirds_y = roi.rows * 2 / 3;
    int one_twelvth_y = roi.rows / 12;
    int segment = 0x00;
    int segment_hit[7] = { 0 };
    int supporta[7] = { 0 }, supportb[7] = { 0 };

    segment_hit[0] = (scanSegmentY(roi, one_third_y, 0, mid_x));
    supporta[0] = scanSegmentY(roi, one_third_y - one_twelvth_y, 0, mid_x);
    supportb[0] = scanSegmentY(roi, one_third_y + one_twelvth_y, 0, mid_x);

    segment_hit[1] = (scanSegmentY(roi, one_third_y, mid_x, roi.cols));
    supporta[1] = scanSegmentY(roi, one_third_y - one_twelvth_y, mid_x, roi.cols);
    supportb[1] = scanSegmentY(roi, one_third_y + one_twelvth_y, mid_x, roi.cols);

    segment_hit[2] = (scanSegmentY(roi, two_thirds_y, 0, mid_x));
    supporta[2] = scanSegmentY(roi, two_thirds_y - one_twelvth_y, 0, mid_x);
    supportb[2] = scanSegmentY(roi, two_thirds_y + one_twelvth_y, 0, mid_x);

    segment_hit[3] = (scanSegmentY(roi, two_thirds_y, mid_x, roi.cols));
    supporta[3] = scanSegmentY(roi, two_thirds_y - one_twelvth_y, mid_x, roi.cols);
    supportb[3] = scanSegmentY(roi, two_thirds_y + one_twelvth_y, mid_x, roi.cols);

    segment_hit[4] = (scanSegmentX(roi, mid_x, 0, one_third_y));
    supporta[4] = scanSegmentX(roi, mid_x - one_sixth_x, 0, one_third_y);
    supportb[4] = scanSegmentX(roi, mid_x + one_sixth_x, 0, one_third_y);

    segment_hit[5] = (scanSegmentX(roi, mid_x, one_third_y, two_thirds_y));
    supporta[5] = scanSegmentX(roi, mid_x - one_sixth_x, one_third_y, two_thirds_y);
    supportb[5] = scanSegmentX(roi, mid_x + one_sixth_x, one_third_y, two_thirds_y);

    segment_hit[6] = (scanSegmentX(roi, mid_x, two_thirds_y, roi.rows));
    supporta[6] = scanSegmentX(roi, mid_x - one_sixth_x, two_thirds_y, roi.rows);
    supportb[6] = scanSegmentX(roi, mid_x + one_sixth_x, two_thirds_y, roi.rows);

    //for (int i=0; i<7; ++i) {
    //cout << "segment " << i << " hit: " <<supporta[i]<<" "<< segment_hit[i] << " "<<supportb[i] << endl;}
    if (segment_hit[0] > SEGMENT_THRES && supporta[0] > SEGMENT_THRES && supportb[0] > SEGMENT_THRES)
        segment |= SEGMENT_F;
    if (segment_hit[1] > SEGMENT_THRES && supporta[1] > SEGMENT_THRES && supportb[1] > SEGMENT_THRES)
        segment |= SEGMENT_B;
    if (segment_hit[2] > SEGMENT_THRES && supporta[2] > SEGMENT_THRES && supportb[2] > SEGMENT_THRES)
        segment |= SEGMENT_E;
    if (segment_hit[3] > SEGMENT_THRES && supporta[3] > SEGMENT_THRES && supportb[3] > SEGMENT_THRES)
        segment |= SEGMENT_C;
    if (segment_hit[4] > SEGMENT_THRES && supporta[4] > SEGMENT_THRES && supportb[4] > SEGMENT_THRES)
        segment |= SEGMENT_A;
    if (segment_hit[5] > SEGMENT_THRES && supporta[5] > SEGMENT_THRES && supportb[5] > SEGMENT_THRES)
        segment |= SEGMENT_G;
    if (segment_hit[6] > SEGMENT_THRES && supporta[6] > SEGMENT_THRES && supportb[6] > SEGMENT_THRES)
        segment |= SEGMENT_D;

    //cout << "Segment: " << segment << endl;

    switch (segment) {
    case 0x3f:
        return 0;
    case 0x06:
        return 1;
    case 0x5b:
        return 2;
    case 0x4f:
        return 3;
    case 0x66:
        return 4;
    case 0x6d:
        return 5;
    case 0x7d:
        return 6;
    case 0x07:
        return 7;
    case 0x7f:
        return 8;
    case 0x6f:
        return 9;
    default:
        return -1;
    }
}

//int LedSolver::predictSVM(Mat& roi)
//{
//vector<float> descriptors;
////dilate(roi, roi, kernel);
////erode(roi, roi, kernel);
//resize(roi, roi, Size(20, 20));
//Mat inner = Mat::ones(28, 28, CV_8UC1) + 254;
//roi.copyTo(inner(Rect(4, 4, 20, 20)));
////imshow("inner", inner);
//hog->compute(inner, descriptors, Size(1, 1), Size(0, 0));

//Mat SVMPredictMat = Mat(1, (int)descriptors.size(), CV_32FC1);
//memcpy(SVMPredictMat.data, descriptors.data(), descriptors.size() * sizeof(float));
//return (svm->predict(SVMPredictMat));
//}

int LedSolver::getResult(int index)
{
    if (index >= 5 || index < 0)
        return -1;
    return results[index];
}

bool LedSolver::confirmLed()
{
    for (int i = 0; i < 5; ++i) {
        if (results[i] == -1) {
            return false;
        }
    }
    return true;
}
