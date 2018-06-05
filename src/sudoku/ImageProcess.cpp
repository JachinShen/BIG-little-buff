#include "sudoku/ImageProcess.h"

namespace sudoku {

ImageProcess::ImageProcess()
{
}

ImageProcess::~ImageProcess()
{
}

bool ImageProcess::process(Mat& input, Rect& led_rect,
    vector<Rect>& handwrite_rects)
{
    static Mat gray, binary;
    static Mat draw;
    static int SUDOKU_GRAY_THRES = 128;
    static int SUDOKU_AREA_MIN = 3000;
    static int SUDOKU_AREA_MAX = 10000;
    static int SUDOKU_HW_RATIO_MIN = 20; // 2.0
    static int SUDOKU_AREA_RATIO = 6; // 0.6
    vector<vector<Point> > contours;
    vector<Rect> blocks;

    createTrackbar("sudoku gray threshold", "sudoku parameters", &SUDOKU_GRAY_THRES, 255);
    createTrackbar("sudoku area min", "sudoku parameters", &SUDOKU_AREA_MIN, 5000);
    createTrackbar("sudoku area max", "sudoku parameters", &SUDOKU_AREA_MAX, 10000);
    createTrackbar("sudoku hw ratio", "sudoku parameters", &SUDOKU_HW_RATIO_MIN, 100);
    createTrackbar("sudoku area ratio", "sudoku parameters", &SUDOKU_AREA_RATIO, 10);

    draw = input.clone();
    cvtColor(input, gray, CV_BGR2GRAY);
    threshold(gray, binary, SUDOKU_GRAY_THRES, 255, THRESH_BINARY);
    imshow("sudoku binary", binary);
    findContours(binary.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < SUDOKU_AREA_MIN || bound.area() > SUDOKU_AREA_MAX)
            continue;
        float hw_ratio = (float)bound.height / bound.width;
        if (hw_ratio < 1.0)
            hw_ratio = 1.0/hw_ratio;
        //cout << "HW ratio: " << hw_ratio << endl;
        if (hw_ratio > (float)SUDOKU_HW_RATIO_MIN / 10)
            continue;
        float area = contourArea(contours[i]);
        //cout << "Area ratio: " << area/bound.area() << endl;
        if (area / bound.area() < (float)SUDOKU_AREA_RATIO / 10)
            continue;
        //cout << "Area: " << bound.area() << endl;
        blocks.push_back(bound);
    }

    sort(blocks.begin(), blocks.end(), compareRect);

    for (uint i = 0; i < blocks.size(); ++i) {
        rectangle(draw, blocks[i], Scalar(255, 0, 255), 2);
    }
    imshow("sudoku draw", draw);

    if (blocks.size() != 9)
        return false;

    int top_x = blocks[0].x;
    int top_y = blocks[0].y;
    int top_width = blocks[2].x + blocks[2].width - top_x;

    led_rect = Rect(top_x, 0, top_width, top_y);
    copy(blocks.begin(), blocks.end(), back_inserter(handwrite_rects));

    return true;
}
}
