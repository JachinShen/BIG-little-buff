#include "sudoku/ImageProcess.h"

ImageProcess::ImageProcess()
{
}

ImageProcess::~ImageProcess()
{
}

void ImageProcess::init()
{
    SUDOKU_GRAY_THRES   = 200;
    SUDOKU_AREA_MIN     = 3000;
    SUDOKU_AREA_MAX     = 10000;
    SUDOKU_HW_RATIO_MAX = 20; // 2.0
    SUDOKU_AREA_RATIO   = 6;    // 0.6
}

void ImageProcess::setParam(int index, int data)
{
    switch (index) {
    case 1:
        SUDOKU_GRAY_THRES   = data;
        break;
    case 2:
        SUDOKU_AREA_MIN     = data;
        break;
    case 3:
        SUDOKU_AREA_MAX     = data;
        break;
    case 4:
        SUDOKU_HW_RATIO_MAX = data;
        break;
    case 5:
        SUDOKU_AREA_RATIO   = data;
        break;
    default:
        break;
    }
}

bool ImageProcess::process(Mat& input, Rect& led_rect,
    vector<Rect>& handwrite_rects)
{
    static Mat gray, binary;
    static Mat draw;
    vector<vector<Point> > contours;
    vector<Rect> blocks;

    draw = input.clone();
    cvtColor(input, gray, CV_BGR2GRAY);
    threshold(gray, binary, SUDOKU_GRAY_THRES, 255, THRESH_BINARY);
    findContours(binary.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < SUDOKU_AREA_MIN || bound.area() > SUDOKU_AREA_MAX)
            continue;

        float hw_ratio = (float)bound.height / bound.width;
        if (hw_ratio < 1.0)
            hw_ratio = 1.0 / hw_ratio;
        //cout << "HW ratio: " << hw_ratio << endl;
        if (hw_ratio > (float)SUDOKU_HW_RATIO_MAX / 10)
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
    imshow("sudoku binary", binary);

    if (blocks.size() != 9)
        return false;

    int top_x = blocks[0].x;
    int top_y = blocks[0].y;
    int top_width = blocks[2].x + blocks[2].width - top_x;

    led_rect = Rect(top_x, 0, top_width, top_y);
    copy(blocks.begin(), blocks.end(), back_inserter(handwrite_rects));

    return true;
}
