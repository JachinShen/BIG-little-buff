#include "sudoku/BlockSplit.h"

BlockSplit::BlockSplit()
{
}

BlockSplit::~BlockSplit()
{
}

void BlockSplit::init()
{
    SUDOKU_GRAY_THRES   = 200;
    SUDOKU_AREA_MIN     = 3000;
    SUDOKU_AREA_MAX     = 10000;
    SUDOKU_HW_RATIO_MAX = 20; // 2.0
    SUDOKU_AREA_RATIO   = 6;    // 0.6
}

void BlockSplit::setParam(int index, int data)
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

bool BlockSplit::processMnist(Mat& input, Rect& led_rect,
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

bool BlockSplit::processFire(Mat& input, Rect& led_rect,
    vector<Rect>& handwrite_rects)
{
    static Mat gray, yellow, binary;
    static Mat draw;
    vector<vector<Point> > contours;
    vector<Rect> side_blocks;
    vector<Rect> blocks;

    draw = input.clone();
    cvtColor(input, gray, CV_BGR2GRAY);
    threshold(gray, binary, SUDOKU_GRAY_THRES, 255, THRESH_BINARY);
    findContours(binary.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 30 || bound.area() > 1000)
            continue;

        float hw_ratio = (float)bound.height / bound.width;
        ////cout << "HW ratio: " << hw_ratio << endl;
        if (hw_ratio > 1.0)
            continue;
        //if (hw_ratio > (float)30 / 10)
            //continue;

        float area = contourArea(contours[i]);
        //cout << "Area ratio: " << area/bound.area() << endl;
        if (area / bound.area() < (float)8 / 10)
            continue;
        //cout << "Area: " << bound.area() << endl;
        side_blocks.push_back(bound);
    }

    for (uint i = 0; i < side_blocks.size(); ++i) {
        rectangle(draw, side_blocks[i], Scalar(255, 0, 255), 2);
    }

    if (side_blocks.size() < 8)
        return false;

    int top_left_x = 630,
        top_left_y = 470,
        buttom_right_x = 0,
        buttom_right_y = 0;

    for (uint i=0; i<side_blocks.size(); ++i) {
        if (side_blocks[i].x < top_left_x)
            top_left_x = side_blocks[i].x;
        if (side_blocks[i].y < top_left_y)
            top_left_y = side_blocks[i].y;
        if (side_blocks[i].x > buttom_right_x)
            buttom_right_x = side_blocks[i].x;
        if (side_blocks[i].y > buttom_right_y)
            buttom_right_y = side_blocks[i].y;
    }

    int side_block_width = side_blocks[0].width;
    int side_block_height = side_blocks[0].height;

    int whole_block_x = top_left_x + side_block_width;
    int whole_block_y = top_left_y;
    int whole_block_width = buttom_right_x - whole_block_x;
    int whole_block_height = buttom_right_y - top_left_y + side_block_height;

    if (whole_block_width < 20)
        return false;

    Rect whole_block = Rect(whole_block_x + 10, whole_block_y,
            whole_block_width - 20, whole_block_height);

    Mat block_roi = binary(whole_block);
    imshow("block roi", block_roi);

    contours.clear();
    findContours(block_roi.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < 500)
            continue;

        blocks.push_back(Rect(whole_block.tl()+bound.tl(), bound.size()));
    }

    for (uint i = 0; i < blocks.size(); ++i) {
        rectangle(draw, blocks[i], Scalar(0, 0, 255), 2);
    }

    imshow("sudoku draw", draw);
    imshow("sudoku binary", binary);

    //int block_height = whole_block_height / 3;
    //int block_width = block_height;
    //int block_gap_width = (whole_block_width - 3 * block_width) / 4;

    //for (int i=0; i<3; ++i) {
        //for (int j=0; j<3; ++j) {
            //blocks.push_back(Rect(whole_block_x +
                        //(j+1) * block_gap_width +
                        //j * block_width,
                        //whole_block_y +
                        //i * block_height,
                        //block_width, block_height));
        //}
    //}

    sort(blocks.begin(), blocks.end(), compareRect);

    //int top_x = blocks[0].x + blocks[0].width;
    //int top_y = blocks[0].y;
    //int top_width = blocks[blocks.size()-1].x - top_x;

    led_rect = Rect(whole_block_x, 0, 
            whole_block_width, whole_block_y);

    copy(blocks.begin(), blocks.end(), back_inserter(handwrite_rects));

    return true;
}
