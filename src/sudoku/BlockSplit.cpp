#include "sudoku/BlockSplit.h"

BlockSplit::BlockSplit()
{
}

BlockSplit::~BlockSplit()
{
}

void BlockSplit::init()
{
    param[GRAY_THRES]   = 170;
    param[AREA_MIN]     = 500;
    param[AREA_MAX]     = 2000;
    param[HW_RATIO_MIN] = 30; // 2.0
    param[HW_RATIO_MAX] = 100; // 2.0
    param[AREA_RATIO]   = 70;    // 0.6
}

void BlockSplit::setParam(int index, int data)
{
    if (0 <= index && index < PARAM_SIZE) {
        param[index] = data;
        return;
    } else {
        cout << "Set Param Error!" << endl;
    }
    //switch (index) {
    //case 1:
        //param[GRAY_THRES]   = data;
        //break;
    //case 2:
        //param[AREA_MIN]     = data;
        //break;
    //case 3:
        //param[AREA_MAX]     = data;
        //break;
    //case 4:
        //param[HW_RATIO_MIN] = data;
        //break;
    //case 5:
        //param[HW_RATIO_MAX] = data;
        //break;
    //case 6:
        //param[AREA_RATIO]   = data;
        //break;
    //default:
        //break;
    //}
}

//bool BlockSplit::processMnist(Mat& input, Rect& led_rect,
    //vector<Rect>& handwrite_rects)
//{
    //static Mat gray, binary;
    //static Mat draw;
    //vector<vector<Point> > contours;
    //vector<Rect> blocks;

    //draw = input.clone();
    //cvtColor(input, gray, CV_BGR2GRAY);
    //threshold(gray, binary, param[GRAY_THRES], 255, THRESH_BINARY);
    //findContours(binary.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    //for (uint i = 0; i < contours.size(); ++i) {
        //Rect bound = boundingRect(contours[i]);
        //if (bound.area() < param[AREA_MIN] || bound.area() > param[AREA_MAX])
            //continue;

        //float hw_ratio = (float)bound.height / bound.width;
        //if (hw_ratio < 1.0)
            //hw_ratio = 1.0 / hw_ratio;
        ////cout << "HW ratio: " << hw_ratio << endl;
        //if (hw_ratio > (float)param[HW_RATIO_MAX] / 10)
            //continue;

        //float area = contourArea(contours[i]);
        ////cout << "Area ratio: " << area/bound.area() << endl;
        //if (area / bound.area() < (float)param[AREA_RATIO] / 10)
            //continue;
        ////cout << "Area: " << bound.area() << endl;
        //blocks.push_back(bound);
    //}

    //sort(blocks.begin(), blocks.end(), compareRect);

    //for (uint i = 0; i < blocks.size(); ++i) {
        //rectangle(draw, blocks[i], Scalar(255, 0, 255), 2);
    //}
    //imshow("sudoku draw", draw);
    //imshow("sudoku binary", binary);

    //if (blocks.size() != 9)
        //return false;

    //int top_x = blocks[0].x;
    //int top_y = blocks[0].y;
    //int top_width = blocks[2].x + blocks[2].width - top_x;

    //led_rect = Rect(top_x, 0, top_width, top_y);
    //copy(blocks.begin(), blocks.end(), back_inserter(handwrite_rects));

    //return true;
//}

bool BlockSplit::process(Mat& input, Rect& led_rect,
    Rect& sudoku_rect)
{
    static Mat gray, yellow, binary;
    static Mat draw;
    vector<vector<Point> > contours;
    vector<Rect> side_blocks;
    vector<Rect> blocks;

    draw = input.clone();
    if (input.channels() != 1) {
        cvtColor(input, gray, CV_BGR2GRAY);
    }
    threshold(gray, binary, param[GRAY_THRES], 255, THRESH_BINARY);
    findContours(binary.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); ++i) {
        Rect bound = boundingRect(contours[i]);
        if (bound.area() < param[AREA_MIN] || bound.area() > param[AREA_MAX])
            continue;

        float hw_ratio = (float)bound.height / bound.width * 100;
        ////cout << "HW ratio: " << hw_ratio << endl;
        if (hw_ratio < param[HW_RATIO_MIN] || hw_ratio > param[HW_RATIO_MAX])
            continue;
        //if (hw_ratio > (float)30 / 10)
            //continue;

        float area = contourArea(contours[i]);
        //cout << "Area ratio: " << area/bound.area() << endl;
        if (area / bound.area() * 100 < param[AREA_RATIO])
            continue;
        //cout << "Area: " << bound.area() << endl;
        side_blocks.push_back(bound);
    }

    for (uint i = 0; i < side_blocks.size(); ++i) {
        rectangle(draw, side_blocks[i], Scalar(255, 0, 255), 2);
    }
    imshow("sudoku draw", draw);
    imshow("sudoku binary", binary);

    if (side_blocks.size() != 10)
        return false;

    int top_left_x = input.cols,
        top_left_y = input.rows,
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

    if (whole_block_width < side_blocks[0].width)
        return false;

    sudoku_rect = Rect(whole_block_x + 10, whole_block_y,
            whole_block_width - 20, whole_block_height);

    //cout << "Sudoku Rect: " << sudoku_rect << endl;

    //Mat block_roi = binary(whole_block);
    //imshow("block roi", block_roi);

    //contours.clear();
    //findContours(block_roi.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //for (uint i = 0; i < contours.size(); ++i) {
        //Rect bound = boundingRect(contours[i]);
        //if (bound.area() < 500)
            //continue;

        //blocks.push_back(Rect(whole_block.tl()+bound.tl(), bound.size()));
    //}

    //for (uint i = 0; i < blocks.size(); ++i) {
        //rectangle(draw, blocks[i], Scalar(0, 0, 255), 2);
    //}

    //imshow("sudoku draw", draw);
    //imshow("sudoku binary", binary);

    ////int block_height = whole_block_height / 3;
    ////int block_width = block_height;
    ////int block_gap_width = (whole_block_width - 3 * block_width) / 4;

    ////for (int i=0; i<3; ++i) {
        ////for (int j=0; j<3; ++j) {
            ////blocks.push_back(Rect(whole_block_x +
                        ////(j+1) * block_gap_width +
                        ////j * block_width,
                        ////whole_block_y +
                        ////i * block_height,
                        ////block_width, block_height));
        ////}
    ////}

    //sort(blocks.begin(), blocks.end(), compareRect);

    ////int top_x = blocks[0].x + blocks[0].width;
    ////int top_y = blocks[0].y;
    ////int top_width = blocks[blocks.size()-1].x - top_x;

    led_rect = Rect(whole_block_x, 0, 
            whole_block_width, whole_block_y);

    //copy(blocks.begin(), blocks.end(), back_inserter(handwrite_rects));

    return true;
}
