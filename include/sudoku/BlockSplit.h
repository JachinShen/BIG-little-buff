/*********************************************************************
recognize the 7-segment led number
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
*********************************************************************/

#include "Headers.h"

class BlockSplit {
public:
    BlockSplit();
    ~BlockSplit();
    void init();
    //bool processMnist(Mat& input, Rect& led_rect, vector<Rect>& handwrite_rects);
    bool process(Mat& input, Rect& led_rect, Rect& sudoku_rect);
    void setParam(int index, int data);
    //void createSudokuParamTrackbar(char * window_name, cv::TrackbarCallback * callbackFunc);
    enum SudokuParam {
        GRAY_THRES,
        AREA_MIN,
        AREA_MAX,
        HW_RATIO_MIN,
        HW_RATIO_MAX,
        AREA_RATIO,
        PARAM_SIZE
    };

    
private:
    int param[PARAM_SIZE];

    //int SUDOKU_GRAY_THRES;
    //int SUDOKU_AREA_MIN;
    //int SUDOKU_AREA_MAX;
    //int SUDOKU_HW_RATIO_MIN;
    //int SUDOKU_HW_RATIO_MAX;
    //int SUDOKU_AREA_RATIO;
};
