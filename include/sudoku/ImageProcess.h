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

#ifndef __sudoku__ImageProcess__
#define __sudoku__ImageProcess__

#include "Headers.h"

class ImageProcess {
public:
    ImageProcess();
    ~ImageProcess();
    void init();
    bool process(Mat& input, Rect& led_rect, vector<Rect>& handwrite_rects);
    void setParam(int index, int data);

private:
    int SUDOKU_GRAY_THRES;
    int SUDOKU_AREA_MIN;
    int SUDOKU_AREA_MAX;
    int SUDOKU_HW_RATIO_MIN;
    int SUDOKU_AREA_RATIO;
};

#endif /* defined(__sudoku__ImageProcess__) */
