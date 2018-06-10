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

#ifndef __sudoku__LedSolver__
#define __sudoku__LedSolver__

#include "Headers.h"

using namespace cv::ml;

namespace sudoku {

class LedSolver {
public:
    LedSolver();
    ~LedSolver();
    void init(const char* file);
    bool process(Mat& led_roi);
    int getResult(int index);
    void setRedThreshold(int thres);

private:
    Ptr<SVM> svm;
    Mat kernel;
    HOGDescriptor* hog;
    int results[5];
    int RED_THRESHOLD;

    void getRed(Mat& led_roi, Mat& led_roi_binary);
    int predictSVM(Mat& roi);
    int predictCross(Mat& roi);
    int scanSegmentX(Mat& roi, int line_x, int y_begin, int y_end);
    int scanSegmentY(Mat& roi, int line_y, int x_begin, int x_end);
};
}

#endif /* ifndef __sudoku__LedSolver__ */
