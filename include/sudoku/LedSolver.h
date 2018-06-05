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
    LedSolver(const char* file)
    {
        svm = SVM::create();
        svm = svm->load(file);
        kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        hog = new cv::HOGDescriptor(cvSize(28, 28), cvSize(14, 14), cvSize(7, 7), cvSize(7, 7), 9);
    };
    ~LedSolver(){};
    void process(Mat& led_roi);

private:
    Ptr<SVM> svm;
    Mat kernel;
    HOGDescriptor *hog;
};
}

#endif /* ifndef __sudoku__LedSolver__ */
