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

//using namespace cv::ml;

class LedSolver {
public:
    LedSolver();
    ~LedSolver();
    void init(const char* file);
    bool process(Mat& led_roi);
    int getResult(int index);
    void setParam(int index, int value);

	enum LedParam{
		RED_THRESHOLD,
		GRAY_THRESHOLD,
		BOUND_AREA_MAX,
		HW_RATIO_MIN,
		HW_RATIO_MAX,
		HW_RATIO_FOR_DIGIT_ONE,
		ROTATION_DEGREE,
		PARAM_SIZE
	};



private:
    //Ptr<SVM> svm;
    Mat kernel;
    //HOGDescriptor* hog;
    int results[5];
	
	int param[PARAM_SIZE];

    //int RED_THRESHOLD;
    //int GRAY_THRESHOLD;

    void getRed(Mat& led_roi, Mat& led_roi_binary);
    //int predictSVM(Mat& roi);
    int predictCross(Mat& roi);
    int scanSegmentX(Mat& roi, int line_x, int y_begin, int y_end);
    int scanSegmentY(Mat& roi, int line_y, int x_begin, int x_end);
};
#endif /* ifndef __sudoku__LedSolver__ */
