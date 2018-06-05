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

#include "sudoku/LedSolver.h"

namespace sudoku {
    void LedSolver::process(Mat& led_roi) {
        static Mat led_roi_binary;
        static Mat bgr_split[3];
        static Mat led_roi_red;
        vector< vector<Point> > contours;
        split(led_roi, bgr_split);
        led_roi_red = 2 * bgr_split[2] - bgr_split[1] - bgr_split[0];
        imshow("Led Red: ", led_roi_red);
        //threshold(led_roi, led_roi_binary, 128, 255, THRESH_BINARY);
        //findContours(led_roi_binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        //for (uint i=0; i<contours.size(); ++i) {
            //Rect bound = boundingRect(contours[i]);
            //float hw_ratio = (float) bound.height / bound.width;
            //if (hw_ratio < 1.0)
                //hw_ratio = 1.0 / hw_ratio;
            //if (hw_ratio < 1.5 || hw_ratio > 3)
                //continue;
        //}
    }
}
