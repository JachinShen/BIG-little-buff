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

    LedSolver::LedSolver()
    {
        hog = NULL;
    }

    void LedSolver::init(const char* file)
    {
        svm = SVM::create();
        svm = svm->load(file);
        kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        hog = new cv::HOGDescriptor(cvSize(28, 28), cvSize(14, 14), cvSize(7, 7), cvSize(7, 7), 9);

        for (int i=0; i<5; ++i)
            results[i] = -1;

        RED_THRESHOLD = 80;
    }

    LedSolver::~LedSolver()
    {
        if (hog != NULL)
            delete hog;
    }

    void LedSolver::setRedThreshold(int thres)
    {
        RED_THRESHOLD = thres;
    }

    void LedSolver::getRed(Mat& led_roi, Mat& led_roi_binary)
    {
        static Mat bgr_split[3];
        static Mat led_roi_red;

        split(led_roi, bgr_split);
        led_roi_red = 2 * bgr_split[2] - bgr_split[1] - bgr_split[0];
        cout << "RED THRES: " << RED_THRESHOLD << endl;
        threshold(led_roi_red, led_roi_binary, RED_THRESHOLD, 255, THRESH_BINARY);
        imshow("Led Red Binary: ", led_roi_binary);
        dilate(led_roi_binary, led_roi_binary, kernel);
    }

    bool LedSolver::process(Mat& led_roi) {
        static Mat led_roi_binary;
        static Mat draw;
        vector< vector<Point> > contours;
        vector< Rect > digits;

        draw = led_roi.clone();

        getRed(led_roi, led_roi_binary);
        findContours(led_roi_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        for (uint i=0; i<contours.size(); ++i) {
            Rect bound = boundingRect(contours[i]);
            if(bound.area() < 20)
                continue;
            float hw_ratio = (float) bound.height / bound.width;
            if (hw_ratio < 1.0)
                hw_ratio = 1.0 / hw_ratio;
            if (hw_ratio < 1.5 || hw_ratio > 15)
                continue;
            digits.push_back(bound);
        }

        sort(digits.begin(), digits.end(), compareRect);

        for (uint i = 0; i < digits.size(); ++i) {
            rectangle(draw, digits[i], Scalar(255, 0, 0), 2);
        }

        imshow("draw", draw);

        if (digits.size() > 5)
            return false;

        for (uint i = 0; i < digits.size(); ++i) {
            float hw_ratio = (float) digits[i].height / digits[i].width;
            if (hw_ratio < 1.0)
                hw_ratio = 1.0 / hw_ratio;
            if (hw_ratio > 3.0) {
                results[i] = 1;
                continue; 
            }
            Mat roi = (~led_roi_binary)(digits[i]);
            results[i] = predict(roi);
        }

        return true;
    }

    int LedSolver::predict(Mat& roi)
    {
        vector<float> descriptors;
        erode(roi, roi, kernel);
        resize(roi, roi, Size(20, 20));
        Mat inner = Mat::ones(28, 28, CV_8UC1) + 254;
        roi.copyTo(inner(Rect(4,4,20,20)));
        imshow("inner", inner);
        hog->compute(inner, descriptors, Size(1, 1), Size(0, 0));

        Mat SVMPredictMat = Mat(1, (int)descriptors.size(), CV_32FC1);
        memcpy(SVMPredictMat.data, descriptors.data(), descriptors.size() * sizeof(float));
        return (svm->predict(SVMPredictMat));
    }

    int LedSolver::getResult(int index)
    {
        if (index >=5 )
            return -1;
        return results[index];
    }
}
