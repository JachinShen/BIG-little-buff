//
//  caffe_prototype.cpp
//  sudoku
//
//  Created by ding on 17/6/5.
//  Copyright (c) 2017年 ding. All rights reserved.
//

#ifndef _SUDOKU_CAFFE_PROTOTYPE_
#define _SUDOKU_CAFFE_PROTOTYPE_

#include "Headers.h"

#if PLATFORM == PC
#  define CPU_ONLY
#endif

#include <caffe/caffe.hpp>

using namespace caffe;
/*-------------------caffe prototype---------------*/

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;

class Classifier {
public:
    Classifier(){};
    void Initialize(const string& model_file,
        const string& trained_file,
        const string& mean_file,
        const string& label_file);

    std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);
    std::vector<float> Predict(const cv::Mat& img);

private:
    void SetMean(const string& mean_file);

    void WrapInputLayer(std::vector<cv::Mat>* input_channels);

    void Preprocess(const cv::Mat& img,
        std::vector<cv::Mat>* input_channels);

private:
    boost::shared_ptr<Net<float> > net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;
    std::vector<string> labels_;
};

/*-------------------caffe prototype---------------*/
#endif
