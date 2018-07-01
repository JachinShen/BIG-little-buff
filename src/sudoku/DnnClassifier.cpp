#include "sudoku/DnnClassifier.h"

void DnnClassifier::init(const string& model_file,
    const string& trained_file,
    const string& mean_file,
    const string& label_file)
{
    classifier.Initialize(model_file, trained_file, mean_file, label_file);
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
}

void DnnClassifier::process(vector<Mat>& block_roi)
{
    Mat roi;
    vector<vector<float> > predictions;
    for (uint i = 0; i < block_roi.size(); ++i) {
        predictions.push_back(classifier.Predict(block_roi[i]));
    }

    for (int i = 0; i < 10; ++i) {
        results[i] = -1;
        results_value[i] = -1;
    }

    for (uint i = 0; i < predictions.size(); ++i) {
        for (uint j = 0; j < predictions[i].size(); ++j) {
            if (predictions[i][j] > results_value[j]) {
                results_value[j] = predictions[i][j];
                results[j] = i + 1;
            }
            //cout << " " << predictions[i][j];
        }
        //cout << endl;
    }
}

int DnnClassifier::getNumberBlockID(int number)
{
    return results[number];
}

int DnnClassifier::confirmNumber(int number)
{
    return results_value[number] * 100;
}
