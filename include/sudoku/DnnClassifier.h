#include "Headers.h"
#include "sudoku/caffe_prototype.h"

class DnnClassifier {
public:
    DnnClassifier(){};
    ~DnnClassifier(){};
    void init(const string& model_file,
        const string& trained_file,
        const string& mean_file,
        const string& label_file);

    void process(vector<Mat>& block_roi);
    int getNumberBlockID(int number);
    int confirmNumber(int number);
    bool confirm();

private:
    Classifier classifier;
    Mat kernel;
    int results[10];
    float results_value[10];
};
