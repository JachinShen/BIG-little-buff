#ifndef __sudoku__LedSolver__
#define __sudoku__LedSolver__

#include "Headers.h"

//using namespace cv::ml;

class LedSolver {
public:
    LedSolver();
    ~LedSolver();
    //void init(const char* file);
    void init();
    bool process(Mat& led_roi, Rect& bound_all_rect);
    int getResult(int index);
    void setParam(int index, int value);
    bool confirmLed();

	enum LedParam{
		RED_THRESHOLD,
		GRAY_THRESHOLD,
		BOUND_AREA_MIN,
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

    void getRed(Mat& led_roi, Mat& led_roi_binary);
    //int predictSVM(Mat& roi);
    int predictCross(Mat& roi);
    int scanSegmentX(Mat& roi, int line_x, int y_begin, int y_end);
    int scanSegmentY(Mat& roi, int line_y, int x_begin, int x_end);
};
#endif /* ifndef __sudoku__LedSolver__ */
