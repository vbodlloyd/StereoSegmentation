#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <opencv2/core/types_c.h>
#include <vector>

class DTSStereo {
public :
    DTSStereo();
    int compute(uchar* rgb_image, int size, int width);
    int computeLab(uchar* rgb_image, int size, int width);
    int computeHisto(cv::Mat mat_image, int size, int width);
    int getResizeHeight();
    int getResizeWidth();



private:
    int resizeHeight;
    int resizeWidth;

    int thresholdSky_;
    int partOfSky_;
    int thresholdWidth_;
};