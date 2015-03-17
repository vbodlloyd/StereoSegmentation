#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <vector>

class STVFlow{

public:
    STVFlow();
    void compute(cv::Mat& disparityImage,cv::Mat& leftImage);
    void compute3D(cv::Mat& disparityImage,cv::Mat& dest);

private:
    cv::Mat previousImage;
    std::vector<int> points_;
};