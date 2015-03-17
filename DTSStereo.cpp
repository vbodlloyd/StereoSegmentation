#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DTSStereo.h"

/*
* Destroy The Sky ( DTS ) is a class which remove the sky from an image and return the new height of it.
* It not have to be a real sky, in fact it look the up part of the image and remove the up until it pass a threshold
* of not "sky"
*
* For all compute function, the method is the same. We take the mean of the first  and the fifth line of the image and
* compare it with all the lines under. If the mean color is not present at 'threshold' percent, we consider this is
* no more the "sky"
* */


const int DTSSTEREO_THRESHOLD_SKY = 5;
const int DTSSTEREO_PART_OF_SKY = 6;
const int DTSSTEREO_THRESHOLD_WIDTH = 70;

int32_t DTSStereo::getResizeHeight() {
    return this->resizeHeight;
}

int32_t DTSStereo::getResizeWidth() {
    return this->resizeWidth;
}

DTSStereo::DTSStereo() : partOfSky_(DTSSTEREO_PART_OF_SKY),
                        thresholdSky_(DTSSTEREO_THRESHOLD_SKY),
                        thresholdWidth_(DTSSTEREO_THRESHOLD_WIDTH){}

int DTSStereo::compute(uchar *rgb_image, int size, int width) {
    //compute with a rgb image
    int indexForMean = 0;
    int value_r=0,value_g=0,value_b=0;
    for(int idx1 = 0; idx1 < width*3 ; idx1=idx1+3){
        value_r += rgb_image[idx1];
        value_g += rgb_image[idx1+1];
        value_b += rgb_image[idx1+2];
        indexForMean++;
    }
    int indexForMean2 = 0;
    int value_r2=0,value_g2=0,value_b2=0;
    for(int idx1 = width*3*2; idx1 < width*3*3 ; idx1=idx1+3){
        value_r2 += rgb_image[idx1];
        value_g2 += rgb_image[idx1+1];
        value_b2 += rgb_image[idx1+2];
        indexForMean2++;
    }
    int mean_r = static_cast<int>(ceil(value_r/indexForMean));
    int mean_g = static_cast<int>(ceil(value_g/indexForMean));
    int mean_b = static_cast<int>(ceil(value_b/indexForMean));
    int mean_r2 = static_cast<int>(ceil(value_r2/indexForMean2));
    int mean_g2 = static_cast<int>(ceil(value_g2/indexForMean2));
    int mean_b2 = static_cast<int>(ceil(value_b2/indexForMean2));
    printf("mean r g b : %d  %d  %d with size : %d\n",mean_r,mean_b,mean_g,size);
    printf("mean r g b : %d  %d  %d with size : %d\n",mean_r2,mean_b2,mean_g2,size);
    int newHeight = 0,indexWidth = 0, nbrWidthSky =0 ,truc=0,machin = 5;
    bool isSky = true;
    for(int idx = 0; idx < size*3 ; idx = idx+3){
        if(isSky) {
            if (rgb_image[idx] <= mean_r + thresholdSky_ && rgb_image[idx] >= mean_r - thresholdSky_ &&
                    rgb_image[idx + 1] <= mean_g + thresholdSky_ && rgb_image[idx + 1] >= mean_g - thresholdSky_ &&
                    rgb_image[idx + 2] <= mean_b + thresholdSky_ && rgb_image[idx + 2] >= mean_b - thresholdSky_) {
                nbrWidthSky++;

            }else if(rgb_image[idx] <= mean_r2 + thresholdSky_ && rgb_image[idx] >= mean_r2 - thresholdSky_ &&
                    rgb_image[idx + 1] <= mean_g2 + thresholdSky_ && rgb_image[idx + 1] >= mean_g2 - thresholdSky_ &&
                    rgb_image[idx + 2] <= mean_b2 + thresholdSky_ && rgb_image[idx + 2] >= mean_b2 - thresholdSky_){

                nbrWidthSky++;
            }

            indexWidth++;
            if (indexWidth >= width) {
                indexWidth = 0;
                if (nbrWidthSky*3 >= (float)( thresholdWidth_*0.01) * width*3) {
                    //delete this line
                    truc = 0;

                } else {
                    //complete the image
                    truc++;
                    if(truc > machin ) {
                        isSky = false;
                    }
                }
                nbrWidthSky = 0;
            }
        }else{

            newHeight++;
        }

    }
    return (newHeight/width);
}

int DTSStereo::computeLab(uchar *rgb_image, int size, int width) {
    //compute with a rgb image change in lab image
    float * inputLabImage_ = reinterpret_cast<float*>(malloc(size*3*sizeof(float)));
    std::vector<float> sRGBGammaCorrections(256);
    for (int pixelValue = 0; pixelValue < 256; ++pixelValue) {
        double normalizedValue = pixelValue/255.0;
        double transformedValue = (normalizedValue <= 0.04045) ? normalizedValue/12.92 : pow((normalizedValue+0.055)/1.055, 2.4);

        sRGBGammaCorrections[pixelValue] = static_cast<float>(transformedValue);
    }

    for (int idx = 0; idx < size*3; idx=idx+3) {

            float correctedR = sRGBGammaCorrections[rgb_image[idx]];
            float correctedG = sRGBGammaCorrections[rgb_image[idx+1]];
            float correctedB = sRGBGammaCorrections[rgb_image[idx+2]];

            float xyzColor[3];
            xyzColor[0] = correctedR*0.412453f + correctedG*0.357580f + correctedB*0.180423f;
            xyzColor[1] = correctedR*0.212671f + correctedG*0.715160f + correctedB*0.072169f;
            xyzColor[2] = correctedR*0.019334f + correctedG*0.119193f + correctedB*0.950227f;

            const double epsilon = 0.008856;
            const double kappa = 903.3;
            const double referenceWhite[3] = { 0.950456, 1.0, 1.088754 };

            float normalizedX = static_cast<float>(xyzColor[0]/referenceWhite[0]);
            float normalizedY = static_cast<float>(xyzColor[1]/referenceWhite[1]);
            float normalizedZ = static_cast<float>(xyzColor[2]/referenceWhite[2]);
            float fX = (normalizedX > epsilon) ? static_cast<float>(pow(normalizedX, 1.0/3.0)) : static_cast<float>((kappa*normalizedX + 16.0)/116.0);
            float fY = (normalizedY > epsilon) ? static_cast<float>(pow(normalizedY, 1.0/3.0)) : static_cast<float>((kappa*normalizedY + 16.0)/116.0);
            float fZ = (normalizedZ > epsilon) ? static_cast<float>(pow(normalizedZ, 1.0/3.0)) : static_cast<float>((kappa*normalizedZ + 16.0)/116.0);

            inputLabImage_[idx] = static_cast<float>(116.0*fY - 16.0);
            inputLabImage_[idx+1] = static_cast<float>(500.0*(fX - fY));
            inputLabImage_[idx+2] = static_cast<float>(200.0*(fY - fZ));

    }
    int indexForMean = 0;
    int value_l=0,value_a=0,value_b=0;
    for(int idx1 = 0; idx1 < width*3 ; idx1=idx1+3){
        value_l += inputLabImage_[idx1];
        value_a += inputLabImage_[idx1+1];
        value_b += inputLabImage_[idx1+2];
        indexForMean++;
    }
    int indexForMean2 = 0;
    int value_l2=0,value_a2=0,value_b2=0;
    for(int idx1 = width*3*2; idx1 < width*3*3 ; idx1=idx1+3){
        value_l2 += inputLabImage_[idx1];
        value_b2 += inputLabImage_[idx1+1];
        value_b2 += inputLabImage_[idx1+2];
        indexForMean2++;
    }
    int mean_l = static_cast<int>(ceil(value_l/indexForMean));
    int mean_a = static_cast<int>(ceil(value_a/indexForMean));
    int mean_b = static_cast<int>(ceil(value_b/indexForMean));
    int mean_l2 = static_cast<int>(ceil(value_l2/indexForMean2));
    int mean_a2 = static_cast<int>(ceil(value_a2/indexForMean2));
    int mean_b2 = static_cast<int>(ceil(value_b2/indexForMean2));
    printf("mean r g b : %d  %d  %d with size : %d\n",mean_l,mean_b,mean_a,size);
    printf("mean r g b : %d  %d  %d with size : %d\n",mean_l2,mean_b2,mean_a2,size);
    int newHeight = 0,indexWidth = 0, nbrWidthSky =0 ,truc=0,machin = 1;
    bool isSky = true;
    for(int idx = 0; idx < size*3 ; idx = idx+3){
        if(isSky) {
            if ( inputLabImage_[idx + 1] <= mean_a + thresholdSky_ && inputLabImage_[idx + 1] >= mean_a - thresholdSky_ &&
                    inputLabImage_[idx + 2] <= mean_b + thresholdSky_ && inputLabImage_[idx + 2] >= mean_b - thresholdSky_) {
                nbrWidthSky++;

            }else if(
                    inputLabImage_[idx + 1] <= mean_a2 + thresholdSky_ && inputLabImage_[idx + 1] >= mean_a2 - thresholdSky_ &&
                            inputLabImage_[idx + 2] <= mean_b2 + thresholdSky_ && inputLabImage_[idx + 2] >= mean_b2 - thresholdSky_){

                nbrWidthSky++;
            }

            indexWidth++;
            if (indexWidth >= width) {
                indexWidth = 0;
                if (nbrWidthSky*3 >= (float)( thresholdWidth_*0.01) * width*3) {
                    //delete this line
                    truc = 0;

                } else {
                    //complete the image
                    truc++;
                    if(truc > machin ) {
                        isSky = false;
                    }
                }
                nbrWidthSky = 0;
            }
        }else{
            newHeight++;
        }
    }

    free(inputLabImage_);
    return (newHeight/width);
}

int DTSStereo::computeHisto(cv::Mat mat_image, int size, int width) {
    //compute with a greyscale image obtain with the threshold function of the opencv library
    cv::Mat grey_image;
    cv::cvtColor(mat_image,grey_image,cv::COLOR_RGB2GRAY);
   // cv::equalizeHist(grey_image,grey_image);
    cv::threshold(grey_image, grey_image, 252, 255, cv::THRESH_OTSU);
    uchar* img = grey_image.data;
    bool isSky = true;
    int nbrWidthSky =0,newHeight = 0,indexWidth=0;
    for(int idx = 0; idx < size ; idx++){
        if(isSky) {
            if (img[idx] <= 255 + thresholdSky_ && img[idx] >= 255 - thresholdSky_) {
                nbrWidthSky++;}

            indexWidth++;
            if (indexWidth >= width) {
                indexWidth = 0;
                if (nbrWidthSky >= (float)( thresholdWidth_*0.01) * width) {
                    //delete this line
                } else {
                    //complete the image
                    isSky = false;
                }
                nbrWidthSky = 0;
            }
        }else{
            newHeight++;
        }

    }

    return (newHeight/width);
}

