#include "STVFlow.h"
/* Segment The Video ( STV )
* segmentation of the image with colors ( not very good )
* */
STVFlow::STVFlow(){};

void STVFlow::compute(cv::Mat& disparityImage,cv::Mat& leftImage) {

    disparityImage.convertTo(disparityImage, CV_8U,1/255.0,1/255.0);
    cv::Mat disp_far,disp_close;

    cv::threshold(disparityImage,disp_far,50, 255, cv::THRESH_BINARY);
    cv::threshold(disparityImage,disp_close,100, 255, cv::THRESH_BINARY);
    int height = disparityImage.rows;
    int width = disparityImage.cols;
    cv::Mat black(height,width, CV_8UC1);
    memset(black.data, 0, height*width);

    int heightMax = height;
    for(int y = height*1/3; y < height ; ++y){
        int nbrBlank = 0;
        for(int x = 0; x<width ; ++x){
            if(disp_far.at<unsigned char>(y,x) == 255){
                nbrBlank++;
            }
        }
        if(nbrBlank > width *0.80){
            heightMax = y;
            break;
        }
    }

    for(int y = 0; y < heightMax ; ++y){
        for(int x = 0; x<width ; ++x) {

            if (disp_far.at<unsigned char>(y, x) == 255) {
                bool isVines = true;
                int idxOfSize = 0, idxOfSize2 = 0,idxOfSizeY=0;
                // check if line is white too ( but not all ) >
                for (int xi = x; xi < x + 50 && xi < width; ++xi) {
                    if (disp_far.at<unsigned char>(y, xi) == 255) {
                        if (xi == width - 1) {
                            idxOfSize += 1000;
                        }
                        idxOfSize++;
                    }
                }
                // check if line is white too ( but not all ) <
                for (int xi = x; xi > x - 50 && xi > 0; --xi) {
                    if (disp_far.at<unsigned char>(y, xi) == 255) {
                        if (xi == 1) {
                            idxOfSize2 += 1000;
                        }
                        idxOfSize2++;
                    }
                }
                // check if column is white too ( could be all ) V
                for (int yi = y; yi < heightMax; ++yi) {
                    idxOfSizeY++;
                    if (disp_far.at<unsigned char>(yi, x) != 255) {
                        isVines = false;
                        break;
                    }
                }


                if (idxOfSize + idxOfSize2 > 10 && idxOfSizeY>10/*&& idxOfSize+idxOfSize2 < 100 */&& isVines) {
                    //printf("promoted point : %d  %d \n",x,y);
                    /*points_.push_back(x);
                    points_.push_back(y);*/
                    /* tableauPoints[y*width+x] = x;
                    tableauPoints[y*width+x+1] = y;*/
                    cv::Rect win2(x - idxOfSize2, y, idxOfSize + idxOfSize2, 1);
                    cv::rectangle(black, win2, cv::Scalar(255, 255, 255), 1);
                }
            }
        }}
    for(int y = 0; y < height ; ++y){
        for(int x = 0; x<width ; ++x){
            if(disp_close.at<unsigned char>(y,x) == 255){
               // printf("white detected at %d %d\n",x,y);
                bool isVines = true;
                int idxOfSize =0,idxOfSize2=0;
                // check if line is white too ( but not all ) >
                for(int xi = x; xi < x+50 && xi < width; ++xi){
                    if(disp_close.at<unsigned char>(y,xi) == 255){
                        idxOfSize++;
                    }
                }

                // check if line is white too ( but not all ) <
                for(int xi = x; xi > x-50 && xi > 0; --xi){
                    if(disp_close.at<unsigned char>(y,xi) == 255){

                        idxOfSize2++;
                    }
                }

                // check if column is white too ( could be all ) V
                for(int yi = y; yi < y+50 && yi < height ; ++yi){
                    if(disp_close.at<unsigned char>(yi,x) != 255){
                        isVines = false;
                        break;
                    }
                }

                if(idxOfSize+idxOfSize2 > 10 /*&& idxOfSize+idxOfSize2 < 60*/ && isVines){
                    //printf("promoted point : %d  %d \n",x,y);
                    /*points_.push_back(x);
                    points_.push_back(y);*/
                   /* tableauPoints[y*width+x] = x;
                    tableauPoints[y*width+x+1] = y;*/
                    cv::Rect win2(x-idxOfSize2,y,idxOfSize+idxOfSize2,1);
                    cv::rectangle(black,win2,cv::Scalar(255,255,255),1);

                }
            }
        }
    }
    /*cv::namedWindow("ggg");
    cv::imshow("ggg", black);
    cv::waitKey(100);*/
    int state = 0;//Seeking , 1 : find and looking for width, 2 : looking for height, 3 : display it
    int xpos=0, ypos=0,ymem =0;
    int w=0,h=0;
    int mdH = height;
    int mdW = width;
    for(int y = 0; y < mdH ; ++y){
        for(int x = 0; x<mdW ; ++x){
            if(state == 0) {
                if (black.at<unsigned char>(y, x) == 0) {continue;}
                xpos = x; ypos = y;
                state = 1;
                ymem = y;
            }else if(state == 1){
                if(ymem != y){
                    state = 0;
                    continue;
                }
                if (black.at<unsigned char>(y, x) == 255) {continue;}
                w = x-xpos;
                state =2;
                x-= w/2;
            }else if(state ==2){
                --x;
                y+=1;
                if (black.at<unsigned char>(y, x) == 255) {continue;}
                h = y - ypos;
                state = 3;
            }else if(state ==3){
                state =0;
                cv::Rect win2(xpos,ypos,w,h);
                if(h > 20 && w > 3) {
                    cv::rectangle(leftImage, win2, cv::Scalar(255, 255, 255), 1);

                }
                cv::rectangle(black,win2,cv::Scalar(0,0,0),-1);
                x = 0; y =0;
            }
        }
    }

}

void STVFlow::compute3D(cv::Mat& disparityImage,cv::Mat& dest) {

    cv::Mat Q(4,4, CV_32FC1);
    Q.at<float>(0,0)=1;
    Q.at<float>(0,1)=0;
    Q.at<float>(0,2)=0;
    Q.at<float>(0,3)=-348.69208526611328;
    Q.at<float>(1,0)=0;
    Q.at<float>(1,1)=1;
    Q.at<float>(1,2)=0;
    Q.at<float>(1,3)=-240.57121276855469;
    Q.at<float>(2,0)=0;
    Q.at<float>(2,1)=0;
    Q.at<float>(2,2)=0;
    Q.at<float>(2,3)=291.85303892184800;
    Q.at<float>(3,0)=0;
    Q.at<float>(3,1)=0;
    Q.at<float>(3,2)=0.10283932101370377;
    Q.at<float>(3,3)=0;

   /* Q.at<float>(0,0)=1;
    Q.at<float>(1,0)=0;
    Q.at<float>(2,0)=0;
    Q.at<float>(3,0)=-348.69208526611328;
    Q.at<float>(0,1)=0;
    Q.at<float>(1,1)=1;
    Q.at<float>(2,1)=0;
    Q.at<float>(3,1)=-240.57121276855469;
    Q.at<float>(0,2)=0;
    Q.at<float>(1,2)=0;
    Q.at<float>(2,2)=0;
    Q.at<float>(3,2)=291.85303892184800;
    Q.at<float>(0,3)=0;
    Q.at<float>(1,3)=0;
    Q.at<float>(2,3)=0.10283932101370377;
    Q.at<float>(3,3)=0;*/
    disparityImage.convertTo(disparityImage, CV_8U,1/255.0,1/255.0);
    cv::Mat tmp(disparityImage.size(), CV_32FC3);
    cv::reprojectImageTo3D(disparityImage,tmp, Q,false, CV_32F);
    int height = disparityImage.rows;
    int width = disparityImage.cols;
    int state =0,xpos=0,ypos=0,ymem=0,w=0,h=0,zmem=0;
    dest.create(height,width, CV_8UC1);
    memset(dest.data, 0, height*width);

    for(int y = 0; y < height ; ++y){
        float * jesa = tmp.ptr<float>(y);
        for(int x = 0; x<width ; ++x) {
            float zi = jesa[x*3 +2];
            if (zi < 50) {
                dest.at<unsigned char>(y, x) = 255;
            }


            /*if(state == 0) {
                if (zi >= 100) {continue;}
                xpos = x; ypos = y;
                state = 1;
                ymem = y;
                zmem = zi;
            }else if(state == 1){
                if(ymem != y){
                    state = 0;
                    continue;
                }
                if (zmem >= zi-10 && zmem <= zi+10) {continue;}
                w = x-xpos;
                state =2;
                x-= w/2;
            }else if(state ==2){
                --x;
                y+=1;
                if (zmem >= zi-10 && zmem <= zi+10) {continue;}
                h = y - ypos;
                state = 3;
            }else if(state ==3){
                state =0;
                //cv::Rect win2(xpos,ypos,w,h);
               // if(h > 20 && w > 3) {
                    dest.at<unsigned char>(y, x) = 255;

                //}
                //cv::rectangle(black,win2,cv::Scalar(0,0,0),-1);
                //x = 0; y =0;
            }*/
        }
    }


    /*for(int i=0; i < tmp.cols*tmp.rows*3;i=i+3){
        if(tmp.data[i+2] > 100) {
            printf("CA PLANTE PSK %d  %d et %d et %d\n",tmp.data[i],tmp.data[i+1],i, tmp.cols*tmp.rows*3);
            dest.at<unsigned char>(tmp.data[i],tmp.data[i+1]) = 127;
        }
    }*/

}