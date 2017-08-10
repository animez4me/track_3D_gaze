//
// Created by alexandros on 21/07/17.
//

#ifndef BRISK_RGB_H
#define BRISK_RGB_H

#include <opencv2/opencv.hpp>
#include <iostream>  //NOLINT

class RGBcamera
{
public:
    RGBcamera(std::string filename);
    virtual ~RGBcamera();

    void captureFrame();

private:
    void readCameraParameters(std::string filename);

public:
    cv::Mat RGBFrame;
    cv::Mat camMatrix;
    cv::Mat distCoeffs;

private:
    cv::VideoCapture cap;

};

#endif //BRISK_RGB_H
