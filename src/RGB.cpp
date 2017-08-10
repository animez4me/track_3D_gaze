//
// Created by alexandros on 21/07/17.
//

#include "RGB.h"


RGBcamera::RGBcamera(std::string filename)
{
    cap = cv::VideoCapture(0);
    readCameraParameters(filename);
}

RGBcamera::~RGBcamera() {}

void RGBcamera::captureFrame()
{
    cap.read(RGBFrame);
}

void RGBcamera::readCameraParameters(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cout << "RGB calibration file cannot be opened" << std::endl;
    fs["Camera_Matrix"] >> camMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    std::cout << "RGB cameraMatrix: " << camMatrix << std::endl;
    std::cout << "RGB distortion: " << distCoeffs << std::endl;
}
