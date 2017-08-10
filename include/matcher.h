//
// Created by alexandros on 24/07/17.
//

#ifndef BRISK_MATCHER_H
#define BRISK_MATCHER_H

#include <opencv2/opencv.hpp>


class matcher
{
public:
    matcher();
    virtual ~matcher();

    void matchFrames();

public:
    cv::Mat imgRGB1;
    cv::Mat imgRGB2;
    cv::Mat imgGray1;
    cv::Mat imgGray2;
    std::vector < std::vector<cv::DMatch> > matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

private:
    bool do_rot;
    float threshold;
    bool scaleInvariant;
    int octaves;
    bool suppressScaleNonmaxima;
    cv::Ptr < cv::FeatureDetector > detector;
    bool hamming;
    cv::Ptr < cv::DescriptorExtractor > descriptorExtractor;
    bool rotationInvariant;
    cv::Mat descriptors1, descriptors2;
    std::vector < cv::DMatch > indices;
    cv::Ptr < cv::BFMatcher > descriptorMatcher;
    float matchingThreshold;
};


#endif //BRISK_MATCHER_H
