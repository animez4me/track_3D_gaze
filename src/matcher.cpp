//
// Created by alexandros on 24/07/17.
//

#include "matcher.h"
#include <brisk/brisk.h>

matcher::matcher()
{
    threshold = 34.0;
    scaleInvariant = true;
    do_rot = false;
    octaves = 4;

    if (!scaleInvariant) {
        octaves = 0;
    }

    suppressScaleNonmaxima = true;
    detector = new brisk::BriskFeatureDetector(threshold, octaves, suppressScaleNonmaxima);

    // Now the extractor:
    hamming = true;
    rotationInvariant = true;
    // Now the extractor:
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
            rotationInvariant, scaleInvariant, brisk::BriskDescriptorExtractor::Version::briskV2);

    matchingThreshold = 0.0;
}

matcher::~matcher() {}

void matcher::matchFrames()
{
    keypoints1.clear();
    keypoints2.clear();
    matches.clear();

    detector->detect(imgGray1, keypoints1);
    descriptorExtractor->compute(imgGray1, keypoints1, descriptors1);

    detector->detect(imgGray2, keypoints2);
    descriptorExtractor->compute(imgGray2, keypoints2, descriptors2);

    /// Matching
    if (hamming) {
        brisk::BruteForceMatcher matcher;
        if (matchingThreshold < 1.0e-12) {
            matchingThreshold = 55.0 * descriptors1.cols / 48.0;
        }
        matcher.radiusMatch(descriptors2, descriptors1, matches, matchingThreshold);
    } else {
        cv::BFMatcher matcher(cv::NORM_L2);
        if (matchingThreshold < 1.0e-12) {
            matchingThreshold = 0.21;
        }
        matcher.radiusMatch(descriptors2, descriptors1, matches, matchingThreshold);
    }
}