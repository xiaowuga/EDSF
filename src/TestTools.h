//
// Created by xiaowuga on 2021/11/24.
//

#ifndef CYLINDERPOSETRACKING_TESTTOOLS_H
#define CYLINDERPOSETRACKING_TESTTOOLS_H

#include <iostream>
#include <opencv2/opencv.hpp>

bool TestOverlap(const cv::Mat1b& gt, const cv::Mat1b& test, float th);
cv::Vec3f ell_evaluate(std::vector<cv::RotatedRect>& ellGt,
                       std::vector<cv::RotatedRect>& ellTest, float th_score, int h, int w);
#endif //CYLINDERPOSETRACKING_TESTTOOLS_H
