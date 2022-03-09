//
// Created by xiaowuga on 2021/11/22.
//

#ifndef CYLINDERPOSETRACKING_EV_H
#define CYLINDERPOSETRACKING_EV_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

class ELLIPSE_VALIDATE {

public:
    ELLIPSE_VALIDATE(cv::RotatedRect& ell,cv::Mat& _frame, cv::Mat1s* dx, cv::Mat1s* dy, int _numBins, int _radius, int _height, int _width);
    int computeDirection(cv::Point& center);
    float dis_to_ellipse(const cv::Point& p) const;
    float getGrayVal(int r, int c);
    bool calc_grad_mag_ori(int r, int c, double& mag, double& ori);
    void reset();
    double computeEnergy();

private:
    int xc, yc;
    float a, b;
    float cos_theta, sin_theta;
    float invA2, invB2;

    cv::Mat1s* dx;
    cv::Mat1s* dy;

    int radius, height, width;
    std::vector<float> H;
    std::vector<cv::Vec2f> HVec;
    int numBins;
    cv::RotatedRect ell;
    cv::Mat frame;
    cv::Mat grayFrame;
};

#endif //CYLINDERPOSETRACKING_EV_H
