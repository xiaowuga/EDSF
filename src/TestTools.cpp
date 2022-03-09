//
// Created by xiaowuga on 2021/11/24.
//

#include "TestTools.h"
bool TestOverlap(const cv::Mat1b& gt, const cv::Mat1b& test, float th) {
    float fAND = float(cv::countNonZero(gt & test));
    float fOR = float(cv::countNonZero(gt | test));
    float fsim = fAND / fOR;
    return (fsim >= th);
}

cv::Vec3f ell_evaluate(std::vector<cv::RotatedRect>& ellGt, std::vector<cv::RotatedRect>& ellTest, float th_score, int h, int w) {
    float threshold_overlap = th_score;
    int sz_gt = ellGt.size();
    int sz_test = ellTest.size();

    std::vector<cv::Mat1b> gts(sz_gt);
    std::vector<cv::Mat1b> tests(sz_test);

    for(int i = 0; i < sz_gt; i++) {
        cv::Mat1b tmp(h, w, uchar(0));
        cv::ellipse(tmp, ellGt[i], cv::Scalar(255), -1);
        gts[i] = tmp;
    }

    for(int i = 0; i < sz_test; i++) {
        cv::Mat1b tmp(h, w, uchar(0));
        cv::ellipse(tmp, ellTest[i], cv::Scalar(255), -1);
        tests[i] = tmp;
    }

    cv::Mat1b overlap(sz_gt, sz_test, uchar(0));

    for(int r= 0; r < overlap.rows; r++) {
        for(int c = 0; c < overlap.cols; c++) {
            overlap(r, c) = TestOverlap(gts[r], tests[c], threshold_overlap) ? uchar(255) : uchar(0);
        }
    }

    int counter = 0;
    std::vector<bool>vec_gt(sz_gt, false);
    for(int i = 0; i < sz_test; i++) {
        for(int j = 0; j < sz_gt; j++) {
            if(vec_gt[j]) continue;
            if(overlap(j, i) != 0) {
                vec_gt[j] = true;
                counter++;
                break;
            }
        }
    }
//    std::cout << counter <<std::endl;
    int fn = sz_gt - counter;
    int fp = sz_test - counter;

    float pr = 0.0f;
    float re = 0.0f;
    float fmeasure = 0.0f;

    if(counter == 0) {
        if (fp == 0) {
            pr = 1.f;
            re = 0.f;
            fmeasure = (2.f * pr * re) / (pr + re);
        }
        else {
            pr = 0.f;
            re = 0.f;
            fmeasure = 0.f;
        }
    } else {
        pr = float(counter) / float(counter + fp);
        re = float(counter) / float(counter + fn);
        fmeasure = (2.f * pr * re) / (pr + re);
    }

    return cv::Vec3f(pr, re, fmeasure);
}

