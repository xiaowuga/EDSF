//
// Created by xiaowuga on 2021/10/27.
//

#include"ED.h"
#include "CED.h"


cv::Mat CED::drawEdgeContours() {
    cv::Mat edgeMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < edgeList.size(); ++i) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar SegEdgesColor = cv::Scalar(b, g, r);
        for (int j = 0; j < edgeList[i].size() - 1; ++j) {
            cv::line(edgeMap, edgeList[i][j], edgeList[i][j + 1], SegEdgesColor, 1);//Scalar(0, 0, 0)
        }
    }
    return edgeMap;
}
cv::Mat CED::drawEdgeSegments() {
    cv::Mat edgeSegMap(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < segList.size(); ++i) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar color = cv::Scalar(b, g, r);
        for (int j = 0; j < segList[i].size(); ++j) {
            cv::Point st = edgeList[i][segList[i][j].first];
            cv::Point ed = edgeList[i][segList[i][j].second - 1];
            cv::line(edgeSegMap, st, ed, color, 1);//Scalar(0, 0, 0)
        }
    }
    return edgeSegMap;
}

cv::Mat CED::drawEdgeSegmentsAfterSplit() {
    cv::Mat edgeSegMapAftersplit(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < ellArcSeg.size(); ++i) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar color = cv::Scalar(b, g, r);
        for (int j = 0; j < ellArcSeg[i].size(); ++j) {
            cv::Point st = ellArc[i][ellArcSeg[i][j].first];
            cv::Point ed = ellArc[i][ellArcSeg[i][j].second - 1];
            cv::line(edgeSegMapAftersplit, st, ed, color, 1);//Scalar(0, 0, 0)
        }
    }
    return edgeSegMapAftersplit;
}


cv::Mat CED::drawEllArc()  {
    cv::Mat test3(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int es1 = 0; es1 < ellArc.size(); ++es1) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar SegEdgesColor = cv::Scalar(b, g, r);
        for (int es2 = 0; es2 < ellArc[es1].size() - 1; ++es2) {
            cv::line(test3, ellArc[es1][es2], ellArc[es1][es2 + 1], SegEdgesColor, 2);//Scalar(0, 0, 0)
        }
    }
    return test3;
}

cv::Mat CED::drawEdgeById(const std::vector<int> &ids) {
    cv::Mat test5(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for(auto i : ids) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar SegEdgesColor = cv::Scalar(b, g, r);
        for (int es2 = 0; es2 < ellArc[i].size() - 1; ++es2) {
            cv::line(test5, ellArc[i][es2], ellArc[i][es2 + 1], SegEdgesColor, 2);//Scalar(0, 0, 0)
        }
    }
    return test5;
}

cv::Mat CED::drawArcDirection() {
    cv::Mat test5(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for(auto arc : ellArc) {
        cv::Mat test5(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar SegEdgesColor = cv::Scalar(b, g, r);
        for (int es2 = 0; es2 < arc.size() - 1; ++es2) {
            cv::line(test5, arc[es2], arc[es2 + 1], SegEdgesColor, 1);//Scalar(0, 0, 0)
        }
        circle(test5,arc.back(),5, cv::Scalar(0, 0, 0), -1);

    }
    return test5;
}

void CED::drawEllipseOneByOne(const cv::Mat& img) {
    for(int i = 0; i < ellipseList.size(); i++) {
        cv::Mat test = img.clone();
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::ellipse(test, ellipseList[i], cv::Scalar(b, g, r), 2);
        for(auto id : ellipseArcId[i]) {
            for(int j = 0; j < ellArc[id].size() - 1; j++) {
                cv::line(test, ellArc[id][j], ellArc[id][j + 1], cv::Scalar(0, 255, 0),2);
            }
        }
        std::cout << ellipse_score[i] << std::endl;
        cv::imshow("drawEllipseOneByOne", test);
        cv::waitKey();
    }
}

cv::Mat CED::drawEllipses(const cv::Mat& img) {
    if(img.size != srcImage.size) {
        std::cerr <<  "CED::CED::drawEllipses: "
        << "img.size != srcImage.size" << std::endl;
        return img;
    }
//    std::cout << ellipseList.size() << std::endl;
    cv::Mat tmp = img.clone();
    for(auto e : ellipseList) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar ellipseColor = cv::Scalar(0, 0, 255);
//        cv::Scalar ellipseColor = cv::Scalar(0, 255, 0);
        ellipse(tmp, e, ellipseColor, 2);
    }
    return tmp;
}

cv::Mat CED::drawEllipsesAfterCluster(const cv::Mat &img) {
    if(img.size != srcImage.size) {
        std::cerr <<  "CED::drawEllipsesAfterCluster: "
                  << "img.size != srcImage.size" << std::endl;
        return img;
    }
    cv::Mat tmp = img.clone();
//    std::cout << clustered_ellipse.size() << std::endl;
    int ct = 0;
    std::cout << clustered_ellipse.size() << std::endl;
    for(auto e : clustered_ellipse) {
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        cv::Scalar ellipseColor = cv::Scalar(0, 255, 0);
        ellipse(tmp, e, ellipseColor, 2);
//        std::cout << clustered_ellipse_score[ct++] <<std::endl;
//        cv::imshow("tmp", tmp);
//        cv::waitKey();
    }
    return tmp;
}

