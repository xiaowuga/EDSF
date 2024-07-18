#include <iostream>
#include"EDLib.h"
#include "CED.h"
#include <opencv2/core/utils/logger.hpp>
int main() {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    std::string imgPath = "../data/1.jpg";
    cv::Mat img = cv::imread(imgPath);
    CED ced = CED(img);
    ced.run_CED();
    cv::Mat sss =  ced.drawEdgeSegments();
    cv::Mat ellMapCluster = ced.drawEllipsesAfterCluster(img);

    cv::imshow("ellMapCluster", ellMapCluster);
    cv::imwrite("../result.jpg", ellMapCluster);
    cv::waitKey();
    return 0;
}
