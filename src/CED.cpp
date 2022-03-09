#include"ED.h"
#include "CED.h"
#include "EV.h"
#define M_PI 3.14159265358979323846

CED::CED(cv::Mat srcImage)
        :ED(srcImage, PREWITT_OPERATOR, 11, 3) {
    imgRGB = srcImage.clone();
    std::copy(segmentPoints.begin(), segmentPoints.end(), std::back_inserter(edgeList));
}

void CED::run_CED() {

    run_RDP();

    splitEdge();

    buildWeightedEdge();

    initUnionFind();

    arcMatchingByUnionFind();

    ellipseCluster();

}

void CED::run_RDP() {
    segList.resize(edgeList.size());
    sort(edgeList.begin(), edgeList.end(),
         [&](std::vector<cv::Point> &a, std::vector<cv::Point> &b) {
             return a.size() > b.size();
         });
    parallel_for_(cv::Range(0, threads), Parallel_For_RDP(edgeList.data(), segList.data(), epsilon, (int)edgeList.size(),threads));
}


void CED::splitEdge() {
    double th_angle = cos(sharpAngle * M_PI / 180);

    for(int i = 0; i < segList.size(); i++) {
        if(edgeList[i].size() < minimum_edge_length) break;
        if(segList[i].size() < 2) {
            continue;
        }
        int pre_index = 0;
        int preSegId = 0;
        auto& seg = segList[i];
        for(int j = 1; j < seg.size(); j++) {
            bool isCornerPoint = false, isInflectionPoint = false;
            cv::Point l2_v = edgeList[i][seg[j - 1].second - 1] - edgeList[i][seg[j - 1].first];
            cv::Point l3_v = edgeList[i][seg[j].second - 1] - edgeList[i][seg[j].first];
            double angle = l2_v.dot(l3_v) / (cv::norm(l2_v) * cv::norm(l3_v));
            if(angle <= th_angle) {
                isCornerPoint = true;
            }
            if(j >= 2) {
                cv::Point l1_v = edgeList[i][seg[j - 2].second - 1] - edgeList[i][seg[j - 2].first];
                if (l1_v.cross(l2_v) * l2_v.cross(l3_v) < 0) {
                    isInflectionPoint = true;
                }
            }
            if(isCornerPoint || isInflectionPoint) {
                int len = seg[j].first - pre_index;
                if(len > minimum_edge_length && j - preSegId > 1) {
                    ellArc.emplace_back(std::vector<cv::Point>(edgeList[i].begin() + pre_index,
                                                   edgeList[i].begin() + seg[j].first));
                    std::vector<cv::Point>& arc = ellArc.back();
                    auto v1 = arc[arc.size() / 2] - arc.front();
                    auto v2 = arc.back() - arc[arc.size() / 2];
                    if(v1.cross(v2) > 0) {
                        std::reverse(arc.begin(), arc.end());
                    }
                }
                pre_index = seg[j].first;
                preSegId = j;
            }
        }
        int len = seg.back().second - pre_index;
        if (len >= minimum_edge_length && seg.size() - preSegId > 1) {
            ellArc.emplace_back(std::vector<cv::Point>(edgeList[i].begin() + pre_index,
                                                    edgeList[i].begin() + seg.back().second));
            std::vector<cv::Point>& arc = ellArc.back();
            auto v1 = arc[arc.size() / 2] - arc.front();
            auto v2 = arc.back() - arc[arc.size() / 2];
            if(v1.cross(v2) > 0) {
                std::reverse(arc.begin(), arc.end());
            }
        }
    }
    ellArcSeg.resize(ellArc.size());
    sort(ellArc.begin(), ellArc.end(),
         [&](std::vector<cv::Point> &a, std::vector<cv::Point> &b) {
             return a.size() > b.size();
         });
    parallel_for_(cv::Range(0, threads), Parallel_For_RDP(ellArc.data(), ellArcSeg.data(), epsilon, (int)ellArc.size(),threads));

}




void CED::buildWeightedEdge() {
    setNodes.resize(ellArc.size());
    for(int i = 0; i < ellArc.size(); i++) {
        if(ellArcSeg[i].size() < 2) {
            continue;
        }
        for(int j = i + 1; j < ellArc.size(); j++) {
            if(ellArcSeg[j].size() < 2) {
                continue;
            }
            if(canFromWeightedPair(i, j)) {
                cv::RotatedRect ell = fit({i, j});
                const std::vector<int>tmp = {j};
                cv::Vec3f score = interiorRate(tmp, ell);
                if(score[1] > minimum_edge_score) {
                    setNodes[i].wq.emplace_back(WeightedEdge(j, score));
                }
            }
        }
    }
}


void CED::ellipseCluster() {

    for(int i = 0; i < setNodes.size(); i++) {
        if(setNodes[i].bro.size() > 0 && setNodes[i].score[2] > minimum_ellipse_score2) {
            ellipseList.emplace_back(setNodes[i].ell);
            ellipse_score.emplace_back(setNodes[i].score);
            ellipseArcId.emplace_back(setNodes[i].set_elements);
        }

        if(i < ellArcSeg.size() && ellArcSeg[i].size() >= 5) {
            std::vector<int>tmp = {i};
            cv::RotatedRect ell = fit(tmp);
            cv::Vec3f score = interiorRate(tmp, ell);
            if(score[1] > minimum_ellipse_score1 && score[2] > minimum_ellipse_score2) {
                ellipseList.emplace_back(ell);
                ellipse_score.emplace_back(score);
                ellipseArcId.emplace_back(tmp);
            }
        }
    }

    std::vector<float>ellScore;
    cv::Mat2f direction(smoothImage.size(),cv::Vec2f(0,0));

    cv::Mat1s dx,dy;
    Sobel(smoothImage,dx,CV_16S,1,0, 3);
    Sobel(smoothImage,dy,CV_16S,0,1, 3);
    for (int i = 0; i < smoothImage.rows; ++i) {
        for (int j = 0; j < smoothImage.cols; ++j) {
            double len=sqrt(dx(i,j)*dx(i,j)+dy(i,j)*dy(i,j));
            if (len!=0)
            {
                direction(i, j)=cv::Point2f(dx(i,j)/len,dy(i,j)/len);
            }
            else
                direction(i, j) = cv::Point2f(0, 0);
        }
    }
    std::vector<std::vector<int>>remainId;
    std::vector<std::vector<double>> remainScore;
    remainId.resize(threads);
    remainScore.resize(threads);
    parallel_for_(cv::Range(0, threads),
                  Parallel_For_ComputeMatchScore(ellipseList.data(),remainId.data(), remainScore.data(), direction, sampleNum, radius, width, height, remain_score, (int)ellipseList.size(), threads));


    for(int i = 0; i < remainId.size(); i++) {
        for(int j = 0; j < remainId[i].size(); j++) {
            int id = remainId[i][j];
            double score = remainScore[i][j];
            clustered_ellipse.emplace_back(ellipseList[id]);
            clustered_ellipseArcId.emplace_back(ellipseArcId[id]);
            clustered_ellipse_score.emplace_back(ellipse_score[id]);
            ellScore.emplace_back(score);
        }
    }
    std::vector<int>inDegree(clustered_ellipse.size(), 0);
    for(int i = 0; i < clustered_ellipse.size(); i++) {
        cv::RotatedRect& ell1 = clustered_ellipse[i];
        for(int j = i + 1; j < clustered_ellipse.size(); j++) {
            cv::RotatedRect& ell2 = clustered_ellipse[j];
            float diff = sqrt(pow(ell1.center.x - ell2.center.x, 2) + pow(ell1.center.y - ell2.center.y, 2)
                    + pow(ell1.size.height - ell2.size.height, 2) + pow(ell1.size.width - ell2.size.width, 2));
            if(diff < cluster_dis){
                float s1 = CV_PI * ell1.size.width * ell1.size.height;
                float s2 = CV_PI * ell2.size.width * ell2.size.height;
                if(ellScore[j] < ellScore[i]) {
                    inDegree[j]++;
                }
                else {
                    inDegree[i]++;
                }
            }
        }
    }
    std::vector<cv::RotatedRect> tmp;
    for(int i = 0; i < clustered_ellipse.size(); i++) {
        if(inDegree[i] == 0) {
            tmp.emplace_back(clustered_ellipse[i]);
        }
    }
    clustered_ellipse = tmp;
}

