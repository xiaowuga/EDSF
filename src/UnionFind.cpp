//
// Created by xiaowuga on 2021/10/30.
//
#include"ED.h"
#include "CED.h"


int CED::findRoot(int k) {
    return k == setNodes[k].rootId ? k : findRoot(setNodes[k].rootId);
}

void CED::initUnionFind() {
    int ct = 0;
    for(auto& node : setNodes) {
        node.rootId = ct;
        node.broRootId = ct++;
        if(node.wq.empty()) continue;
        std::sort(node.wq.begin(),  node.wq.end());
//        node.bro.emplace_back(ct - 1);
//        node.set_elements.emplace_back(ct - 1);
    }
}

void CED::arcMatchingByUnionFind() {
    for(int i = 0; i < ellArc.size(); i++) {
        if(!setNodes[i].wq.empty()) {
            for(int j = 0; j < setNodes[i].wq.size(); j++){
                int id = setNodes[i].wq[j].id;

//                cv::Mat pairMap = drawEdgeById({i, id});
//                cv::imshow("pairMap", pairMap);
//                cv::waitKey();

                int r1 = findRoot(i);
                int r2 = findRoot(id);
                if(r1 == r2) continue;
                bool isMerge = false;
                for(auto bro : setNodes[i].bro) {
                    int r = findRoot(bro);
                    if(r2 == r) {
                        isMerge = true;
                        break;
                    }
                    bool isMatch = true;
                    for(auto se : setNodes[r].set_elements) {
                        if(!canMerge(setNodes[se].broRootId, id)) {
                            isMatch = false; break;
                        }
                    }
                    if(isMatch) {
                        std::vector<int> ids(setNodes[r].set_elements);
                        ids.emplace_back(id);
                        cv::RotatedRect ell = fit(ids);
                        cv::Vec3f score = interiorRate(ids, ell);
                        if(score[1] > minimum_ellipse_score1 && score[2] > setNodes[bro].score[2]) {
                            setNodes[bro].set_elements.emplace_back(id);
                            setNodes[bro].ell = ell;
                            setNodes[bro].score = score;
                            setNodes[id].rootId = r;
                            isMerge = true; break;
                        }
                    }
                }
                if(!isMerge) {
                    const std::vector<int>ids = {i, id};
                    cv::RotatedRect ell = fit(ids);
                    cv::Vec3f score = interiorRate(ids, ell);
                    if(score[1] > minimum_ellipse_score1) {
                        if(setNodes[i].bro.empty()) {
                            setNodes[i].bro.emplace_back(i);
                            setNodes[id].rootId = i;
                            setNodes[i].ell = ell;
                            setNodes[i].score = score;
                            setNodes[i].set_elements = {i, id};
                        } else {
                            int roodId = setNodes.size();
                            setNodes[i].bro.emplace_back(roodId);
                            setNodes[id].rootId = roodId;
                            setNodes.emplace_back(SetNode(roodId, i, ids, ell, score));
                            setNodes.back().bro.emplace_back(roodId);
                        }
                    }
                }
            }
        }
    }
}

inline bool CED::canMerge(int id1, int id2) {
    const std::vector<cv::Point>& e1 = ellArc[id1];
    const std::vector<cv::Point>& e2 = ellArc[id2];
    const cv::Point& p1m = e1[e1.size() / 2];
    const cv::Point& p2m = e2[e2.size() / 2];

    cv::Point chord_v1 = e1.back() - e1.front();
    if(chord_v1.cross(p1m - e1.front()) * chord_v1.cross(p2m - e1.front()) > 0) {
        return false;
    }

    cv::Point chord_v2 = e2.back() - e2.front();
    if(chord_v2.cross(p1m - e2.front()) * chord_v2.cross(p2m - e2.front()) > 0) {
        return false;
    }
    return true;
}


