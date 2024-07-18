//
// Created by xiaowuga on 2021/10/13.
//

#ifndef CYLINDERPOSETRACKING_CED_H
#define CYLINDERPOSETRACKING_CED_H

#include"ED.h"



class CED : public ED {

public:
    struct WeightedEdge {
        int id;
        cv::Vec3f score;
        WeightedEdge(int _id, const cv::Vec3f& _score) : id(_id), score(_score){}
        bool operator < (const WeightedEdge& e) const {
            return score[0] > e.score[0];
        }
    };
    struct SetNode {
        int rootId, broRootId;
        std::vector<WeightedEdge> wq;
        std::vector<int>set_elements;
        cv::Vec3f score;
        std::vector<int> bro;
        cv::RotatedRect ell;
        SetNode() {}
        SetNode(int _rootId, int _broRootId, const std::vector<int>& _set_elements,
                const cv::RotatedRect& _ell, const cv::Vec3f& _score)
        : rootId(_rootId), broRootId(_broRootId), set_elements(_set_elements),
        ell(_ell),score(_score){}
    };

public:
    CED(cv::Mat srcImage);
    void run_CED();
    std::vector<cv::RotatedRect> getEllipses() { return  ellipseList;}
    std::vector<cv::RotatedRect> getEllipsesAfterCluster() { return  clustered_ellipse;}
//    draw
    cv::Mat drawEdgeContours();
    cv::Mat drawEdgeSegments();
    cv::Mat drawEdgeById(const std::vector<int>& ids);
    cv::Mat drawEllArc();
    cv::Mat drawArcDirection();
    cv::Mat drawEllipses(const cv::Mat& img);
    void CED::drawEllipseOneByOne(const cv::Mat& img);
    cv::Mat drawEllipsesAfterCluster(const cv::Mat& img);
    cv::Mat drawEdgeSegmentsAfterSplit();

//    algo
    void run_RDP();
    void splitEdge();
    void buildWeightedEdge();
    void initUnionFind();
    bool canFromWeightedPair(int id1, int id2);
    void arcMatchingByUnionFind();
    void ellipseCluster();

//    tools
    int findRoot(int k);
    bool check(const std::vector<cv::Point>& lv);
    bool canMatch(const int id1, const int id2);
    cv::RotatedRect fit(const std::vector<int>& ids);
    cv::Vec3f interiorRate(const std::vector<cv::Point>& points, cv::RotatedRect& ell);
    cv::Vec3f interiorRate(const std::vector<int> &ids, cv::RotatedRect &ell);
    bool canMerge(int id1, int id2);


public:
    float sharpAngle = 70;
    double minimum_edge_length = 15;
    double minimum_edge_score = 0.35;
    double th_lsr = 3, th_dis = 3;
    double minimum_ellipse_score1 = 0.3;
    double minimum_ellipse_score2 = 0.35;
    double remain_score = 0.65;
    double inlier_dis = 1.5;
    double cluster_dis = 10;
    int sampleNum = 720, radius = 3;
    double epsilon = 0.8;
    int threads = 8;

private:
    std::vector<std::vector<cv::Point>> edgeList;
    std::vector<std::vector<std::pair<int, int>>> segList;
    std::vector<std::vector<cv::Point>> ellArc;
    std::vector<std::vector<std::pair<int, int>>> ellArcSeg;

public:
    std::vector<SetNode> setNodes;

//  ellipse list
    std::vector<cv::Vec3f> ellipse_score;
    std::vector<cv::RotatedRect> ellipseList;
    std::vector<std::vector<int>> ellipseArcId;

    std::vector<cv::Vec3f> clustered_ellipse_score;
    std::vector<cv::RotatedRect> clustered_ellipse;
    std::vector<std::vector<int>> clustered_ellipseArcId;

    cv::Mat imgRGB;

};
class Parallel_For_RDP : public cv::ParallelLoopBody {
private:
    float _epsilon;
    int _threads, range, _num;
    std::vector<cv::Point>* _edgeLists;
    std::vector<std::pair<int, int>>* _segLists;
public:
    Parallel_For_RDP(std::vector<cv::Point>* edgeLists, std::vector<std::pair<int, int>>* segLists, float epsilon, int num, int threads) {
        _edgeLists = edgeLists;

        _segLists = segLists;

        _epsilon = epsilon;

        _threads = threads;

        _num = num;

        range = num / _threads;
    }
    static double PerpendicularDistance2(const cv::Point& pt, const cv::Point& lineStart, const cv::Point& lineEnd) {
        double dx = lineEnd.x - lineStart.x;
        double dy = lineEnd.y - lineStart.y;

        //Normalise
        double mag = pow(pow(dx, 2.0) + pow(dy, 2.0), 0.5);
        if (mag > 0.0)
        {
            dx /= mag; dy /= mag;
        }

        double pvx = pt.x - lineStart.x;
        double pvy = pt.y - lineStart.y;

        //Get dot product (project pv onto normalized direction)
        double pvdot = dx * pvx + dy * pvy;

        //Scale line direction vector
        double dsx = pvdot * dx;
        double dsy = pvdot * dy;

        //Subtract this from pv
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        return ax * ax + ay * ay;
    }

    void RDP(const std::vector<cv::Point>& edge, int l, int r, double epsilon, int id) const{
        if(r - l < 2) {
            return;
        }
        double dMax = 0;
        int idx = 0;
        for(size_t i = l + 1; i < r; i++) {
            double d = PerpendicularDistance2(edge[i], edge[l], edge[r - 1]);
            if(d > dMax) {
                idx = i; dMax = d;
            }
        }
        if(dMax > epsilon) {
            RDP(edge, l, idx + 1, epsilon, id);
            RDP(edge, idx, r, epsilon, id);
        } else {
            _segLists[id].emplace_back(std::make_pair(l, r));
        }

    }

    virtual void operator()(const cv::Range &r) const {
        for(int v = r.start; v < _num; v += _threads){
            if (_edgeLists[v].size() > 2) {
                _segLists[v].reserve(10);
                RDP(_edgeLists[v], 0, (int)_edgeLists[v].size(), _epsilon * _epsilon, v);
            }
        }
    }
};

class Parallel_For_ComputeMatchScore : public cv::ParallelLoopBody {
private:
    int _sampleNum, _radius;
    double _remain_score;
    int _threads;
    cv::RotatedRect* _ellipseList;
    cv::Mat2f _direction;
    std::vector<int>* _remainId;
    std::vector<double>* _remainScore;
    std::vector<double> sin_alpha;
    std::vector<double> cos_alpha;
    int _width, _height, _nums;
public:
    Parallel_For_ComputeMatchScore(cv::RotatedRect* ellipseList,
                                   std::vector<int>* remainId,
                                   std::vector<double>* remainScore,
                                   const cv::Mat2f direction,
                                   int sampleNum, int radius,
                                   int width, int height,
                                   double remain_score,
                                   int nums, int threads) {
        _ellipseList = ellipseList;
        _remainId = remainId;
        _remainScore = remainScore;
        _sampleNum = sampleNum;
        _radius = radius;
        _threads = threads;
        _direction = direction;
        _remain_score = remain_score;

        sin_alpha.resize(sampleNum);
        cos_alpha.resize(sampleNum);

        _width = width;
        _height = height;
        _nums = nums;
        for(int i = 0; i < sampleNum; i++) {
            double rad = i * CV_2PI / sampleNum;
            sin_alpha[i] = sin(rad);
            cos_alpha[i] = cos(rad);
        }
    }

    virtual void operator()(const cv::Range& r) const{
        int range = _nums/_threads;

        int vEnd = r.end*range;
        if(r.end == _threads) {
            vEnd = _nums;
        }
        std::vector<int>* id = &_remainId[r.start];
        std::vector<double>* score = &_remainScore[r.start];

        for(int v = r.start * range; v < vEnd; v++) {
            cv::RotatedRect& ell = _ellipseList[v];
            float b = ell.size.height * 0.5;
            float a = ell.size.width * 0.5;
            float theta = -ell.angle * CV_PI / 180.0;
            float sin_theta = sin(theta), cos_theta = cos(theta);
            float invA2 = 1.f / (a * a);
            float invB2 = 1.f / (b * b);
            std::vector<cv::Vec2f> imgGrad(_sampleNum);
            std::vector<cv::Vec2f> ellGrad(_sampleNum);
            for(int j = 0; j < _sampleNum; j++) {
                float cosa = a * cos_alpha[j], cosb = b * sin_alpha[j];
                int x_i = cos_theta * cosa + sin_theta * cosb + ell.center.x;
                int y_i = -sin_theta * cosa + cos_theta * cosb + ell.center.y;
                if(x_i < 0 || x_i >= _width || y_i < 0 || y_i >= _height) {
                    imgGrad[j] = cv::Vec2f(1, 0);
                    ellGrad[j] = cv::Vec2f(1, 0);
                    continue;
                }
                cv::Point p(x_i, y_i);
                imgGrad[j] = _direction(p);

                cv::Point tp = cv::Point2f(p) - ell.center;
                float rx = (tp.x * cos_theta - tp.y * sin_theta);
                float ry = (tp.x * sin_theta + tp.y * cos_theta);
                cv::Vec2f rdir(2 * rx * cos_theta * invA2 + 2 * ry * sin_theta * invB2, 2 * rx * -sin_theta * invA2 + 2 * ry * cos_theta * invB2);
                rdir /= cv::norm(rdir);
                ellGrad[j] = rdir;
            }

            int count = 0;
            for(int j = 0; j < _sampleNum; j++) {
                cv::Vec2f cdir = imgGrad[j];
                for(int k = 1; k < _radius; k++) {
                    cdir += imgGrad[(j - k + _sampleNum) % _sampleNum];
                    cdir += imgGrad[(j + k) % _sampleNum];
                }
                cdir /= _radius * 2 - 1;
                cdir /= cv::norm(cdir);
                cv::Vec2f rdir = ellGrad[j];
                double angle = std::abs(std::acos(cdir.dot(rdir))) * 180.0 / CV_PI;
                if (angle > 90)
                    angle = 180 - angle;
                if(angle < 15) count++;

                if(cdir.dot(rdir) < 0) cdir = -cdir;
            }

            double tmp = 1.0 * count / _sampleNum;
            if(tmp > _remain_score) {
                id->emplace_back(v);
                score->emplace_back(tmp);
            }
        }
    }
};

#endif //CYLINDERPOSETRACKING_CED_H
