#include"ED.h"
#include "CED.h"
#define M_PI 3.14159265358979323846
#define M_1_2_PI 1.57079632679489661923
#define M_180_PI 0.0174532925 // PI/180
cv::Vec3f CED::interiorRate(const std::vector<cv::Point> &points, cv::RotatedRect &ell) {
    int a = ell.size.width / 2.0, b = ell.size.height / 2.0;
    if(std::max(a, b)> std::max(height, width) || std::min(a, b) < 5) {
        return cv::Vec3f(0, 0, 0);
    }
    double L = CV_PI * (3 * (a + b) - sqrt((3 * a + b) * (a + 3 * b)));
    double theta = ell.angle;

    float _cos = cos(-theta*CV_PI/180);
    float _sin = sin(-theta*CV_PI/180);

    float invA2 = 1.f / (a * a);
    float invB2 = 1.f / (b * b);

    auto count_on_ellipse=[ell,_cos,_sin,invA2,invB2,this](const std::vector<cv::Point> &points)->int
    {
        int counter_on_perimeter = 0;
        for (auto p : points)
        {
            auto tp = cv::Point2f(p) - ell.center;
            float rx = (tp.x * _cos - tp.y * _sin);
            float ry = (tp.x * _sin + tp.y * _cos);
            float h = (rx * rx) * invA2 + (ry * ry) * invB2;
            //float d=norm(tp)*(1-1/sqrt(h));
            float d2 = (tp.x * tp.x + tp.y * tp.y) * (h * h * 0.25 - h * 0.5 + 0.25);//approx of above

            if (d2 < inlier_dis)
            {
                ++counter_on_perimeter;
            }
        }
        return counter_on_perimeter;
    };

    double ct = count_on_ellipse(points);

    return cv::Vec3f(ct, ct / points.size(), ct / L);
}

cv::Vec3f CED::interiorRate(const std::vector<int> &ids, cv::RotatedRect &ell) {
    int a = ell.size.width / 2.0, b = ell.size.height / 2.0;
    double r = 1.0 * a / b;
    if(std::max(a, b)> std::max(height, width)
    || std::min(a, b) < 2
    || r < 0.2 || r > 5) {
        return cv::Vec3f(0, 0, 0);
    }

    double L = CV_PI * (3 * (a + b) - sqrt((3 * a + b) * (a + 3 * b)));
    double theta = ell.angle;

    float _cos = cos(-theta*CV_PI/180);
    float _sin = sin(-theta*CV_PI/180);

    float invA2 = 1.0f / (a * a);
    float invB2 = 1.0f / (b * b);

    auto count_on_ellipse=[ell,_cos,_sin,invA2,invB2,this](const std::vector<cv::Point> &points)->int
    {
        double inlier_dis2 = inlier_dis * inlier_dis;
        int counter_on_perimeter = 0;
        for (auto p : points)
        {
            auto tp = cv::Point2f(p) - ell.center;
            float rx = (tp.x * _cos - tp.y * _sin);
            float ry = (tp.x * _sin + tp.y * _cos);
            float h = (rx * rx) * invA2 + (ry * ry) * invB2;
            //float d=norm(tp)*(1-1/sqrt(h));
            float d2 = (tp.x * tp.x + tp.y * tp.y) * (h * h * 0.25 - h * 0.5 + 0.25);//approx of above

            if (d2 < inlier_dis2)
            {
                ++counter_on_perimeter;
            }
        }
        return counter_on_perimeter;
    };
    double ct = 0;
    int total = 0;
    for(auto i : ids) {
        total += ellArc[i].size();
        ct += count_on_ellipse(ellArc[i]);
    }

    return cv::Vec3f(ct, ct / total, ct / L);
}

bool CED::check(const std::vector<cv::Point>& lv) {
    for(int i = 1; i < lv.size() - 1; i++) {
        if(lv[i - 1].cross(lv[i]) * lv[i].cross(lv[i + 1]) < 0) return false;
    }
    return true;
}
bool CED::canMatch(const int id1, const int id2) {
    const std::vector<cv::Point>& e1 = ellArc[id1];
    const std::vector<cv::Point>& e2 = ellArc[id2];
    const std::vector<std::pair<int, int>>& seg1 = ellArcSeg[id1];
    const std::vector<std::pair<int, int>>& seg2 = ellArcSeg[id2];
    int longerLen = std::max(e1.size(), e2.size());
    int shorterLen = std::min(e1.size(), e2.size());
    int segSize1 = seg1.size();
    int segSize2 = seg2.size();

    cv::Point l1 = e1[seg1[segSize1 - 2].second - 1] - e1[seg1[segSize1 - 2].first];
    cv::Point l2 = e1[seg1[segSize1 - 1].second - 1] - e1[seg1[segSize1 - 1].first];

    cv::Point l3 = e2[seg2[0].first] - e1[seg1[segSize1 - 1].second - 1];

    cv::Point l4 = e2[seg2[0].second - 1] - e2[seg2[0].first];
    cv::Point l5 = e2[seg2[1].second - 1] - e2[seg2[1].first];
    int flag = true;
    if (!check({l1, l2, l3, l4, l5})) {
        cv::Point l3_1 = e2[seg2[0].first] - e1[seg1[segSize1 - 2].second - 1];
        int len_l1 = seg1[segSize1 - 1].second - seg1[segSize1 - 1].first;

        cv::Point l3_2 = e2[seg2[1].first] - e1[seg1[segSize1 - 1].second - 1];
        int len_l4 = seg2[0].second - seg1[0].first;
        bool tmpFlag = false;
        if (segSize1 > 2) {
            cv::Point l6 = e1[seg1[segSize1 - 3].second - 1]
                           - e1[seg1[segSize1 - 3].first];
            if (check({l6, l1, l3_1, l4, l5}) && len_l1 * 3 < e1.size()) tmpFlag = true;
        } else {
            if (check({l1, l3_1, l4, l5}) && len_l1 * 3 < e1.size()) tmpFlag = true;
        }

        if (segSize2 > 2) {
            cv::Point l7 = e2[seg2[2].second - 1]
                           - e2[seg2[2].first];
            if (check({l1, l2, l3_2, l5, l7}) && len_l4 * 3 < e2.size()) tmpFlag = true;
        } else {
            if (check({l1, l2, l3_2, l5}) && len_l4 * 3 < e2.size()) tmpFlag = true;
        }

        std::vector<cv::Point>lv;
        if(segSize1 > 2) {
            cv::Point l6 = e1[seg1[segSize1 - 3].second - 1]
                           - e1[seg1[segSize1 - 3].first];
            lv.emplace_back(l6);
        }
        lv.emplace_back(l1);
        lv.emplace_back(l5);
        if(segSize2 > 2) {
            cv::Point l7 = e2[seg2[2].second - 1]
                           - e2[seg2[2].first];
            lv.emplace_back(l7);
        }
        if(check(lv)) tmpFlag = true;
        flag = tmpFlag;
    }
    return flag;


}
bool CED::canFromWeightedPair(int id1, int id2) {
    const std::vector<cv::Point>& e1 = ellArc[id1];
    const std::vector<cv::Point>& e2 = ellArc[id2];
    const std::vector<std::pair<int, int>>& seg1 = ellArcSeg[id1];
    const std::vector<std::pair<int, int>>& seg2 = ellArcSeg[id2];
    int longerLen = std::max(e1.size(), e2.size());
    int shorterLen = std::min(e1.size(), e2.size());

//    弧段比限制
    if(1.0 * longerLen / shorterLen > th_lsr) {
        return false;
    }

//    距离限制
    const cv::Point& p1m = e1[e1.size() / 2];
    const cv::Point& p2m = e2[e2.size() / 2];
    if(cv::norm(p1m - p2m)  > th_dis * (longerLen + shorterLen) ){
        return false;
    }
//    if(cv::norm(p1m - p2m)  > longerLen ){
//        return false;
//    }

    cv::Point chord_v1 = e1.back() - e1.front();
    if(chord_v1.cross(p1m - e1.front()) * chord_v1.cross(p2m - e1.front()) > 0) {
        return false;
    }

    cv::Point chord_v2 = e2.back() - e2.front();
    if(chord_v2.cross(p1m - e2.front()) * chord_v2.cross(p2m - e2.front()) > 0) {
        return false;
    }


    return canMatch(id1, id2) && canMatch(id2, id1);
}

cv::RotatedRect CED::fit(const std::vector<int>& ids) {
    std::vector<cv::Point>points;
//    fit_ct++;
    points.reserve(12);
    for(auto i : ids) {
        for(auto seg : ellArcSeg[i]) {
            points.emplace_back(ellArc[i][seg.first]);
        }
        points.emplace_back(ellArc[i][ellArcSeg[i].back().second - 1]);
    }
    return fitEllipse(points);
}


