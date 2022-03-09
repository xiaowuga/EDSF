//
// Created by xiaowuga on 2021/11/22.
//

#include "EV.h"
#define M_1_2_PI 1.57079632679489661923
#define M_180_PI 0.0174532925 // PI/180
#define M_PI_180 57.2957795

//void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2)
//{
//    double angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
//    double hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    // Here we lengthen the arrow by a factor of scale
//    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
//    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
//    line(img, p, q, colour, 1, cv::LINE_AA);
//    // create the arrow hooks
//    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
//    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
//    line(img, p, q, colour, 1, cv::LINE_AA);
//    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
//    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
//    line(img, p, q, colour, 1, cv::LINE_AA);
//}

ELLIPSE_VALIDATE::ELLIPSE_VALIDATE(cv::RotatedRect &_ell,cv::Mat& _frame, cv::Mat1s* _dx, cv::Mat1s* _dy,
                                   int _numBins, int _radius,  int _height, int _width) :
numBins(_numBins),  radius(_radius), ell(_ell), height(_height), width(_width){
        xc = ell.center.x;
        yc = ell.center.y;
        b = ell.size.height * 0.5;
        a = ell.size.width * 0.5;
        float theta = -ell.angle * CV_PI / 180.0;
        sin_theta = sin(theta), cos_theta = cos(theta);

        dx = _dx;
        dy = _dy;
        invA2 = 1.0f / (a * a);
        invB2 = 1.0f / (b * b);

        H.resize(numBins);
        HVec.resize(numBins);
        float tmp = CV_2PI / 2 *  numBins;
        for(int j = 0; j < numBins; ++j) {
            float rad = j * CV_2PI /  numBins + tmp;
            HVec[j] = cv::Vec2f(cos(rad), sin(rad));
        }
        frame = _frame.clone();
}


bool ELLIPSE_VALIDATE::calc_grad_mag_ori(int r, int c, double &mag, double &ori) {
    if(r > 0 && r < height - 1 && c > 0 && c < width - 1) {
        double cdx = dx->at<short>(r, c);
        double cdy = dy->at<short>(r, c);
        mag = sqrt(cdx * cdx + cdy * cdy);
        ori = atan2(cdy, cdx);
        return true;
    }else {
        return false;
    }
}

int ELLIPSE_VALIDATE::computeDirection(cv::Point &center) {
    int r = center.y, c = center.x;
    std::fill(H.begin(), H.end(), 0);
    for(int i = -radius; i <= radius; i++) {
        for(int j = -radius; j <= radius; j++) {
            double mag, ori;
            if(calc_grad_mag_ori(r + i, c + j, mag, ori)) {
//                double w = exp(-(i * i + j * j) / 2.0f);
                double w = 1.0;
                int bin = cvRound(numBins * (ori + CV_PI) / CV_2PI);
                bin = (bin < numBins) ? bin : 0;
                H[bin] += w * mag;
            }
        }
    }
    float maxx = 0;
    int id = -1;
    for(int i = 0; i < numBins; i++) {
        if(H[i] > maxx) {
            maxx = H[i];
            id = i;
        }
    }
    return id;
}


void ELLIPSE_VALIDATE::reset() {
    std::fill(H.begin(), H.end(), 0);
}

double ELLIPSE_VALIDATE::computeEnergy() {
    float ct = 0;
    int sampleNum = 360;
    for(int i = 0; i < sampleNum; i++) {
        float rad = i * CV_2PI / sampleNum;
        cv::Mat tmp = frame.clone();
//        cv::ellipse(tmp, ell, cv::Scalar(0,255,0), 2);
        float cosa = a * cos(rad), cosb = b * sin(rad);
        float x_i = cos_theta * cosa + sin_theta * cosb + ell.center.x;
        float y_i = -sin_theta * cosa + cos_theta * cosb + ell.center.y;
        cv::Point p = cv::Point(x_i, y_i);
        cv::Point tp = cv::Point2f(p) - ell.center;
        float rx = (tp.x * cos_theta - tp.y * sin_theta);
        float ry = (tp.x * sin_theta + tp.y * cos_theta);
        cv::Vec2f rdir(2 * rx * cos_theta * invA2 + 2 * ry * sin_theta * invB2,
                       2 * rx * -sin_theta * invA2 + 2 * ry * cos_theta * invB2);
        rdir /= cv::norm(rdir);
        if(p.x < 0 || p.x >= width || p.y < 0 ||p.y >= height) {
            continue;
        }
        int binNum = computeDirection(p);
        cv::Vec2f cdir = HVec[binNum];

        double angle = std::abs(std::acos(cdir.dot(rdir))) * 180.0 / CV_PI;
        if (angle > 90)
            angle = 180 - angle;
        if(angle < 15) ct++;

//        if(cdir.dot(rdir) < 0) cdir = -cdir;
//        drawAxis(tmp, p, p + cv::Point( 200 * cdir), cv::Scalar(0, 0, 255), 0.2);
//        drawAxis(tmp, p, p + cv::Point( 200 * rdir), cv::Scalar(0, 255, 0), 0.2);

//        if(i == 0) {
//            cv::imshow("cir", tmp);
//            cv::waitKey(0);
//        }
    }
//    std::cout << 1.0f * ct / sampleNum << std::endl;
    return 1.0f * ct / sampleNum;
}