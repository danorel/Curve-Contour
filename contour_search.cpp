//
// Created by Dan Orel on 2019-11-10.
//
#include "contour_search.h"


std::vector<double>
    FindContour(
        const cv::Mat &Img,
        const int k) {
    cv::Mat BoundaryTracedImg = Img.clone();
    // Proving the vector, which will store the fully-connected path
    std::vector<cv::Point> motion;
    // Find the closes element of the image from the right-bottom corner
    motion.push_back(FindClosest(Img));
    while (true) {
        cv::Point temp = motion.at(motion.size() - 1);
        BoundaryTracing(
                Img,
                BoundaryTracedImg,
                temp.x,
                temp.y,
                motion);
        if (
                motion.size() > 2 &&
                motion.at(0).x == motion.at(motion.size() - 2).x &&
                motion.at(0).y == motion.at(motion.size() - 2).y &&
                motion.at(1).x == motion.at(motion.size() - 1).x &&
                motion.at(1).y == motion.at(motion.size() - 1).y) {
            break;
        }
    }
    // Define the vector, which will store the contour angles
    std::vector<double> AngleContourMap;
    double db       = 0.,
           df       = 0.,
           theta_b  = 0.,
           abs_b1   = 0.,
           abs_b2   = 0.,
           theta_f  = 0.,
           abs_f1   = 0.,
           abs_f2   = 0.,
           thetai   = 0.,
           sigma_f  = 0.,
           ki       = 0.;
    int    prev     = 0,
           next     = 0;
    for(int index = 0; index < motion.size(); index++){
        if(index - k < 0){
            prev = motion.size() - (k - index) - 1;
            next = index;
            db = sqrt(pow(motion.at(prev).x - motion.at(next).x, 2) + pow(motion.at(prev).y - motion.at(next).y, 2));
            abs_b1 = abs(motion.at(prev).x - motion.at(next).x);
            abs_b2 = abs(motion.at(prev).y - motion.at(next).y);
        } else {
            prev = index - k;
            next = index;
            db = sqrt(pow(motion.at(prev).x - motion.at(next).x, 2) + pow(motion.at(prev).y - motion.at(next).y, 2));
            abs_b1 = abs(motion.at(prev).x - motion.at(next).x);
            abs_b2 = abs(motion.at(prev).y - motion.at(next).y);
        }
        if(index + k >= motion.size()){
            prev = index + k - motion.size();
            next = index;
            df = sqrt(pow(motion.at(prev).x - motion.at(next).x, 2) + pow(motion.at(prev).y - motion.at(next).y, 2));
            abs_f1 = abs(motion.at(prev).x - motion.at(next).x);
            abs_f2 = abs(motion.at(prev).y - motion.at(next).y);
        } else {
            prev = index + k;
            next = index;
            df = sqrt(pow(motion.at(prev).x - motion.at(next).x, 2) + pow(motion.at(prev).y - motion.at(next).y, 2));
            abs_f1 = abs(motion.at(prev).x - motion.at(next).x);
            abs_f2 = abs(motion.at(prev).y - motion.at(next).y);
        }
        theta_f = atan(abs_f1 / abs_f2);
        theta_b = atan(abs_b1 / abs_b2);
        thetai  = theta_b / 2 + theta_f / 2;
        sigma_f = theta_f - thetai;
        ki      = (sigma_f * (db + df)) / (2 * db * df);
        AngleContourMap.push_back(ki);
    }
    return AngleContourMap;
}


cv::Mat VisualizeContour(
        const std::vector<double> &AngleContourMap,
        const int height,
        const int width) {
    const int size   = 256;
    const int factor = 1000;
    int bin_width = cvRound(static_cast<double>(width / size));
    cv::Mat ContourImage(
            height,
            width,
            CV_8UC1,
            cv::Scalar(255, 255, 255)
    );
    // Draw the intensity line for histogram
    int alignment = static_cast<int>(height / 2);
    for(int i = 0; i < size; i++)
        line(
                ContourImage,
                cv::Point(
                        bin_width * (i),
                        alignment),
                cv::Point(
                        bin_width * (i),
                        alignment - (AngleContourMap.at(i) * factor)),
                cv::Scalar(0,0,0),
                1,
                8,
                0
        );
    return ContourImage;
}