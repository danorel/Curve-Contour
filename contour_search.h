//
// Created by Dan Orel on 2019-11-10.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "boundary_tracing.h"
#ifndef CURVECONTOUR_CONTOUR_SEARCH_H
#define CURVECONTOUR_CONTOUR_SEARCH_H
std::vector<double>
    FindContour(
        const cv::Mat &Img,
        int k);

cv::Mat
    VisualizeContour(
        const std::vector<double> &AngleContourMap,
        const int height,
        const int width);
#endif //CURVECONTOUR_CONTOUR_SEARCH_H
