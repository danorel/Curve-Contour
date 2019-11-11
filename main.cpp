#include "contour_search.h"


int main() {
    std::string pathToImage = "/Users/danorel/Workspace/World of C/C++/KMA/MMPI/CurveContour/";
    std::string image = "hand.jpg";

    std::string ImgLabel        = "Original Image",
                ContourImgLabel = "Boundary Tracing Image";

    cv::Mat Img = cv::imread(
            pathToImage + image,
            cv::IMREAD_GRAYSCALE);

    // Make the angle map of the found contour and visualize it
    std::vector<double> AngleContourMap = FindContour(
            Img,
            5);
    cv::Mat ContourImage = VisualizeContour(
            AngleContourMap,
            480,
            640);

    // Show the starter image
    cv::imshow(
            ImgLabel,
            Img);
    cv::imshow(
            ContourImgLabel,
            ContourImage);
    cv::waitKey();
    return 0;
}