// core opencv libraries
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// feature detection
#include <opencv2/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"

#include <opencv/cv.hpp>

// opencv libraries are in /usr/local/include/opencv2

#include <iostream>

using namespace cv;

int displayImg(Mat img)
{
    imshow("Display window", img);
    int k = waitKey(0);
    return k;
}

// Helpful cv functions
// drawKeypoints(prev_img, prev_keypoints, prev_img);
//  draws keypoints onto a source image for visualizing

int main() {
    // Set up the file paths for images
    std::string prev_img_path = "../img/kinect1.png";
    std::string curr_img_path = "../img/kinect2.png";

    // Load in images as cv::Mat in grayscale
    Mat prev_img = imread(prev_img_path, IMREAD_GRAYSCALE);
    Mat curr_img = imread(curr_img_path, IMREAD_GRAYSCALE);

    if (prev_img.empty())
    {
        std::cout << "Could not read image : " << prev_img_path << std::endl;
        return 1;
    }
    if (curr_img.empty())
    {
        std::cout << "Could not read image : " << curr_img_path << std::endl;
        return 1;
    }

    // Set up data structure for keypoints
    std::vector<KeyPoint> prev_keypoints, curr_keypoints;

    // Detect keypoints in previous and current images using FAST
    // FastFeatureDetector ffd;

    // int minHessian = 400;
    // auto detector = features2d::FAST::create( minHessian );

    // Create a fast feature detector
    Ptr<FastFeatureDetector> ffd = FastFeatureDetector::create();

    // Find the features in each image
    ffd->detect(prev_img, prev_keypoints);
    ffd->detect(curr_img, curr_keypoints);


    // Convert those keypoinits to points
    // Not sure why this is necessary or what's happening under-the-hood
    // std::vector<Point2f> prev_points;
    // std::vector<Point2f> curr_points;
    // KeyPointsToPoints(prev_keypoints, prev_points);
    // KeyPointsToPoints(prev_keypoints, prev_points);
    // KeyPointsToPoints


    // Calculate the optical flow field
    // std::vector<uchar>vstatus; std::vector<float>verror;
    // calcOpticalFlowPyrLK(prev_img, curr_img, pre)


    return 0;
}



// g++ vis_odom.cpp `pkg-config --cflags --libs opencv4`
// g++ vis_odom.cpp -l opencv_xfeatures2d `pkg-config --cflags --libs opencv4` ; ./a.out
