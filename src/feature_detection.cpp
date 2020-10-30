#include <iostream>
#include <cv.hpp>
#include "opencv2/core.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

/*
 * Camera calib for kinect:
 * D = [0.21171973795157867, -0.38281401647972596, -0.010182636971816126, 0.0008744079680358974, 0.0]
 * K = [540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0]
 * R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
 * P = [555.9996948242188, 0.0, 322.44680611515287, 0.0, 0.0, 552.9210205078125, 238.24188986985973, 0.0, 0.0, 0.0, 1.0, 0.0]
 */


int main( int argc, char* argv[] )
{
    Mat img1 = imread("/home/amy/robo_ws/src/computer_vision/img/view1.jpg", IMREAD_GRAYSCALE );
    Mat img2 = imread("/home/amy/robo_ws/src/computer_vision/img/view2.jpg", IMREAD_GRAYSCALE );

    if ( img1.empty() || img2.empty())
    {
        cout << "Could not open or find the image!\n" << endl;
        cout << "Usage: " << argv[0] << " <Input image>" << endl;
        return -1;
    }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );


    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Find camera matrices -------------------------------------------------------------------------------------
    std::vector<Point2f>imgpts1,imgpts2;
    for( unsigned int i = 0; i<good_matches.size(); i++ ){
    // queryIdx is the "left" image
        imgpts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
    // trainIdx is the "right" image
        imgpts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
    }
    // TODO: Replace K with camera info topic
    float k_arr[9] = {540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0};
    Mat_<double> K = cv::Mat(3, 3, CV_32F, k_arr);

    Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)

    // Decompose essential matrix into rotation and translation elements --------------------------------------------
    SVD svd(E);
    Matx33d W(0,-1,0,//HZ 9.13
              1,0,0,
              0,0,1);
    Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19
    Mat_<double> t = svd.u.col(2); //u3
    Matx34d P1( R(0,0),R(0,1), R(0,2), t(0),
                R(1,0),R(1,1), R(1,2), t(1),
                R(2,0),R(2,1), R(2,2), t(2));


    cout << t << endl;

    //    std::vector<DMatch> n_first_matches(good_matches.begin(), good_matches.begin() + 10);

    //-- Draw matches
    Mat img_matches;
    drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    imshow("Good Matches", img_matches );
    waitKey();
    return 0;


//    //-- Draw keypoints
//    Mat img_keypoints;
//    drawKeypoints( img1, keypoints, img_keypoints );
//    //-- Show detected (drawn) keypoints
//    imshow("SURF Keypoints", img_keypoints );
//    waitKey();
//    return 0;
}