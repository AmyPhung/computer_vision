////
//// Created by amy on 10/28/20.
////
//
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//
//#include <sstream>
//#include <opencv2/opencv.hpp>
//#include <iostream>
//
//#include <opencv2/xfeatures2d.hpp>
//
//using namespace cv;
//using namespace std;
//
int main(int argc, char** argv) {
    return 0;
}
//    CommandLineParser parser( argc, argv, keys );
//    // Read the image file
//    Mat img1 = imread("/home/amy/robo_ws/src/cv_project/img/view1.jpg", IMREAD_GRAYSCALE );
//    Mat img2 = imread("/home/amy/robo_ws/src/cv_project/img/view2.jpg", IMREAD_GRAYSCALE );
//
//    if ( img1.empty() || img2.empty() )
//    {
//        cout << "Could not open or find the image!\n" << endl;
//        parser.printMessage();
//        cin.get(); //wait for any key press
//        return -1;
//    }
//
//    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
//    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create( minHessian );
//    std::vector<KeyPoint> keypoints1, keypoints2;
//    Mat descriptors1, descriptors2;
//    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
//    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );
//
////    // detectingkeypoints
////    SurfFeatureDetectordetector();
////    vector<KeyPoint> keypoints1, keypoints2;
////    detector.detect(img1, keypoints1);
////    detector.detect(img2, keypoints2);
////    // computing descriptors
////    SurfDescriptorExtractor extractor;
////    Mat descriptors1, descriptors2;
////    extractor.compute(img1, keypoints1, descriptors1);
////    extractor.compute(img2, keypoints2, descriptors2);
////    // matching descriptors
////    BruteForceMatcher<L2<float>> matcher;
////    vector<DMatch> matches;
////    matcher.match(descriptors1, descriptors2, matches);
//
////
////
////    String windowName = "The Guitar"; //Name of the window
////
////    namedWindow(windowName); // Create a window
////
////    imshow(windowName, image); // Show our image inside the created window.
////
////    waitKey(0); // Wait for any keystroke in the window
////
////    destroyWindow(windowName); //destroy the created window
//
//    return 0;
//}