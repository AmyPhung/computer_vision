////
//// Visual Odometry Node
////
/*
 * DONE:
 * Camera calibration
 *
 * TODO:
 *
 */
#include "visual_odometry.h"

VisualOdometryNode::VisualOdometryNode(ros::NodeHandle* nodehandle):nh_(*nodehandle), it_(*nodehandle)
{
    ROS_INFO("Initializing Visual Odometry Node");
    initializePublishers();
}

void VisualOdometryNode::initializePublishers() {
    ROS_INFO("Initializing Subscribers");

    // TODO: Make these topics ROS parameters & attributes of class
    // Provides synchronized raw image and camera info
    // Based on: https://github.com/tu-darmstadt-ros-pkg/hector_vision/blob/kinetic-devel/hector_qrcode_detection/src/qrcode_detection.cpp#L59
    // and https://answers.ros.org/question/290341/correct-way-to-subscribe-to-image-and-camerainfo-topic-using-image_transport-c-in-kinetic/
    camera_subscriber_ = it_.subscribeCamera("camera/rgb/image_raw", 1, &VisualOdometryNode::cameraRawCallback,this);
}
//using namespace cv;
//using namespace cv::xfeatures2d;
void VisualOdometryNode::cameraRawCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info) {
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(cv_bridge::toCvShare(image, "bgr8")->image, keypoints);
    //-- Draw keypoints
    cv::Mat img_keypoints;
    drawKeypoints( cv_bridge::toCvShare(image, "bgr8")->image, keypoints, img_keypoints );
    //-- Show detected (drawn) keypoints
    imshow("SURF Keypoints", img_keypoints );

//    try {
//        cv::imshow("Camera view", cv_bridge::toCvShare(image, "bgr8")->image);
//        cv::waitKey(30);
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
//    }

//    camera_info->K;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_odom");
    ros::NodeHandle nh;

    VisualOdometryNode voNode(&nh);

    cv::namedWindow("Camera view");
    cv::startWindowThread();

    ros::spin();
    cv::destroyWindow("Camera view");
}




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