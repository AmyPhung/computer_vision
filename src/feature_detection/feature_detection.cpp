#include <iostream>

// One of these is for RANSAC
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/photo.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/core.hpp"

// Core OpenCV libraries
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

// ROS Stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

/*
 * Camera calib for kinect:
 * D = [0.21171973795157867, -0.38281401647972596, -0.010182636971816126, 0.0008744079680358974, 0.0]
 * K = [540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0]
 * R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
 * P = [555.9996948242188, 0.0, 322.44680611515287, 0.0, 0.0, 552.9210205078125, 238.24188986985973, 0.0, 0.0, 0.0, 1.0, 0.0]
 */

bool CheckCoherentRotation(cv::Mat_<double>& R) {
    if(fabsf(determinant(R))-1.0 > 1e-07) {
        cerr<<"det(R) != +-1.0, this is not a rotation matrix"<<endl;
        return false;
    }
    return true;
}

Mat_<double> LinearLSTriangulation(
        Point3d u,//homogenous image point (u,v,1)
        Matx34d P,//camera 1 matrix
        Point3d u1,//homogenous image point in 2nd camera
        Matx34d P1//camera 2 matrix
) {
//build A matrix
    Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2)
    );
//build B vector
    Matx41d B(-(u.x*P(2,3)-P(0,3)),
              -(u.y*P(2,3)-P(1,3)),
              -(u1.x*P1(2,3)-P1(0,3)),
              -(u1.y*P1(2,3)-P1(1,3)));
//solve for X
    Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);
    return X;
}
//
//void TriangulatePoints(
//        const vector<KeyPoint>& pt_set1,
//        const vector<KeyPoint>& pt_set2,
//        const Mat& K,
//        const Mat& Kinv,
//        const Matx34d& P,
//        const Matx34d& P1,
//        vector<Point3d>& pointcloud) {
//    vector<double> reproj_error;
//    for (unsigned int i=0; i<pt_set1.size(); i++) {
////convert to normalized homogeneous coordinates
//        Point2f kp = pt_set1[i].pt;
//        Point3d u(kp.x,kp.y,1.0);
//        Mat_<double> um = Kinv * Mat_<double>(u);
//        u = um.at<Point3d>(0);
//        Point2f kp1 = pt_set2[i].pt;
//        Point3d u1(kp1.x,kp1.y,1.0);
//        Mat_<double> um1 = Kinv * Mat_<double>(u1);
//        u1 = um1.at<Point3d>(0);
//
////triangulate
//        Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
//
////calculate reprojection error (breaks for some reason? Matrix sizes don't agree
////        Mat_<double> xPt_img = K * Mat(P1) * X;
////        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
////        reproj_error.push_back(norm(xPt_img_-kp1));
//
////store 3D point
//        pointcloud.push_back(Point3d(X(0),X(1),X(2)));
//    }
////return mean reprojection error
////    Scalar me = mean(reproj_error);
////    return me[0];
//}
//
//// Modified triangulate points function that also finds color of each keypoint
//void TriangulatePointsWithColor(
//        const vector<KeyPoint>& pt_set1,
//        const vector<KeyPoint>& pt_set2,
//        const Mat& K,
//        const Mat& Kinv,
//        const Matx34d& P,
//        const Matx34d& P1,
//        vector<Point3d>& pointcloud,
//        vector<Vec3b>& colors,
//        const Mat& color_img) {
//    vector<double> reproj_error;
//    for (unsigned int i=0; i<pt_set1.size(); i++) {
//        //convert to normalized homogeneous coordinates
//        Point2f kp = pt_set1[i].pt;
//        Point3d u(kp.x,kp.y,1.0);
//        Mat_<double> um = Kinv * Mat_<double>(u);
//        u = um.at<Point3d>(0);
//        Point2f kp1 = pt_set2[i].pt;
//        Point3d u1(kp1.x,kp1.y,1.0);
//        Mat_<double> um1 = Kinv * Mat_<double>(u1);
//        u1 = um1.at<Point3d>(0);
//
//        //triangulate
//        Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
//
//
////calculate reprojection error (breaks for some reason? Matrix sizes don't agree
//        cout << K << endl;
//        cout << Mat(P1) << endl;
//        cout << X << endl;
//        Mat_<double> xPt_img = X * K * Mat(P1); // TODO what???
//        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
//        cout << norm(xPt_img_-kp1) << endl;
//        reproj_error.push_back(norm(xPt_img_-kp1));
//
////store 3D point
//
////return mean reprojection error
////    Scalar me = mean(reproj_error);
////    return me[0];
//
//
//        //store 3D point
//        pointcloud.push_back(Point3d(X(0),X(1),X(2)));
//
//        // Extract color info from image 1
//        Vec3b bgrPixel = color_img.at<Vec3b>(kp.y, kp.x); // Extracts in row, column (y, x) order
//        colors.push_back(bgrPixel);
//    }
//}

void FindCameraMatrices(const Mat& K,
                        const vector<Point2f>& imgpts1,
                        const vector<Point2f>& imgpts2,
                        Matx34d& P1,
                        Mat_<double>& R,
                        Mat_<double>& t) {
    //Find camera matrices
    //Get Fundamental Matrix
    Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
    //Essential matrix: compute then extract cameras [R|t]
    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
    //decompose E to P' , HZ (9.19)
    SVD svd(E,SVD::MODIFY_A);
    Mat svd_u = svd.u;
    Mat svd_vt = svd.vt;
    Mat svd_w = svd.w;
    Matx33d W(0,-1,0,//HZ 9.13
              1,0,0,
              0,0,1);
    R = svd_u * Mat(W) * svd_vt; //HZ 9.19
    t = svd_u.col(2); //u3
    if (!CheckCoherentRotation(R)) {
        cout<<"resulting rotation is not coherent\n";
        return;
    }
    P1 = Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                 R(1,0),R(1,1),R(1,2),t(1),
                 R(2,0),R(2,1),R(2,2),t(2));
}

bool isRotationMatrix(Mat &R) {
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}

Vec3f rotationMatrixToEulerAngles(Mat &R) {
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6;

    float x, y, z;

    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

int main( int argc, char* argv[] ) {
    // Initialize ROS Node--------------------------------------------------------------------------------------------
    ros::init(argc, argv, "feature_detection");
    ros::NodeHandle n;
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud>("reconstruction_pcl2", 10);
    ros::Publisher cam2_pose_pub = n.advertise<geometry_msgs::PoseStamped>("cam2_pose", 10);
    ros::Rate loop_rate(10);

    // Detect Keypoints & create descriptors --------------------------------------------------------------------------
    Mat image1 = imread("/home/amy/robo_ws/src/computer_vision/img/fountain0.jpg", IMREAD_GRAYSCALE);
    Mat image2 = imread("/home/amy/robo_ws/src/computer_vision/img/fountain1.jpg", IMREAD_GRAYSCALE);





//    float k_arr[9] = {2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1};


    // Camera intristic parameter matrix
    // I did not calibration
    cv::Mat K = (cv::Mat_<float>(3,3) <<  2759.48f, 0.f, 1520.69f,
            0.f, 2764.16f, 1006.81f,
            0.f, 0.f, 1.f);

    vector<cv::KeyPoint> kpts_vec1, kpts_vec2;
    cv::Mat desc1, desc2;
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    // extract feature points and calculate descriptors
    akaze -> detectAndCompute(image1, cv::noArray(), kpts_vec1, desc1);
    akaze -> detectAndCompute(image2, cv::noArray(), kpts_vec2, desc2);


    cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_L2, false);
    // cross check flag set to false
    // because i do cross-ratio-test match
    vector< vector<cv::DMatch> > matches_2nn_12, matches_2nn_21;
    matcher->knnMatch( desc1, desc2, matches_2nn_12, 2 );
    matcher->knnMatch( desc2, desc1, matches_2nn_21, 2 );
    const double ratio = 0.8;

    vector<cv::Point2f> selected_points1, selected_points2;

    for(int i = 0; i < matches_2nn_12.size(); i++) { // i is queryIdx
        if( matches_2nn_12[i][0].distance/matches_2nn_12[i][1].distance < ratio
            and
            matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].distance
            / matches_2nn_21[matches_2nn_12[i][0].trainIdx][1].distance < ratio )
        {
            if(matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].trainIdx
               == matches_2nn_12[i][0].queryIdx)
            {
                selected_points1.push_back(kpts_vec1[matches_2nn_12[i][0].queryIdx].pt);
                selected_points2.push_back(
                        kpts_vec2[matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].queryIdx].pt
                );
            }
        }
    }

    if(true) {
        cv::Mat src;
        cv::hconcat(image1, image2, src);
        for(int i = 0; i < selected_points1.size(); i++) {
            cv::line( src, selected_points1[i],
                      cv::Point2f(selected_points2[i].x + image1.cols, selected_points2[i].y),
                      1, 1, 0 );
        }
//        cv::imwrite("match-result.png", src);
    }

    cv::Mat Kd;
    K.convertTo(Kd, CV_64F);

    cv::Mat mask; // unsigned char array
    cv::Mat E = cv::findEssentialMat(selected_points1, selected_points2, Kd.at<double>(0,0),
            // cv::Point2f(0.f, 0.f),
                                     cv::Point2d(image1.cols/2., image1.rows/2.),
                                     cv::RANSAC, 0.999, 1.0, mask);
    // E is CV_64F not 32F

    vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<unsigned char>(i)){
            inlier_match_points1.push_back(selected_points1[i]);
            inlier_match_points2.push_back(selected_points2[i]);
        }
    }

    if(true) {
        cv::Mat src;
        cv::hconcat(image1, image2, src);
        for(int i = 0; i < inlier_match_points1.size(); i++) {
            cv::line( src, inlier_match_points1[i],
                      cv::Point2f(inlier_match_points2[i].x + image1.cols, inlier_match_points2[i].y),
                      1, 1, 0 );
        }
//        cv::imwrite("inlier_match_points.png", src);
    }

    mask.release();
    cv::Mat R, t;
    cv::recoverPose(E,
                    inlier_match_points1,
                    inlier_match_points2,
                    R, t, Kd.at<double>(0,0),
            // cv::Point2f(0, 0),
                    cv::Point2d(image1.cols/2., image1.rows/2.),
                    mask);
    // R,t is CV_64F not 32F

    vector<cv::Point2d> triangulation_points1, triangulation_points2;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<unsigned char>(i)){
            triangulation_points1.push_back
                    (cv::Point2d((double)inlier_match_points1[i].x,(double)inlier_match_points1[i].y));
            triangulation_points2.push_back
                    (cv::Point2d((double)inlier_match_points2[i].x,(double)inlier_match_points2[i].y));
        }
    }

    if(true) {
        cv::Mat src;
        cv::hconcat(image1, image2, src);
        for(int i = 0; i < triangulation_points1.size(); i++) {
            cv::line( src, triangulation_points1[i],
                      cv::Point2f((float)triangulation_points2[i].x + (float)image1.cols,
                                  (float)triangulation_points2[i].y),
                      1, 1, 0 );
        }
//        cv::imwrite("triangulatedPoints.png", src);
    }

    cv::Mat Rt0 = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat Rt1 = cv::Mat::eye(3, 4, CV_64FC1);
    R.copyTo(Rt1.rowRange(0,3).colRange(0,3));
    t.copyTo(Rt1.rowRange(0,3).col(3));


    cv::Mat point3d_homo;
//    cv::Mat pnts3D(4,triangulation_points1.size(),CV_64FC4);
    cv::Mat cam0 = Kd * Rt0;
    cv::Mat cam1 = Kd * Rt1;
    cout<<Kd * Rt0<<endl;
    cout<<Kd * Rt1<<endl;
    cout<<triangulation_points1<<endl;
    cout<<triangulation_points2<<endl;

    cv::triangulatePoints(cam0, cam1,
                          triangulation_points1, triangulation_points2, point3d_homo);

    cout<<point3d_homo<<endl;
    //point3d_homo is 64F
    //available input type is here
    //https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
//
//    cout<<"please"<<endl;
//    assert(point3d_homo.cols == triangulation_points1.size());
//
////    // prepare a viewer
////    pcl::visualization::PCLVisualizer viewer("Viewer");
////    viewer.setBackgroundColor (255, 255, 255);
////
////    // create point cloud
////    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
////    cloud->points.resize (point3d_homo.cols);
////
    for(int i = 0; i < point3d_homo.cols; i++) {

//        pcl::PointXYZRGB &point = cloud->points[i];
        cv::Mat p3d;
        cv::Mat _p3h = point3d_homo.col(i);
        cv::Mat _p3h_T = _p3h.t();
        convertPointsFromHomogeneous(_p3h_T, p3d);
        cout<<p3d.at<double>(2)<<endl;
//        point.x = p3d.at<double>(0);
//        point.y = p3d.at<double>(1);
//        point.z = p3d.at<double>(2);
//        point.r = 0;
//        point.g = 0;
//        point.b = 255;
    }
//
//    viewer.addPointCloud(cloud, "Triangulated Point Cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                             3,
//                                             "Triangulated Point Cloud");
//    viewer.addCoordinateSystem (1.0);
//
//    // add the second camera pose
//    Eigen::Matrix4f eig_mat;
//    Eigen::Affine3f cam_pose;
//
//    R.convertTo(R, CV_32F);
//    t.convertTo(t, CV_32F);
//
//    //this shows how a camera moves
//    cv::Mat Rinv = R.t();
//    cv::Mat T = -Rinv * t;
//
//    eig_mat(0,0) = Rinv.at<float>(0,0);eig_mat(0,1) = Rinv.at<float>(0,1);eig_mat(0,2) = Rinv.at<float>(0,2);
//    eig_mat(1,0) = Rinv.at<float>(1,0);eig_mat(1,1) = Rinv.at<float>(1,1);eig_mat(1,2) = Rinv.at<float>(1,2);
//    eig_mat(2,0) = Rinv.at<float>(2,0);eig_mat(2,1) = Rinv.at<float>(2,1);eig_mat(2,2) = Rinv.at<float>(2,2);
//    eig_mat(3,0) = 0.f; eig_mat(3,1) = 0.f; eig_mat(3,2) = 0.f;
//    eig_mat(0, 3) = T.at<float>(0);
//    eig_mat(1, 3) = T.at<float>(1);
//    eig_mat(2, 3) = T.at<float>(2);
//    eig_mat(3, 3) = 1.f;
//
//    cam_pose = eig_mat;
//
//    //cam_pose should be Affine3f, Affine3d cannot be used
//    viewer.addCoordinateSystem(1.0, cam_pose, "2nd cam");
//
//    viewer.initCameraParameters ();
//    while (!viewer.wasStopped ()) {
//        viewer.spin();
//    }

    return 0;


//    // For pointcloud coloring
//    Mat color_img = imread("/home/amy/robo_ws/src/computer_vision/img/fountain0.jpg");
//
//    if (img1.empty() || img2.empty()) {
//        cout << "Could not open or find the image!\n" << endl;
//        cout << "Usage: " << argv[0] << " <Input image>" << endl;
//        return -1;
//    }
//
//    //-- Step 1: Detect the keypoints using SURF Detector
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create(minHessian);
//
//    // Initialize keypoints and descriptors
//    vector<KeyPoint> keypoints1, keypoints2;
//    Mat descriptors1, descriptors2;
//
//    // Detect keypoints and create descriptors
//    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
//    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
//
//    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
//    // Since SURF is a floating-point descriptor NORM_L2 is used
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
//    vector<vector<DMatch> > knn_matches;
//    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
//    //-- Filter matches using the Lowe's ratio test
//    const float ratio_thresh = 0.3f;
//    vector<DMatch> good_matches;
//    for (size_t i = 0; i < knn_matches.size(); i++) {
//        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
//            good_matches.push_back(knn_matches[i][0]);
//        }
//    }
//
//    // Load camera calibration matrix
//    // TODO: Replace K with camera info topic
////    // Kinect Calibration
////    float k_arr[9] = {540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0};
//
//    // Benchmark Photo Calibration
//    float k_arr[9] = {2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1};
//
//    Mat_<double> K = cv::Mat(3, 3, CV_32F, k_arr);
//    cv::Mat Kinv;
//    invert(K, Kinv);
//    cout << "Camera Calibration K:" << endl;
//    cout << K << endl;
//    cout << "Inverse of K:" << endl;
//    cout << Kinv << endl;
//
//    vector<Point2f> imgpts1, imgpts2;
//    for (unsigned int i = 0; i < good_matches.size(); i++) {
//        // queryIdx is the "left" image
//        imgpts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
//        // trainIdx is the "right" image
//        imgpts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
//    }
//    Matx34d P1;
//    Mat_<double> R;
//    Mat_<double> t;
//    // Finds transform between camera to map
//    FindCameraMatrices(K, imgpts1, imgpts2, P1, R, t);
//
//    // Invert matrix to get tf between map and camera instead of camera to map
//    Mat_<double> Rinv = R.inv();
//    // Convert from rotation matrix to euler angles
//    Vec3f cam_euler = rotationMatrixToEulerAngles(Rinv);
//    // Convert from euler angles to quaternion
//    tf2::Quaternion cam_quat;
//    cam_quat.setRPY( cam_euler[0], cam_euler[1],cam_euler[2] );
//
//    // TODO: Display cameras in RVIZ
//    cout << "Camera 2 Rotation & Translation" << endl;
//    cout << cam_euler << endl;
//    cout << t << endl;
//
//    geometry_msgs::PoseStamped cam2_pose_msg;
//    // TODO: Remove hardcode
//    cam2_pose_msg.header.frame_id = "map";
//    cam2_pose_msg.header.stamp = ros::Time().now();
//    cam2_pose_msg.pose.orientation.x = cam_quat.x();
//    cam2_pose_msg.pose.orientation.y = cam_quat.y();
//    cam2_pose_msg.pose.orientation.z = cam_quat.z();
//    cam2_pose_msg.pose.orientation.w = cam_quat.w();
//    // Make translations negative (to go from map to camera instaed of camera to map)
//    cam2_pose_msg.pose.position.x = -*t[0];
//    cam2_pose_msg.pose.position.y = -*t[1];
//    cam2_pose_msg.pose.position.z = -*t[2];
//
////    // Find camera matrices -------------------------------------------------------------------------------------
////    vector<Point2f> imgpts1, imgpts2;
////    for (unsigned int i = 0; i < good_matches.size(); i++) {
////        // queryIdx is the "left" image
////        imgpts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
////        // trainIdx is the "right" image
////        imgpts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
////    }
////    // TODO: Replace K with camera info topic
////    // Kinect Calibration
//////    float k_arr[9] = {540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0};
////    // Benchmark Photo Calibration
////    float k_arr[9] = {2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1};
////
////    Mat_<double> K = cv::Mat(3, 3, CV_32F, k_arr);
////
////    Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
////    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
////
////    // Decompose essential matrix into rotation and translation elements --------------------------------------------
////    SVD svd(E);
////    Matx33d W(0, -1, 0,//HZ 9.13
////              1, 0, 0,
////              0, 0, 1);
////    Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19
////    Mat_<double> t = svd.u.col(2); //u3
////    Matx34d P1(R(0, 0), R(0, 1), R(0, 2), t(0),
////               R(1, 0), R(1, 1), R(1, 2), t(1),
////               R(2, 0), R(2, 1), R(2, 2), t(2));
////
////    // Rotation & translation matrices!!!
////    cout << t << endl;
////    cout << R << endl;
////
//    // Triangulation ---------------------------------------------------------------------------------------------------
//    // Initial perspective is fixed with no rotation and no translation
//    Matx34d P(1, 0, 0, 0,
//              0, 1, 0, 0,
//              0, 0, 1, 0);
//
////    // Custom Triangulation
////    vector<Point3d> pointcloud;
////    vector<Vec3b> colors;
////    TriangulatePointsWithColor(keypoints1, keypoints2, K, Kinv, P, P1, pointcloud, colors, color_img);
//
//    // OpenCV Triangulation
//    Matx34d cam0 = K.dot(P1);
//    triangulatePoints();
//
//    // Convert from pointcloud to ROS message ------------------------------------------------------------------
//    // TODO: Move this somewhere else
//    sensor_msgs::PointCloud ros_pcl_msg;
//    ros_pcl_msg.header.frame_id = "map";
//    ros_pcl_msg.header.stamp = ros::Time::now();
//
//    sensor_msgs::ChannelFloat32 new_pt_b;
//    sensor_msgs::ChannelFloat32 new_pt_g;
//    sensor_msgs::ChannelFloat32 new_pt_r;
//    new_pt_b.name = "b";
//    new_pt_g.name = "g";
//    new_pt_r.name = "r";
//
//    for (int i=0; i<pointcloud.size(); i++) {
//        // Add point to ROS message
//        Point3d pt = pointcloud[i];
//        geometry_msgs::Point32 new_pt;
//
//        new_pt.x = pt.x;
//        new_pt.y = pt.y;
//        new_pt.z = pt.z;
//
//        ros_pcl_msg.points.push_back(new_pt);
//
//        // Add color info to ROS message
//        Vec3b pt_color = colors[i];
//        // Convert to floating point values between 0 and 1
//        new_pt_b.values.push_back(pt_color[0]/255.0f);
//        new_pt_g.values.push_back(pt_color[1]/255.0f);
//        new_pt_r.values.push_back(pt_color[2]/255.0f);
//    }
//    ros_pcl_msg.channels.push_back(new_pt_b);
//    ros_pcl_msg.channels.push_back(new_pt_g);
//    ros_pcl_msg.channels.push_back(new_pt_r);
//
//    // Draw matches -------------------------------------------------------------------------------------------------
//    Mat img_matches;
//    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
//                Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//    // Show detected matches
//    namedWindow("Good Matches",WINDOW_NORMAL);
//    imshow("Good Matches", img_matches);
//    resizeWindow("Good Matches", 500,500);
//    waitKey();
//
//    // Publish results to ROS ------------------------------------------------------------------------------------------
//    while (ros::ok()) {
//        cout << ros_pcl_msg.points.size() << endl;
//        pcl_pub.publish(ros_pcl_msg);
//
//        cam2_pose_pub.publish(cam2_pose_msg);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//    cout << "Reached End" << endl;
//    return 0;
}


//    //-- Draw matches-------------------------------------------------------------
//
//    Mat img_matches;
//    drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
//                 Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    //-- Show detected matches
//    imshow("Good Matches", img_matches );
//    waitKey();
//    return 0;





//    //-- Draw keypoints
//    Mat img_keypoints;
//    drawKeypoints( img1, keypoints, img_keypoints );
//    //-- Show detected (drawn) keypoints
//    imshow("SURF Keypoints", img_keypoints );
//    waitKey();
//    return 0;


//
//
//
//
//
//
//int main( int argc, char* argv[] )
//{
//    // Initialize ROS Node-----------------------------------------------------
//    ros::init(argc, argv, "feature_detection");
//    ros::NodeHandle n;
//    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud>("reconstruction_pcl2", 10);
//    ros::Rate loop_rate(10);
//
//    // -----------------------------------------------------------------------------
//    Mat img1 = imread("/home/amy/robo_ws/src/computer_vision/img/fountain0.jpg", IMREAD_GRAYSCALE );
//    Mat img2 = imread("/home/amy/robo_ws/src/computer_vision/img/fountain1.jpg", IMREAD_GRAYSCALE );
//
//
//    if ( img1.empty() || img2.empty())
//    {
//        cout << "Could not open or find the image!\n" << endl;
//        cout << "Usage: " << argv[0] << " <Input image>" << endl;
//        return -1;
//    }
//
//    //-- Step 1: Detect the keypoints using SURF Detector
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create( minHessian );
//
//    // Initialize keypoints and descriptors
//    vector<KeyPoint> keypoints1, keypoints2;
//    Mat descriptors1, descriptors2;
//
//    // Detect keypoints and create descriptors
//    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
//    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );
//
//    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
//    // Since SURF is a floating-point descriptor NORM_L2 is used
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
//    vector< vector<DMatch> > knn_matches;
//    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
//    //-- Filter matches using the Lowe's ratio test
//    const float ratio_thresh = 0.7f;
//    vector<DMatch> good_matches;
//    for (size_t i = 0; i < knn_matches.size(); i++)
//    {
//        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
//        {
//            good_matches.push_back(knn_matches[i][0]);
//        }
//    }
//
//    //    vector<DMatch> n_first_matches(good_matches.begin(), good_matches.begin() + 10);
//
//    // Find camera matrices -------------------------------------------------------------------------------------
//    vector<Point2f>imgpts1,imgpts2;
//    for( unsigned int i = 0; i<good_matches.size(); i++ ){
//        // queryIdx is the "left" image
//        imgpts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
//        // trainIdx is the "right" image
//        imgpts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
//    }
//    // TODO: Replace K with camera info topic
//    // Kinect Calibration
////    float k_arr[9] = {540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0};
//    // Benchmark Photo Calibration
//    float k_arr[9] = {2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1};
//
//    Mat_<double> K = cv::Mat(3, 3, CV_32F, k_arr);
//
//    cv::Mat Kinv;
//    invert(K, Kinv);
////    cout << "Camera Calibration K:" << endl;
////    cout << K << endl;
////    cout << "Inverse of K:" << endl;
////    cout << Kinv << endl;
//////    FindCameraMatrices(K, )
//
//
//
//
//
//
//
//
////    cout << K << endl;
////
////    Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
////    cout << F << endl;
////    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
////
////    // Decompose essential matrix into rotation and translation elements --------------------------------------------
////    SVD svd(E);
////    Matx33d W(0,-1,0,//HZ 9.13
////              1,0,0,
////              0,0,1);
////    Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19
////    Mat_<double> t = svd.u.col(2); //u3
////    Matx34d P1( R(0,0),R(0,1), R(0,2), t(0),
////                R(1,0),R(1,1), R(1,2), t(1),
////                R(2,0),R(2,1), R(2,2), t(2));
////
////
////    // Rotation & translation matrices!!!
////    cout << t << endl;
////    cout << R << endl;
////
////    cout << CheckCoherentRotation(R) << endl;
////
////    // Triangulation ---------------------------------------------------------------------------------------------------
////    cv::Mat Kinv;
////    invert(K, Kinv);
////
////    // Initial perspective is fixed with no rotation and no translation
////    Matx34d P( 1, 0, 0, 0,
////               0, 1, 0, 0,
////               0, 0, 1, 0);
////
////    vector<Point3d> pointcloud;
////
////    TriangulatePoints(keypoints1, keypoints2, K, Kinv, P, P1, pointcloud);
////////    const vector<KeyPoint>& pt_set1,
////////    const vector<KeyPoint>& pt_set2,
////////    const Mat&Kinv,
////////    const Matx34d& P,
////////    const Matx34d& P1,
////////    vector<Point3d>& pointcloud)
////
////
////
////    // Convert from pointcloud to ROS message
////    // TODO: Move this somewhere else
////    sensor_msgs::PointCloud ros_pcl_msg;
////    ros_pcl_msg.header.frame_id = "map";
////    ros_pcl_msg.header.stamp = ros::Time::now();
////
////    for(Point3d pt : pointcloud) {
////        geometry_msgs::Point32 new_pt;
////        new_pt.x = pt.x;
////        new_pt.y = pt.y;
////        new_pt.z = pt.z;
////
////        ros_pcl_msg.points.push_back(new_pt);
////    }
////
////    //-- Draw matches-------------------------------------------------------------
//////    Mat img_matches;
//////    drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
//////                 Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//////    //-- Show detected matches
//////    imshow("Good Matches", img_matches );
//////    waitKey();
////
////    while (ros::ok()) {
////        pcl_pub.publish(ros_pcl_msg);
////        ros::spinOnce();
////        loop_rate.sleep();
////    }