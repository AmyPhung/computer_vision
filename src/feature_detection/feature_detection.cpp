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

// Publishing images to ROS
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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

int displayImg(Mat img)
{
    imshow("Display Window", img);
    int k = waitKey(0);
    return k;
}

// height_or_width - true: resize around height
//                   false: resize around width
// side_lenght - the size of the new height or width
// img - input image to be resized
// out_img - the resized image
void resizeImg(Mat img, Mat& out_img, bool height_or_width, int side_length)
{
    // TODO: Generalize the logic in this function

    // std::cout << "Source image rows: " << img.rows << " | Source image cols: " << img.cols << std::endl;

    // Stop if the image is already the correct size
    if ( (height_or_width && img.rows == side_length) || !height_or_width && img.cols == side_length )
    {
        // std::cout << "Image is already the correct size" << std::endl;
        out_img = img;
        return;
    }

    // Calculate a rescaling factor according to height or width
    double rescale_factor;
    if (height_or_width) { rescale_factor = ( (double) side_length) / ( (double) img.rows); } // height
    else { rescale_factor = ( (double) side_length) / ( (double) img.cols); } // width

    // std::cout << "Rescaling factor: " << rescale_factor << std::endl;

    // Set the interpolation method
    int interpolation_method;
    if (rescale_factor > 1) { interpolation_method = INTER_CUBIC; }
    else { interpolation_method = INTER_LINEAR; }

    // Resize the image accordingly
    resize(img, out_img, Size(), rescale_factor, rescale_factor, interpolation_method);
}

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

// Gets colors of keypoints from image
void getKeypointColors(const vector<Point2f>& imgpts, vector<Vec3b>& colors, const Mat& color_img) {
    for (unsigned int i=0; i<imgpts.size(); i++) {
        Point2f kp = imgpts[i];

        // Extract color info from image 1
        Vec3b bgrPixel = color_img.at<Vec3b>(kp.y, kp.x); // Extracts in row, column (y, x) order
        colors.push_back(bgrPixel);
    }
}

// Returns fundamental matrix
void FindCameraMatrices(const Mat& K,
                        const vector<Point2f>& imgpts1,
                        const vector<Point2f>& imgpts2,
                        Matx34d& P1,
                        Mat_<double>& R,
                        Mat_<double>& t,
                        Mat& F) {
    //Find camera matrices
    //Get Fundamental Matrix
    F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
    std::cout << "Fundamental Matrix\n" << F << std::endl;
    //Essential matrix: compute then extract cameras [R|t]
    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
    std::cout << "Essential Matrix\n" << E << std::endl;
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


    // Mat epilines1, epilines2, epilines_quad_img_display, epilines_dual_img_display;
    // calculateNumEpilines(imgpts1, imgpts2, F, epilines1, epilines2, 70);

void calculateNumEpilines(vector<Point2f> imgpts1, vector<Point2f> imgpts2, Mat F,
                          std::vector<Vec3f>& epilines1, std::vector<Vec3f>& epilines2, int numpts = 30)
{
    // Grab a subsample of matched points
    std::vector<Point2f> imgpts1_subsample, imgpts2_subsample;
    for ( int i = 0; i < numpts; i++ )
    {
        imgpts1_subsample.emplace_back(imgpts1[i]);
        imgpts2_subsample.emplace_back(imgpts2[i]);
    }

    // Compute the epilines
    // epilines number corresponds to image those epilines are drawn on
    computeCorrespondEpilines(imgpts1_subsample, 1, F, epilines2);
    computeCorrespondEpilines(imgpts2_subsample, 2, F, epilines1);
}

std::vector<Scalar> getColorPalette()
{
    // Set up colors - BGR
    std::vector<Scalar> color_palette;
    color_palette.emplace_back(Scalar(217, 198, 255)); // pink
    color_palette.emplace_back(Scalar(60, 51, 248));   // red
    color_palette.emplace_back(Scalar(16, 171, 252));  // yellow-orange
    color_palette.emplace_back(Scalar(179, 158, 43));  // cyan
    color_palette.emplace_back(Scalar(105, 175, 68));  // green
    color_palette.emplace_back(Scalar(128, 90, 61));   // dark blue
    color_palette.emplace_back(Scalar(38, 40, 28));    // grey
    color_palette.emplace_back(Scalar(20, 8, 61));     // brown
    return color_palette;
}

void createEpilinesQuadImgDisplay(Mat img1_color, Mat img2_color, std::vector<Point2f> imgpts1, std::vector<Point2f> imgpts2,
                           Mat F, std::vector<Vec3f> epilines1, std::vector<Vec3f> epilines2, Mat& quad_img_display, std::vector<Scalar> color_palette)
{
    // Make copies of each image for drawing
    Mat img2_keypoints = img2_color.clone();
    Mat img2_epilines = img2_color.clone();
    Mat img1_keypoints = img1_color.clone();
    Mat img1_epilines = img1_color.clone();

    // Draw keypoints and corresponding epilines
    for ( int i = 0; i < epilines2.size(); i++ )
    {
        // Get the color
        Scalar color = color_palette[i % color_palette.size()];

        // Calculate endpoints for epilines on image 2
        Point endpoint1 = cv::Point(0, -epilines2[i][2] / epilines2[i][1]);
        Point endpoint2 = cv::Point(img2_color.cols, -( epilines2[i][2] + epilines2[i][0] * img2_color.cols ) / epilines2[i][1] );

        // Draw the epiline on image 2
        line(img2_epilines, endpoint1, endpoint2, color, 3);

        // Draw the corresponding keypoint on image 1
        circle(img1_keypoints, imgpts1[i], 20, color, 5);

        // Calculate endpoints for epilines on image 1
        Point endpoint1a = cv::Point(0, -epilines1[i][2] / epilines1[i][1]);
        Point endpoint2a = cv::Point(img1_color.cols, -( epilines1[i][2] + epilines1[i][0] * img1_color.cols ) / epilines1[i][1] );

        // Draw the epiline on image 2
        line(img1_epilines, endpoint1a, endpoint2a, color, 3);

        // Draw the corresponding keypoint on image 1
        circle(img2_keypoints, imgpts2[i], 20, color, 5);
    }

    // Display epipolar lines and corresponding keypoints
    Mat img1_keypoints_display, img2_keypoints_display;
    Mat img2_epilines_display, img1_epilines_display;
    resizeImg(img2_epilines, img2_epilines_display, true, 500);
    resizeImg(img1_keypoints, img1_keypoints_display, true, 500);
    resizeImg(img1_epilines, img1_epilines_display, true, 500);
    resizeImg(img2_keypoints, img2_keypoints_display, true, 500);

    Mat dual_img_display12, dual_img_display21;
    hconcat(img1_keypoints_display, img2_epilines_display, dual_img_display12);
    hconcat(img1_epilines_display, img2_keypoints_display, dual_img_display21);

    vconcat(dual_img_display12, dual_img_display21, quad_img_display);
}

void createEpilinesDualImgDisplay(Mat img1_color, Mat img2_color, std::vector<Point2f> imgpts1, std::vector<Point2f> imgpts2,
                           Mat F, std::vector<Vec3f> epilines1, std::vector<Vec3f> epilines2, Mat& dual_img_display, std::vector<Scalar> color_palette)
{
    // Make copies of each image for drawing
    Mat img2_copy = img2_color.clone();
    Mat img1_copy = img1_color.clone();

    // Draw keypoints and corresponding epilines
    for ( int i = 0; i < epilines2.size(); i++ )
    {
        // Get the color
        Scalar color = color_palette[i % color_palette.size()];

        // Calculate endpoints for epilines on image 2
        Point endpoint1 = cv::Point(0, -epilines2[i][2] / epilines2[i][1]);
        Point endpoint2 = cv::Point(img2_color.cols, -( epilines2[i][2] + epilines2[i][0] * img2_color.cols ) / epilines2[i][1] );

        // Draw the epiline on image 2
        line(img2_copy, endpoint1, endpoint2, color, 3);

        // Draw the corresponding keypoint on image 1
        circle(img1_copy, imgpts1[i], 20, color, 5);

        // Calculate endpoints for epilines on image 1
        Point endpoint1a = cv::Point(0, -epilines1[i][2] / epilines1[i][1]);
        Point endpoint2a = cv::Point(img1_color.cols, -( epilines1[i][2] + epilines1[i][0] * img1_color.cols ) / epilines1[i][1] );

        // Draw the epiline on image 2
        line(img1_copy, endpoint1a, endpoint2a, color, 3);

        // Draw the corresponding keypoint on image 1
        circle(img2_copy, imgpts2[i], 20, color, 5);
    }

    // Display epipolar lines and corresponding keypoints
    Mat img1_display, img2_display;
    resizeImg(img2_copy, img2_display, true, 500);
    resizeImg(img1_copy, img1_display, true, 500);

    hconcat(img1_display, img2_display, dual_img_display);
    // hconcat(img1_epilines_display, img2_keypoints_display, dual_img_display21);

    // vconcat(dual_img_display12, dual_img_display21, quad_img_display);
}


int main( int argc, char* argv[] ) {
    // Initialize ROS Node--------------------------------------------------------------------------------------------
    ros::init(argc, argv, "feature_detection");
    ros::NodeHandle n;
    ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud>("reconstruction_pcl2", 10);
    ros::Publisher cam2_pose_pub = n.advertise<geometry_msgs::PoseStamped>("cam2_pose", 10);

    // Set up the image publishers for epiline visualizations
    std::string quad_img_topic = "/epilines_quad_img";
    std::string dual_img_topic = "/epilines_dual_img";
    image_transport::ImageTransport it_quad(n);
    image_transport::Publisher epilines_quad_image_pub = it_quad.advertise(quad_img_topic, 1);
    image_transport::ImageTransport it_dual(n);
    image_transport::Publisher epilines_dual_image_pub = it_quad.advertise(dual_img_topic, 1);

    cv_bridge::CvImagePtr cv_ptr_epi_quad(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_epi_dual(new cv_bridge::CvImage);

    std::string img_encoding = "bgr8";
    cv_ptr_epi_quad->encoding = img_encoding;
    cv_ptr_epi_quad->header.frame_id = quad_img_topic;
    cv_ptr_epi_dual->encoding = img_encoding;
    cv_ptr_epi_dual->header.frame_id = dual_img_topic;

    ros::Rate loop_rate(10);

    // Detect Keypoints & create descriptors --------------------------------------------------------------------------
    // std::string ws_path = "/home/amy/robo_ws";
    std::string ws_path = "/home/egonzalez/catkin_ws";
    Mat img1_color = imread( ws_path + "/src/computer_vision/img/fountain0.jpg", IMREAD_COLOR );
    Mat img2_color = imread( ws_path + "/src/computer_vision/img/fountain1.jpg", IMREAD_COLOR );

    if (img1_color.empty() || img2_color.empty()) {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    Mat img1;
    Mat img2;

    cvtColor(img1_color, img1, COLOR_BGR2GRAY);
    cvtColor(img2_color, img2, COLOR_BGR2GRAY);

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);

    // Initialize keypoints and descriptors
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    // Detect keypoints and create descriptors
    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
    vector<vector<DMatch> > knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    // TODO: Make this a ros parameter
    const float ratio_thresh = 0.675f;
    vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Load camera calibration matrix
    // TODO: Replace K with camera info topic
//    // Kinect Calibration
//    float k_arr[9] = {540.707803103922, 0.0, 322.35464797552, 0.0, 538.1654785489391, 242.2561047778664, 0.0, 0.0, 1.0};
    // Benchmark Photo Calibration
    float k_arr[9] = {2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1};

    Mat_<double> K = cv::Mat(3, 3, CV_32F, k_arr);
    cv::Mat Kinv;
    invert(K, Kinv);
    cout << "Camera Calibration K:" << endl;
    cout << K << endl;
    cout << "Inverse of K:" << endl;
    cout << Kinv << endl;

    // Extract 2d camera points from keypoints
    vector<Point2f> imgpts1, imgpts2;
    for (unsigned int i = 0; i < good_matches.size(); i++) {
        // queryIdx is the "left" image
        imgpts1.push_back(keypoints1[good_matches[i].queryIdx].pt);
        // trainIdx is the "right" image
        imgpts2.push_back(keypoints2[good_matches[i].trainIdx].pt);
    }

    // Find transform between camera1 to camera0
    Matx34d P1;
    Mat_<double> R;
    Mat_<double> t;
    Mat F;
    FindCameraMatrices(K, imgpts1, imgpts2, P1, R, t, F);

    // std::cout << imgpts2 << std::endl;

    // Get Visualizations for Epi Polar Lines -------------------------------------------------------------------------------------------------
    std::vector<Vec3f> epilines1, epilines2;
    Mat epilines_quad_img_display, epilines_dual_img_display;

    calculateNumEpilines(imgpts1, imgpts2, F, epilines1, epilines2, 30);
    createEpilinesQuadImgDisplay(img1_color, img2_color, imgpts1, imgpts2, F, epilines1, epilines2, epilines_quad_img_display, getColorPalette());
    createEpilinesDualImgDisplay(img1_color, img2_color, imgpts1, imgpts2, F, epilines1, epilines2, epilines_dual_img_display, getColorPalette());
    cv_ptr_epi_quad->image = epilines_quad_img_display;
    cv_ptr_epi_dual->image = epilines_dual_img_display;

    cv::Mat Rt0 = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat Rt1 = cv::Mat::eye(3, 4, CV_64FC1);
    R.copyTo(Rt1.rowRange(0, 3).colRange(0, 3));
    t.copyTo(Rt1.rowRange(0, 3).col(3));

    // Triangulate Points
    cv::Mat point3d_homo;
    cv::Mat proj_0 = K * Rt0;
    cv::Mat proj_1 = K * Rt1;
    cv::triangulatePoints(proj_0, proj_1,
                          imgpts1, imgpts2,
                          point3d_homo);

    // Extract keypoint colors
    vector<Vec3b> keypoint_colors;
    getKeypointColors(imgpts1, keypoint_colors, img1_color);

    // Visualize camera positions in ROS ------------------------------------------------------------------------------
    // Invert matrix to get tf between map and camera instead of camera to map
    Mat_<double> Rinv = R.inv();
    // Convert from rotation matrix to euler angles
    Vec3f cam_euler = rotationMatrixToEulerAngles(Rinv);
    // Convert from euler angles to quaternion
    tf2::Quaternion cam_quat;
    cam_quat.setRPY( cam_euler[0], cam_euler[1],cam_euler[2] );

    // TODO: Display cameras in RVIZ
    cout << "Camera 2 Rotation & Translation" << endl;
    cout << cam_euler << endl;
    cout << t << endl;

    geometry_msgs::PoseStamped cam2_pose_msg;
    // TODO: Remove hardcoded frame
    cam2_pose_msg.header.frame_id = "map";
    cam2_pose_msg.header.stamp = ros::Time().now();
    cam2_pose_msg.pose.orientation.x = cam_quat.x();
    cam2_pose_msg.pose.orientation.y = cam_quat.y();
    cam2_pose_msg.pose.orientation.z = cam_quat.z();
    cam2_pose_msg.pose.orientation.w = cam_quat.w();
    // Make translations negative (to go from map to camera instaed of camera to map)
    cam2_pose_msg.pose.position.x = -*t[0];
    cam2_pose_msg.pose.position.y = -*t[1];
    cam2_pose_msg.pose.position.z = -*t[2];

    // Convert from pointcloud to ROS message ------------------------------------------------------------------
    // TODO: Move this somewhere else
    sensor_msgs::PointCloud ros_pcl_msg;
    ros_pcl_msg.header.frame_id = "map";
    ros_pcl_msg.header.stamp = ros::Time::now();

    // Initialize color info
    sensor_msgs::ChannelFloat32 new_pt_b;
    sensor_msgs::ChannelFloat32 new_pt_g;
    sensor_msgs::ChannelFloat32 new_pt_r;
    new_pt_b.name = "b";
    new_pt_g.name = "g";
    new_pt_r.name = "r";

    for (int i = 0; i < point3d_homo.cols; i++) {
        geometry_msgs::Point32 new_pt;
        cv::Mat p3d;
        cv::Mat _p3h = point3d_homo.col(i);
        cv::Mat _p3h_T = _p3h.t();

        convertPointsFromHomogeneous(_p3h_T, p3d);

        // Add color info to ROS message
        Vec3b pt_color = keypoint_colors[i];
        // Convert to floating point values between 0 and 1
        new_pt_b.values.push_back(pt_color[0]/255.0f);
        new_pt_g.values.push_back(pt_color[1]/255.0f);
        new_pt_r.values.push_back(pt_color[2]/255.0f);

        new_pt.x = p3d.at<float>(0);
        new_pt.y = p3d.at<float>(1);
        new_pt.z = p3d.at<float>(2);

        ros_pcl_msg.points.push_back(new_pt);
    }

    ros_pcl_msg.channels.push_back(new_pt_b);
    ros_pcl_msg.channels.push_back(new_pt_g);
    ros_pcl_msg.channels.push_back(new_pt_r);

    // Publish results to ROS ------------------------------------------------------------------------------------------
    while (ros::ok()) {
        ros::Time time = ros::Time::now();
        // cout << ros_pcl_msg.points.size() << endl;
        pcl_pub.publish(ros_pcl_msg);
        cv_ptr_epi_quad->header.stamp = time;
        cv_ptr_epi_dual->header.stamp = time;
        epilines_quad_image_pub.publish(cv_ptr_epi_quad->toImageMsg());
        epilines_dual_image_pub.publish(cv_ptr_epi_dual->toImageMsg());
        cam2_pose_pub.publish(cam2_pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}