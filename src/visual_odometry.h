//
// Created by amy on 10/29/20.
//

#ifndef COMPUTER_VISION_VISUAL_ODOMETRY_H
#define COMPUTER_VISION_VISUAL_ODOMETRY_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class VisualOdometryNode {
public:
    VisualOdometryNode(ros::NodeHandle* nodehandle);

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber camera_raw_subscriber_;
    ros::Subscriber camera_info_subscriber_;

    image_transport::ImageTransport it_;

    void initializePublishers();

    void cameraRawCallback(const sensor_msgs::ImageConstPtr& msg);
//    void cameraInfoCallback(const sensor_msgs::CameraInfo& msg);


};


#endif //COMPUTER_VISION_VISUAL_ODOMETRY_H
