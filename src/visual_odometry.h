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
    image_transport::CameraSubscriber camera_subscriber_;
    image_transport::ImageTransport it_;

    void initializePublishers();

    void cameraRawCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
};


#endif //COMPUTER_VISION_VISUAL_ODOMETRY_H
