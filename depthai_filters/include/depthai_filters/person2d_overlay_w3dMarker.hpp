#pragma once

#include "image_transport/image_transport.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"

namespace depthai_filters {
class Person2DOverlay : public nodelet::Nodelet {
   public:
    void onInit() override;

    void overlayCB(const sensor_msgs::ImageConstPtr& preview, const vision_msgs::Detection2DArrayConstPtr& detections);

    message_filters::Subscriber<sensor_msgs::Image> previewSub;
    message_filters::Subscriber<vision_msgs::Detection2DArray> detSub;
    message_filters::Subscriber<sensor_msgs::Image> depthSub;
    ros::Subscriber droneSub; // subscribe to odom topic - LB
    nav_msgs::Odometry::ConstPtr dronePose; // drone odom - LB

    // initialize marker array
    visualization_msgs::MarkerArray markerArray;

    //initialize old drone position
    float old_drone_x = -1E10;
    float old_drone_y = -1E10;
    float old_drone_z = -1E10;
    int id_counter = 0;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    ros::Publisher overlayPub;
    ros::Publisher MarkerPub;
    std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                         "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                         "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
};
}  // namespace depthai_filters
