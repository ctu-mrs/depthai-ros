#include "depthai_filters/person2d_overlay.hpp"

#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

namespace depthai_filters {
void Person2DOverlay::onInit() {
    auto pNH = getPrivateNodeHandle();
    previewSub.subscribe(pNH, "/rgb/preview/image_raw", 1);
    detSub.subscribe(pNH, "/nn/detections", 1);
    depthSub.subscribe(pNH, "/depthImage", 1);
    droneSub = pNH.subscribe("/odom", 1, &Person2DOverlay::odomCB); // subscribe to odom topic - LB
    pNH.getParam("odom_frame", odom_frame); // make sure to add odom frame to the launch file - LB

    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    pNH.getParam("label_map", labelMap);
    sync->registerCallback(std::bind(&Person2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
    MarkerPub = pNH.advertise<visualization_msgs::MarkerArray>("markers", 10);
}
// callback function for odom topic - LB
void Person2DOverlay::odomCB(const nav_msgs::Odometry::ConstPtr& msg){
    dronePose = msg;
}

void Person2DOverlay::overlayCB(const sensor_msgs::ImageConstPtr& preview, const vision_msgs::Detection2DArrayConstPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(preview, sensor_msgs::image_encodings::BGR8);
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);
    auto blue = cv::Scalar(255, 0, 0);

    bool person_detected = false;

    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.y + detections->detections[0].bbox.size_y / 2.0;
        auto labelStr = labelMap[detection.results[0].id];
        if (labelStr != "person"){
          continue;
        }
        person_detected = true;
        auto confidence = detection.results[0].score;

        // get drone position - LB
        auto drone_x = dronePose->pose.pose.position.x;
        auto drone_y = dronePose->pose.pose.position.y;
        auto drone_z = dronePose->pose.pose.position.z;

        cv::Mat img_new = cv::Mat::zeros(previewMat.rows * 2, previewMat.cols * 2, previewMat.type());;
        cv::putText(img_new, labelStr, cv::Point(x2 - 10, y2 - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(img_new, labelStr, cv::Point(x2 - 10, y2 - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point(x2 - 10, y2 - 20), 180, 1.0); //couter clockwise rotation
        cv::warpAffine(img_new, img_new, rotationMatrix, cv::Size(previewMat.cols, previewMat.rows));
        previewMat = previewMat + img_new;

        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        img_new = cv::Mat::zeros(previewMat.rows * 2, previewMat.cols * 2, previewMat.type());;
        cv::putText(img_new, confStr.str(), cv::Point(x2 - 10, y2 - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(img_new, confStr.str(), cv::Point(x2 - 10, y2 - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        rotationMatrix = cv::getRotationMatrix2D(cv::Point(x2 - 10, y2 - 40), 180, 1.0);//couter clockwise rotation
        cv::warpAffine(img_new, img_new, rotationMatrix, cv::Size(previewMat.cols, previewMat.rows));
        previewMat = previewMat + img_new;

        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
        
        // create marker - LB
        if (drone_x-old_drone_x < 1 && drone_y-old_drone_y < 1 && drone_z-old_drone_z < 1){
            old_drone_x = drone_x;
            old_drone_y = drone_y;
            old_drone_z = drone_z;
            continue
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = odom_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "drone_marker";
        marker.id = id_counter;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = drone_x;
        marker.pose.position.y = drone_y;
        marker.pose.position.z = drone_z;
        marker.pose.orientation.x = dronePose->pose.pose.orientation.x;
        marker.pose.orientation.y = dronePose->pose.pose.orientation.y;
        marker.pose.orientation.z = dronePose->pose.pose.orientation.z;
        marker.pose.orientation.w = dronePose->pose.pose.orientation.w;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;  
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
        id_counter++;

        // Update old drone position - LB
        old_drone_x = drone_x;
        old_drone_y = drone_y;
        old_drone_z = drone_z;
    }
 
    if(person_detected){
      sensor_msgs::Image outMsg;
      cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);
      overlayPub.publish(outMsg);
      MarkerPub.publish(markerArray);
    }
}
}  // namespace depthai_filters

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::Person2DOverlay, nodelet::Nodelet)
