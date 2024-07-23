#ifndef _ROS_WRAPPER_H_
#define _ROS_WRAPPER_H_

// std headers
#include <iostream>
#include <vector>

// ROS related headers
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// User-defined types & timer headers
#include "defines.h"
#include "timer.h"

// Mapless Dynamic header
#include <visualization_msgs/Marker.h>

#include "mapless_moving.h"

class ROSWrapper {
    /* =========
        MEMBERS
       ========= */
    // Previous variables
   private:
    Mask mask0;
    bool is_initialized_;       // = default : false.
    bool is_initialized_pose_;  // = default : false.
    bool is_pose_received_;
    PointCloudwithTime::Ptr p0_pcl_wtime_;
    PointCloudwithTime::Ptr p1_pcl_wtime_;

    Pose pose_pre_;
    Pose pose_cur_;
    Pose T01_;
    Pose T10_;

    Rot rot_;
    Eigen::Vector3d trans_;

    geometry_msgs::Quaternion geoQuat_;
    double q0_, q1_, q2_, q3_;

    double first_timestamp_pose_msg_ = 0.0;
    double first_timestamp_pcl_msg_ = 0.0;
    double timestamp_pose_msg_ = 0.0;
    double timestamp_pcl_msg_ = 0.0;

    // ROS nodehandle & subscriber for LiDAR data.
   private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_pose_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_lidar_marker_;
    visualization_msgs::Marker marker_;
    visualization_msgs::Marker lidar_marker_;

    std::string topicname_lidar_;
    std::string topicname_pose_;
    bool rosbag_play_;
    bool T01_slam_;
    std::string dataset_name_;
    std::string data_number_;
    std::string dataset_kitti_dir_;
    std::string dataset_carla_dir_;

    std_msgs::Header cloudHeader_;

    // Mapless Dynamic algorithm object.
   private:
    std::unique_ptr<MaplessDynamic> solver_;

    /* =========
        METHODS
       ========= */
   public:
    ROSWrapper(ros::NodeHandle& nh);  // constructor
    ~ROSWrapper();                    // destructor

   private:
    void run();

    void getLaunchParameters();
    void callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg);
    void callbackPose(const nav_msgs::Odometry::ConstPtr& pose);
    void updatePreviousVariables(PointCloudwithTime::Ptr p0_pcl_wtime,
                                 PointCloudwithTime::Ptr p1_pcl_wtime,
                                 const Mask& mask1);
};

#endif