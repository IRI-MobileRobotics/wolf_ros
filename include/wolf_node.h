//
// Created by jvallve on 13/05/16.
//

#ifndef WOLF_ROS_WOLF_GPS_NODE_H
#define WOLF_ROS_WOLF_GPS_NODE_H

/**************************
 *      raw_gps_utils     *
 **************************/
#include "raw_gps_utils/satellites_obs.h"

/**************************
 *    laser_scan_utils    *
 **************************/
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/laser_scan.h"

/**************************
 *      WOLF includes     *
 **************************/
#include "wolf/processor_gps.h"
#include "wolf/sensor_gps.h"
#include "wolf/sensor_laser_2D.h"
#include "wolf/sensor_odom_2D.h"
#include "wolf/wolf.h"
#include "wolf/problem.h"
#include "wolf/capture_motion.h"
#include "wolf/capture_odom_2D.h"
#include "wolf/capture_fix.h"
#include "wolf/processor_tracker_landmark_corner.h"
#include "wolf/processor_odom_2D.h"

/**************************
 *     CERES includes     *
 **************************/
#include "wolf/ceres_wrapper/ceres_manager.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

class WolfNode
{
public:
    WolfNode();
    virtual ~WolfNode();

protected:
    //wolf problem
    wolf::Problem problem_;

    //Wolf: laser sensors
    std::vector<wolf::SensorLaser2D*> laser_sensor_ptr_;
    std::vector<bool> laser_intrinsics_set_;
    std::vector<bool> laser_extrinsics_set_;
    std::vector<std::string> laser_frame_name_;
    std::map<std::string,unsigned int> laser_frame_2_idx_;

    //ceres
    wolf::CeresManager ceres_manager_;

    void solve();
    const wolf::Problem& getProblem();

    //transforms
    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;
    std::string base_frame_name_;
    std::string gps_frame_name_;
    std::string world_frame_name_;
    std::string map_frame_name_;
    std::string odom_frame_name_;

    //only for visualization reasons
    std::string map_initial_frame_name_;

    //Odometry callback
    ros::Time last_odom_stamp_;
    ros::Subscriber odom_sub_; // odometry subscriber
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Lasers callback
    std::vector<ros::Subscriber> laser_subscribers_;
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void loadLaserIntrinsics(const unsigned int laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg);
    bool loadLaserExtrinsics(const unsigned int laser_idx, Eigen::VectorXs& _extrinsics);

    //transforms
    tf::TransformBroadcaster tfb_;
    tf::TransformListener    tfl_;
    tf::Transform T_map2base_; //wolf output
    tf::Transform T_odom2base_; //published by odom source
    tf::Transform T_map2odom_; //to be broadcasted by this node

    // [publisher attributes]
    ros::Publisher lines_publisher_;
    visualization_msgs::MarkerArray lines_MarkerArray_msg_;

    ros::Publisher constraints_publisher_;
    visualization_msgs::Marker constraints_Marker_msg_;

    ros::Publisher corners_publisher_;
    visualization_msgs::MarkerArray corners_MarkerArray_msg_;

    ros::Publisher vehicle_publisher_;
    visualization_msgs::MarkerArray vehicle_MarkerArray_msg_;

    // ROS node handle
    ros::NodeHandle nh_;
};

#endif //WOLF_ROS_WOLF_GPS_NODE_H
