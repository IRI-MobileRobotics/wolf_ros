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
#include "glog/logging.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

class WolfNode
{
    public:
        WolfNode(char *argv0);
        virtual ~WolfNode();

        void solve();
        const wolf::Problem& getProblem();
        void broadcastTf();
        void publishMarkers();

    protected:
        //wolf problem
        wolf::Problem problem_;
        bool origin_set_;
        Eigen::Matrix3s origin_cov_;
        ros::Time origin_stamp_;

        //Wolf: laser sensors
        std::vector<wolf::SensorLaser2D*> laser_sensor_ptr_;
        std::vector<bool> laser_intrinsics_set_;
        std::vector<bool> laser_extrinsics_set_;
        std::vector<std::string> laser_frame_name_;
        std::map<std::string,unsigned int> laser_frame_2_idx_;

        //ceres
        wolf::CeresManager ceres_manager_;

        //transforms
        tf::TransformBroadcaster tfb_;
        tf::TransformListener    tfl_;
        tf::Transform T_map2base_; //wolf output
        tf::StampedTransform T_base2odom_; //published by odom source
        tf::StampedTransform T_map2odom_; //to be broadcasted by this node
        std::string base_frame_name_;
        std::string map_frame_name_;
        std::string odom_frame_name_;
        Eigen::VectorXs vehicle_pose_;

        //Odometry
        ros::Time last_odom_stamp_;
        ros::Subscriber odom_sub_; // odometry subscriber
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        wolf::CaptureMotion2* odom_capture_ptr_;

        // Lasers
        std::vector<ros::Subscriber> laser_subscribers_;
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void loadLaserIntrinsics(const unsigned int laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg);


        bool loadSensorExtrinsics(const std::string _sensor_frame, Eigen::VectorXs& _extrinsics);
        bool setOrigin();

        // [publisher attributes]
        ros::Publisher constraints_publisher_;
        visualization_msgs::Marker constraints_marker_msg_;

        ros::Publisher landmarks_publisher_;
        visualization_msgs::MarkerArray landmark_marker_array_msg_;
        visualization_msgs::Marker landmark_marker_, landmark_text_marker_;

        ros::Publisher trajectory_publisher_;
        visualization_msgs::MarkerArray trajectory_marker_array_msg_;
        visualization_msgs::Marker vehicle_marker_;

        // ROS node handle
        ros::NodeHandle nh_;
};

#endif //WOLF_ROS_WOLF_GPS_NODE_H
