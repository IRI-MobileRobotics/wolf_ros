//
// Created by jvallve on 13/05/16.
//

#include "wolf_node.h"

WolfNode::WolfNode() :
    nh_(ros::this_node::getName()),
    problem_(wolf::FRM_PO_2D),
    ceres_manager_(&problem_),
    vehicle_pose_(3)
{
    //std::cout << "WolfNode::WolfNode(...) -- constructor\n";

    // parameters
    bool use_wolf_auto_diff;
    int max_iterations, n_lasers;
    double odom_std[2];
    nh_.param<bool>("use_wolf_auto_diff", use_wolf_auto_diff, true);
    nh_.param<int>("max_iterations", max_iterations, 1);
    nh_.param<int>("n_lasers", n_lasers, 6);
    nh_.param<double>("odometry_translational_std", odom_std[0], 0.2);
    nh_.param<double>("odometry_rotational_std", odom_std[1], 0.2);
    nh_.param<std::string>("base_frame_name", base_frame_name_, "agv_base_link");
    nh_.param<std::string>("map_frame_name_", map_frame_name_, "map");
    nh_.param<std::string>("odom_frame_name_", odom_frame_name_, "odom");

    // Initial frame
    //TODO get prior and prior_cov from nh_.param
    Eigen::Vector3s prior = Eigen::Vector3s(0, 0, 0);//prior pose of base in map
    Eigen::Matrix3s prior_cov = Eigen::Matrix3s::Identity() * 0.1;//prior pose covariance of base in map
    problem_.setOrigin(prior, prior_cov, wolf::TimeStamp(ros::Time::now().toSec()));

    // [init publishers]
    // Broadcast 0 transform map-odom
    T_map2odom_.frame_id_ = map_frame_name_;
    T_map2odom_.child_frame_id_ = odom_frame_name_;
    T_map2odom_.setData(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)));
    tfb_.sendTransform( T_map2odom_);

    // [init subscribers]
    odom_sub_ = nh_.subscribe("/teo/odomfused", 10, &WolfNode::odometryCallback, this);

    // set ceres options
    ceres_manager_.getSolverOptions().max_num_iterations = max_iterations;
    ceres_manager_.setUseWolfAutoDiff(use_wolf_auto_diff);

    // Install sensors and processors
    // odometry
    wolf::IntrinsicsOdom2D odom_intrinsics;
    odom_intrinsics.k_disp_to_disp = odom_std[0];
    odom_intrinsics.k_rot_to_rot = odom_std[1];
    wolf::SensorBase* odom_sensor_ptr = problem_.installSensor("ODOM 2D", "odometer", Eigen::VectorXs::Zero(3), &odom_intrinsics);
    problem_.installProcessor("ODOM 2D", "main odometry", "odometer");
    odom_capture_ptr_ = new wolf::CaptureMotion2(wolf::TimeStamp(), odom_sensor_ptr, Eigen::Vector2s::Zero(), Eigen::Matrix2s::Zero());

    // lasers
    laser_sensor_ptr_ = std::vector<wolf::SensorLaser2D*>(n_lasers, nullptr);
    laser_intrinsics_set_= std::vector<bool>(n_lasers, false);
    laser_extrinsics_set_= std::vector<bool>(n_lasers, false);
    laser_subscribers_.resize(n_lasers);
    laser_frame_name_.resize(n_lasers);
    std::stringstream lidar_frame_name_ii, lidar_topic_name_ii;
    for (unsigned int ii = 0; ii<n_lasers; ii++)
    {
        //build name
        lidar_frame_name_ii.str("");
        lidar_frame_name_ii << "laser_" << ii << "_frame_name";
        nh_.param<std::string>(lidar_frame_name_ii.str(), laser_frame_name_[ii]);
        //std::cout << "setting laser " << ii << "tf. frame id: " << laser_frame_name_[ii] << std::endl;

        // store id-name map
        laser_frame_2_idx_[laser_frame_name_[ii]] = ii;

        // install sensor
        Eigen::VectorXs lidar_extrinsics = Eigen::VectorXs::Zero(3);
        laser_extrinsics_set_[ii] = loadSensorExtrinsics(laser_frame_name_[ii], lidar_extrinsics);
        wolf::IntrinsicsLaser2D laser_2_intrinsics;
        problem_.installSensor("LASER 2D", lidar_frame_name_ii.str(), lidar_extrinsics, &laser_2_intrinsics);

        // install processor
        wolf::ProcessorParamsLaser laser_2_processor_params;
        laser_2_processor_params.line_finder_params_ = laserscanutils::LineFinderIterativeParams({0.1, 5});
        laser_2_processor_params.n_corners_th = 10;

        //init lidar subscribers
        lidar_topic_name_ii.str("");
        lidar_topic_name_ii << "laser_" << ii;
        laser_subscribers_[ii] = nh_.subscribe(lidar_topic_name_ii.str(),20,&WolfNode::laserCallback,this);
    }

    // [init publishers]
    constraints_publisher_ = nh_.advertise<visualization_msgs::Marker>("constraints", 2);
    landmarks_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("corners", 2);
    vehicle_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("vehicle", 2);

    // init MARKERS
    // init constraint markers message
    constraints_marker_msg_.type = visualization_msgs::Marker::LINE_LIST;
    constraints_marker_msg_.header.frame_id = "map";
    constraints_marker_msg_.scale.x = 0.1;
    constraints_marker_msg_.color.r = 1; //yellow
    constraints_marker_msg_.color.g = 1; //yellow
    constraints_marker_msg_.color.b = 0; //yellow
    constraints_marker_msg_.color.a = 1;
    constraints_marker_msg_.ns = "/constraints";
    constraints_marker_msg_.id = 1;

    // Init vehicle_MarkerArray_msg with a first RED CUBE marker representing the vehicle at agv_base_link.
    visualization_msgs::Marker vehicle_head_marker;
    vehicle_head_marker.header.stamp = ros::Time::now();
    vehicle_head_marker.header.frame_id = base_frame_name_;
    vehicle_head_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_head_marker.scale.x = 10;
    vehicle_head_marker.scale.y = 3;
    vehicle_head_marker.scale.z = 3;
    vehicle_head_marker.color.r = 1; //red
    vehicle_head_marker.color.g = 0;
    vehicle_head_marker.color.b = 0;
    vehicle_head_marker.color.a = 1;
    vehicle_head_marker.pose.position.x = 0.0;
    vehicle_head_marker.pose.position.y = 0.0;
    vehicle_head_marker.pose.position.z = 1.5;
    vehicle_head_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    vehicle_head_marker.id = 0;
    trajectory_marker_array_msg_.markers.push_back(vehicle_head_marker);

    /*// Init the rest of the vehicle_MarkerArray_msg with YELLOW CUBE markers, with neither pose nor stamp
    visualization_msgs::Marker vehicle_trajectory_marker;
    vehicle_trajectory_marker.header.frame_id = "map";
    vehicle_trajectory_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_trajectory_marker.scale.x = 10;
    vehicle_trajectory_marker.scale.y = 3;
    vehicle_trajectory_marker.scale.z = 3;
    vehicle_trajectory_marker.color.r = 1;//yellow
    vehicle_trajectory_marker.color.g = 1;//yellow
    vehicle_trajectory_marker.color.b = 0;
    vehicle_trajectory_marker.color.a = 0.0; //no show at while no true frame
    vehicle_trajectory_marker.pose.position.x = 0.0;
    vehicle_trajectory_marker.pose.position.y = 0.0;
    vehicle_trajectory_marker.pose.position.z = 1.5;
    vehicle_trajectory_marker.id = 1; //0 already taken by the current vehicle
    trajectory_marker_array_msg_.markers.push_back(vehicle_trajectory_marker); */


    ROS_INFO("STARTING IRI WOLF...");
}

WolfNode::~WolfNode()
{
    std::cout << std::endl << " ========= WolfNode DESTRUCTOR (should not crash) =============" << std::endl;
    odom_capture_ptr_->destruct();
}


void WolfNode::solve()
{
    // Solving ---------------------------------------------------------------------------
    //ROS_INFO("================ SOLVING ==================");
    //ros::Time local_stamp = ros::Time::now();

    ceres::Solver::Summary summary = ceres_manager_.solve();
    std::cout << "------------------------- SOLVED -------------------------" << std::endl;
    //std::cout << summary.FullReport() << std::endl;
    std::cout << summary.BriefReport() << std::endl;

    // Update transforms ---------------------------------------------------------------------------


    // MARKERS VEHICLE & CONSTRAINTS ---------------------------------------------------------------------------
    // [...]
}

void WolfNode::broadcastTf()
{
    // get current vehicle pose
    ros::Time loc_stamp = ros::Time::now();
    problem_.getCurrentState(vehicle_pose_);

    // Broadcast transform ---------------------------------------------------------------------------
    //Get map2base from Wolf result, and builds base2map pose
    T_map2base_.setOrigin( tf::Vector3((double) vehicle_pose_(0), (double) vehicle_pose_(1), 0) );
    T_map2base_.setRotation( tf::createQuaternionFromYaw((double) vehicle_pose_(2)) );

    //std::cout << "Loc: (" << vehicle_pose(0) << "," << vehicle_pose(1) << "," << vehicle_pose(2) << ")" << std::endl;

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    if ( tfl_.waitForTransform(base_frame_name_, odom_frame_name_, loc_stamp, ros::Duration(0.1)) )
    {
        tfl_.lookupTransform(base_frame_name_, odom_frame_name_, loc_stamp, T_base2odom_);
        T_map2odom_.setData(T_map2base_ * T_base2odom_);
        T_map2odom_.stamp_ = loc_stamp;

        //broadcast T_map2odom_
        tfb_.sendTransform( T_map2odom_ );
    }
    else
        ROS_WARN("No odom to base frame received");
}

void WolfNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ////ROS_INFO("WolfAlgNode::odometry_callback: New Message Received");
    odom_capture_ptr_->setTimeStamp(wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec));
    odom_capture_ptr_->setData(Eigen::Vector2s(msg->twist.twist.linear.x, msg->twist.twist.angular.z));
    odom_capture_ptr_->process();
}

const wolf::Problem& WolfNode::getProblem()
{
    return problem_;
}

void WolfNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("WolfAlgNode::laser_front_left_callback: New Message Received");

    //get the id from the message header
    unsigned int lidar_id = laser_frame_2_idx_[msg->header.frame_id];
    //std::cout << "laser_callback() starts. lidar_id: " << lidar_id << std::endl;

    if ( laser_sensor_ptr_[lidar_id] != nullptr ) //checks that the sensor exists
    {
        //sets the extrinsics if they are not set
        if (!laser_extrinsics_set_[lidar_id] )
        {
            Eigen::VectorXs lidar_extrinsics = Eigen::VectorXs::Zero(3);
            if (loadSensorExtrinsics(laser_frame_name_[lidar_id], lidar_extrinsics))
            {
                laser_sensor_ptr_[lidar_id]->getPPtr()->setVector(lidar_extrinsics.head(2));
                laser_sensor_ptr_[lidar_id]->getOPtr()->setVector(lidar_extrinsics.tail(1));
                laser_extrinsics_set_[lidar_id] = true;
            }
        }

        //updates laser scan params in case they are not set yet
        if (!laser_intrinsics_set_[lidar_id])
            loadLaserIntrinsics(lidar_id, msg);

        //create a new capture in the Wolf environment.
        wolf::CaptureLaser2D* new_capture = new wolf::CaptureLaser2D(wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                         laser_sensor_ptr_[lidar_id], msg->ranges);
        new_capture->process();
        //std::cout << "capture added" << std::endl;
    }
}

void WolfNode::loadLaserIntrinsics(const unsigned int _laser_idx, const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // laser intrinsic parameters
    laserscanutils::LaserScanParams params = laser_sensor_ptr_[_laser_idx]->getScanParams();
    params.angle_min_ = msg->angle_min;
    params.angle_max_ = msg->angle_max;
    params.angle_step_ = msg->angle_increment;
    params.scan_time_ = msg->time_increment;
    params.range_min_ = msg->range_min;
    params.range_max_ = msg->range_max;
    params.range_std_dev_ = 0.05; // TODO: get from param
    params.angle_std_dev_ = 0.05; // TODO: get from param
    laser_sensor_ptr_[_laser_idx]->setScanParams(params);
    laser_intrinsics_set_[_laser_idx] = true;
}

bool WolfNode::loadSensorExtrinsics(const std::string _sensor_frame, Eigen::VectorXs& _extrinsics)
{
    //look up for transform from base to ibeo
    //std::cout << "waiting for transform: " << _sensor_frame << std::endl;
    if ( tfl_.waitForTransform(base_frame_name_, _sensor_frame, ros::Time(0), ros::Duration(1.)) )
    {
        tf::StampedTransform base_2_sensor;

        //look up for transform at TF
        tfl_.lookupTransform(base_frame_name_, _sensor_frame, ros::Time(0), base_2_sensor);

        //Set mounting frame. Fill translation part
        _extrinsics << base_2_sensor.getOrigin().x(), base_2_sensor.getOrigin().y(), tf::getYaw(base_2_sensor.getRotation());
        //std::cout << _sensor_frame << ": " << << base_2_sensor.transpose() << std::endl;

        return true;
    }
    return false;
}
