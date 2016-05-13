//
// Created by jvallve on 13/05/16.
//

#include "wolf_node.h"

WolfNode::WolfNode() :
    nh_(ros::this_node::getName()),
    last_odom_stamp_(0),
    problem_(wolf::FRM_PO_2D)
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

    // Initial frame
    //TODO get prior and prior_cov from nh_.param
    Eigen::Vector3s prior = Eigen::Vector3s(0, 0, 0);//prior pose of base in map
    Eigen::Matrix3s prior_cov = Eigen::Matrix3s::Identity() * 0.1;//prior pose covariance of base in map
    problem_.setOrigin(prior, prior_cov, wolf::TimeStamp(ros::Time::now()));

    // TODO: get via parameter
    base_frame_name_ = "teo_base_footprint";
    gps_frame_name_ = "gps_asterx1";
    world_frame_name_ = "world";
    map_frame_name_ = "map";
    odom_frame_name_ = "teo_odom";
    map_initial_frame_name_ = "map_initial";

    // [init publishers]
    // Broadcast 0 transform to align frames initially
    tfb_.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0)), ros::Time::now(), map_frame_name_, odom_frame_name_));

    // [init subscribers]
    odom_sub_ = nh_.subscribe("/teo/odomfused", 10, &WolfNode::odometryCallback, this);

    // init ceres
    //TODO check again with other ceres options
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION;
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = max_iterations;
    ceres_manager_ = CeresManager(&problem_, ceres_options, use_wolf_auto_diff);

    // Install sensors and processors
    // odometry
    wolf::IntrinsicsOdom2D odom_intrinsics;
    odom_intrinsics.k_disp_to_disp = odom_std[0];
    odom_intrinsics.k_rot_to_rot = odom_std[1];
    problem_.installSensor("ODOM 2D", "odometer", Eigen::VectorXs::Zero(3), &odom_intrinsics);
    problem_.installProcessor("ODOM 2D", "main odometry", "odometer");

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
        Eigen::VectorXs lidar_extrinsics = Eigen::VectorXs::Zeros(3);
        laser_extrinsics_set_[ii] = loadLaserExtrinsics(ii, lidar_extrinsics);
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
        lines_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("lines", 2);
        corners_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("corners", 2);
        vehicle_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("vehicle", 2);

        // Broadcast 0 transform to align frames initially
        T_map2base_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
        tfb_.sendTransform( tf::StampedTransform(T_map2base_, ros::Time::now(), "map", "odom"));



        // init MARKERS
        line_colors_.resize(n_lasers_);
        // init constraint markers message
        constraints_Marker_msg_.type = visualization_msgs::Marker::LINE_LIST;
        constraints_Marker_msg_.header.frame_id = "map";
        constraints_Marker_msg_.scale.x = 0.1;
        constraints_Marker_msg_.color.r = 1; //yellow
        constraints_Marker_msg_.color.g = 1; //yellow
        constraints_Marker_msg_.color.b = 0; //yellow
        constraints_Marker_msg_.color.a = 1;
        constraints_Marker_msg_.ns = "/constraints";
        constraints_Marker_msg_.id = 1;

        //init lidar lines marker array
        visualization_msgs::Marker line_list_Marker_msg;
        for (unsigned int i=0; i<n_lasers; i++)
        {
            if (i == 0)
                line_colors_[0].r = 0; line_colors_[0].g = 0; line_colors_[0].b = 1; line_colors_[0].a = 1;
            if (i == 1)
                line_colors_[1].r = 1; line_colors_[1].g = 0; line_colors_[1].b = 1; line_colors_[1].a = 1;
            if (i ==  2)
                line_colors_[2].r = 1; line_colors_[2].g = 0; line_colors_[2].b = 0; line_colors_[2].a = 1;
            if (i ==  3)
                line_colors_[3].r = 1; line_colors_[3].g = 1; line_colors_[3].b = 0; line_colors_[3].a = 1;
            if (i ==  4)
                line_colors_[4].r = 0; line_colors_[4].g = 1; line_colors_[4].b = 0; line_colors_[4].a = 1;
            if (i >= 5)
                line_colors_[i].r = 0; line_colors_[i].g = 1; line_colors_[i].b = 1; line_colors_[i].a = 1;

            line_list_Marker_msg.header.stamp = ros::Time::now();
            line_list_Marker_msg.header.frame_id = base_frame_name_;
            line_list_Marker_msg.type = visualization_msgs::Marker::LINE_LIST;
            line_list_Marker_msg.scale.x = 0.1;
            line_list_Marker_msg.color = line_colors[i];
            line_list_Marker_msg.ns = "/lines";
            line_list_Marker_msg.id = i;
            lines_MarkerArray_msg_.markers.push_back(line_list_Marker_msg);
        }

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
        vehicle_MarkerArray_msg_.markers.push_back(vehicle_head_marker);

        // Init the rest of the vehicle_MarkerArray_msg with YELLOW CUBE markers, with neither pose nor stamp
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
        vehicle_MarkerArray_msg_.markers.push_back(vehicle_trajectory_marker);



        ROS_INFO("STARTING IRI WOLF...");


}

WolfNode::~WolfNode()
{
    std::cout << std::endl << " ========= WolfNode DESTRUCTOR (should not crash) =============" << std::endl;
}


void WolfNode::solve()
{
    ROS_INFO("================ SOLVING ==================");
    ros::Time local_stamp = ros::Time::now();

    ceres::Solver::Summary summary = ceres_manager_.solve();
    std::cout << "------------------------- SOLVED -------------------------" << std::endl;
//    std::cout << summary.FullReport() << std::endl;
    std::cout << summary.BriefReport() << std::endl;

    // Sets localization timestamp & Gets wolf localization estimate
    time_last_process_ = ros::Time::now();
    //Eigen::Vector3s vehicle_pose  = getVehiclePose();


    // Broadcast transforms ---------------------------------------------------------------------------

    //End Broadcast transform -----------------------------------------------------------------------------
    // [fill msg structures]

    // MARKERS VEHICLE & CONSTRAINTS
    // [...]
}

void WolfNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ////ROS_INFO("WolfAlgNode::odometry_callback: New Message Received");
    //if (last_odom_stamp_ != ros::Time(0))
    //{
    //    float dt = (msg->header.stamp - last_odom_stamp_).toSec();
    //
    //    addCapture(new wolf::CaptureOdom2D(wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
    //            wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
    //                                 sensor_prior_,
    //                                 Eigen::Vector3s(msg->twist.twist.linear.x*dt, 0. ,msg->twist.twist.angular.z*dt)));
    //}
    //last_odom_stamp_ = msg->header.stamp;
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

bool WolfNode::loadLaserExtrinsics(const unsigned int laser_idx, Eigen::VectorXs& _extrinsics)
{
    tf::StampedTransform base_2_lidar_ii;

    //look up for transform from base to ibeo
    //std::cout << "waiting for transform: " << laser_frame_name_[laser_idx] << std::endl;
    if ( tfl_.waitForTransform(base_frame_name_, laser_frame_name_[laser_idx], ros::Time(0), ros::Duration(1.)) )
    {
        //look up for transform at TF
        tfl_.lookupTransform(base_frame_name_, laser_frame_name_[laser_idx], ros::Time(0), base_2_lidar_ii);

        //Set mounting frame. Fill translation part
        _extrinsics << base_2_lidar_ii.getOrigin().x(), base_2_lidar_ii.getOrigin().y(), tf::getYaw(base_2_lidar_ii.getRotation());
        //std::cout << "LIDAR " << laser_idx << ": " << laser_frame_name_[laser_idx] << ": " << laser_sensor_pose.transpose() << std::endl;

        return true;
    }
    else
        return false;
}
