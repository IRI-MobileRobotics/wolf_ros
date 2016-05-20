//
// Created by jvallve on 13/05/16.
//

#include "wolf_node.h"

WolfNode::WolfNode(char *argv0) :
    nh_(ros::this_node::getName()),
    problem_(wolf::FRM_PO_2D),
    origin_cov_(Eigen::Matrix3s::Zero()),
    origin_set_(false),
    ceres_manager_(&problem_),
    vehicle_pose_(3),
    last_odom_stamp_(ros::Time(0)),
    last_odom_seq_(0)
{
    std::cout << "WolfNode::WolfNode(...) -- constructor\n";

    // parameters
    bool use_wolf_auto_diff;
    int max_iterations, n_lasers, new_corners_th, loop_frames_th;
    double odom_std[2], origin_covs[3];
    nh_.param<bool>("use_wolf_auto_diff", use_wolf_auto_diff, true);
    nh_.param<int>("max_iterations", max_iterations, 1e3);
    nh_.param<int>("n_lasers", n_lasers, 6);
    nh_.param<int>("new_corners_th", new_corners_th, 3);
    nh_.param<int>("loop_frames_th", loop_frames_th, 3);
    nh_.param<double>("odometry_translational_std", odom_std[0], 0.2);
    nh_.param<double>("odometry_rotational_std", odom_std[1], 0.2);
    nh_.param<std::string>("base_frame_name", base_frame_name_, "agv_base_link");
    nh_.param<std::string>("map_frame_name_", map_frame_name_, "map");
    nh_.param<std::string>("odom_frame_name_", odom_frame_name_, "odom");
    nh_.param<double>("origin_covariance_xx", origin_covs[0], 0.1);
    nh_.param<double>("origin_covariance_yy", origin_covs[1], 0.1);
    nh_.param<double>("origin_covariance_thth", origin_covs[2], 0.1);
    origin_cov_(0,0) = origin_covs[0];
    origin_cov_(1,1) = origin_covs[1];
    origin_cov_(2,2) = origin_covs[2];

    // [init publishers]
    // Broadcast 0 transform map-odom
    T_map2odom_.frame_id_ = map_frame_name_;
    T_map2odom_.child_frame_id_ = odom_frame_name_;
    T_map2odom_.setData(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)));
    tfb_.sendTransform( T_map2odom_);

    // [init subscribers]
    odom_sub_ = nh_.subscribe("odometry", 10, &WolfNode::odometryCallback, this);

    // set ceres options
    ceres_manager_.getSolverOptions().max_num_iterations = max_iterations;
    ceres_manager_.setUseWolfAutoDiff(use_wolf_auto_diff);
    //google::InitGoogleLogging(argv0);

    // Install sensors and processors
    ROS_INFO("0");
    // odometry
    wolf::IntrinsicsOdom2D odom_intrinsics;
    odom_intrinsics.k_disp_to_disp = odom_std[0];
    odom_intrinsics.k_rot_to_rot = odom_std[1];
    wolf::SensorBase* odom_sensor_ptr = problem_.installSensor("ODOM 2D", "odometer", Eigen::VectorXs::Zero(3), &odom_intrinsics);
    problem_.installProcessor("ODOM 2D", "main odometry", "odometer");
    odom_capture_ptr_ = new wolf::CaptureMotion2(wolf::TimeStamp(), odom_sensor_ptr, Eigen::Vector2s::Zero(), Eigen::Matrix2s::Zero());

    // lasers
    ROS_INFO("1");
    laser_sensor_ptr_ = std::vector<wolf::SensorLaser2D*>(n_lasers, nullptr);
    laser_processor_ptr_ = std::vector<wolf::ProcessorTrackerLandmarkCorner*>(n_lasers, nullptr);
    laser_intrinsics_set_= std::vector<bool>(n_lasers, false);
    laser_extrinsics_set_= std::vector<bool>(n_lasers, false);
    laser_subscribers_.resize(n_lasers);
    laser_frame_name_.resize(n_lasers);
    std::stringstream lidar_frame_name_ii, lidar_processor_name_ii, lidar_topic_name_ii;
    for (auto ii = 0; ii<n_lasers; ii++)
    {
        //build names
        lidar_frame_name_ii.str("");
        lidar_frame_name_ii << "laser_" << ii << "_frame_name";
        lidar_processor_name_ii.str("");
        lidar_processor_name_ii << "laser_" << ii << "_processor";

        //get laser frame id
        nh_.param<std::string>(lidar_frame_name_ii.str(), laser_frame_name_[ii], "laser_i_frame_id");
        std::cout << "setting laser " << ii << " tf. frame id: " << laser_frame_name_[ii] << std::endl;

        // store id-name map
        laser_frame_2_idx_[laser_frame_name_[ii]] = ii;

        // install sensor
        Eigen::VectorXs lidar_extrinsics = Eigen::VectorXs::Zero(3);
        laser_extrinsics_set_[ii] = loadSensorExtrinsics(laser_frame_name_[ii], lidar_extrinsics);
        wolf::IntrinsicsLaser2D laser_2_intrinsics;
        laser_sensor_ptr_[ii] = (wolf::SensorLaser2D*)problem_.installSensor("LASER 2D", lidar_frame_name_ii.str(), lidar_extrinsics, &laser_2_intrinsics);

        // install processor
        wolf::ProcessorParamsLaser laser_processor_params;
        laser_processor_params.line_finder_params_ = laserscanutils::LineFinderIterativeParams({0.1, 5});
        laser_processor_params.new_corners_th = new_corners_th;
        laser_processor_params.loop_frames_th = loop_frames_th;
        laser_processor_ptr_[ii] = (wolf::ProcessorTrackerLandmarkCorner*)problem_.installProcessor("LASER 2D", lidar_processor_name_ii.str(), lidar_frame_name_ii.str(), &laser_processor_params);

        //init lidar subscribers
        lidar_topic_name_ii.str("");
        lidar_topic_name_ii << "laser_" << ii;
        laser_subscribers_[ii] = nh_.subscribe(lidar_topic_name_ii.str(),20,&WolfNode::laserCallback,this);
    }

    // [init publishers]
    constraints_publisher_ = nh_.advertise<visualization_msgs::Marker>("constraints", 2);
    landmarks_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("corners", 2);
    trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("vehicle", 2);

    // init MARKERS
    ROS_INFO("2");
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

    // Init vehicle_marker and create the first RED CUBE
    ROS_INFO("3");
    vehicle_marker_.header.frame_id = base_frame_name_;
    vehicle_marker_.type = visualization_msgs::Marker::CUBE;
    vehicle_marker_.scale.x = 10;
    vehicle_marker_.scale.y = 3;
    vehicle_marker_.scale.z = 3;
    vehicle_marker_.color.r = 1; //red
    vehicle_marker_.color.g = 0;
    vehicle_marker_.color.b = 0;
    vehicle_marker_.color.a = 1;
    vehicle_marker_.pose.position.x = 0.0;
    vehicle_marker_.pose.position.y = 0.0;
    vehicle_marker_.pose.position.z = 1.5;
    vehicle_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    vehicle_marker_.id = 0;
    trajectory_marker_array_msg_.markers.push_back(vehicle_marker_);
    vehicle_marker_.color.r = 1; //yellow
    vehicle_marker_.color.g = 1;
    vehicle_marker_.color.b = 0;
    vehicle_marker_.color.a = 1;

    // Init landmark markers
    ROS_INFO("4");
    landmark_marker_.header.frame_id = map_frame_name_;
    landmark_marker_.type = visualization_msgs::Marker::CUBE;
    landmark_marker_.ns = "/landmarks";
    landmark_marker_.scale.z = 3;
    landmark_marker_.pose.position.z = landmark_marker_.scale.z / 2;
    landmark_text_marker_.header.frame_id = map_frame_name_;
    landmark_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    landmark_text_marker_.ns = "/landmarks";
    landmark_text_marker_.color.r = 1;
    landmark_text_marker_.color.g = 1;
    landmark_text_marker_.color.b = 1;
    landmark_text_marker_.color.a = 1;
    landmark_text_marker_.scale.z = 1;

    // Init feature markers
    ROS_INFO("5");
    features_marker_array_msg_.resize(n_lasers);
    features_publisher_.resize(n_lasers);
    feature_marker_.type = visualization_msgs::Marker::CUBE;
    feature_marker_.scale.x = 1.5;
    feature_marker_.scale.y = 0.4;
    feature_marker_.scale.z = 1;
    feature_marker_.pose.position.z = feature_marker_.scale.z / 2;
    feature_marker_.color.a = 1;
    feature_marker_.ns = "/features";
    ROS_INFO("6");
    for (auto ii = 0; ii<n_lasers; ii++)
        features_publisher_[ii] = nh_.advertise<visualization_msgs::MarkerArray>(("features"+ std::to_string(ii)), 2);
    ROS_INFO("STARTING IRI WOLF...");
    // Initial frame
    unsigned int attempts = 0;
    while (!setOrigin() && attempts < 100)
    {
        usleep(1e4);
        attempts++;
    }
    if (attempts == 100)
        ROS_ERROR("WolfNode: Didn't receive odometry tf after 100 attempts");
    else
        ROS_INFO("WolfNode: Receive odometry tf after %i attempts", attempts);

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

    ceres::Solver::Summary summary = ceres_manager_.solve();
    //std::cout << "------------------------- SOLVED -------------------------" << std::endl;
    //std::cout << summary.BriefReport() << std::endl; //summary.FullReport()

    // Broadcast transforms ---------------------------------------------------------------------------
    broadcastTf();

    // MARKERS VEHICLE & CONSTRAINTS ---------------------------------------------------------------------------
    publishMarkers();
}

void WolfNode::broadcastTf()
{
    //ROS_INFO("================ broadcastTf ==================");

    // get current vehicle pose
    ros::Time loc_stamp;
    wolf::TimeStamp loc_ts;
    problem_.getCurrentState(vehicle_pose_, loc_ts);
    loc_stamp.nsec = loc_ts.getNanoSeconds();
    loc_stamp.sec = loc_ts.getSeconds();

    // Broadcast transform ---------------------------------------------------------------------------
    //Get map2base from Wolf result, and builds base2map pose
    T_map2base_.setOrigin( tf::Vector3((double) vehicle_pose_(0), (double) vehicle_pose_(1), 0) );
    T_map2base_.setRotation( tf::createQuaternionFromYaw((double) vehicle_pose_(2)) );

    std::cout << "Vehicle pose: " << vehicle_pose_.transpose() << std::endl;

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    if ( tfl_.waitForTransform(base_frame_name_, odom_frame_name_, loc_stamp, ros::Duration(0.1)) )
    {
        tfl_.lookupTransform(base_frame_name_, odom_frame_name_, loc_stamp, T_base2odom_);
        //std::cout << "Odometry: " << T_base2odom_.inverse().getOrigin().getX() << " " << T_base2odom_.inverse().getOrigin().getY() << " " << T_base2odom_.inverse().getRotation().getAngle() << std::endl;
        T_map2odom_.setData(T_map2base_ * T_base2odom_);
        T_map2odom_.stamp_ = loc_stamp;

        //broadcast T_map2odom_
        tfb_.sendTransform( T_map2odom_ );
    }
    else
        ROS_WARN("No odom to base frame received");
}

void WolfNode::publishMarkers()
{
    //ROS_INFO("================ publishMarkers ==================");

    ros::Time marker_stamp = ros::Time::now();

    // MARKERS VEHICLE & CONSTRAINTS
    wolf::ConstraintBaseList ctr_list;
    constraints_marker_msg_.points.clear();
    constraints_marker_msg_.header.stamp = marker_stamp;
    unsigned int ii = 1; //start from 1 to keep the vehicle_MarkerArray_msg_.marker[0] for the current vehicle state
    geometry_msgs::Point point1, point2;
    for (auto frm_ptr : (*problem_.getTrajectoryPtr()->getFrameListPtr())) //runs the list of frames
    {
        if (frm_ptr->isKey())
        {
            // KEY FRAMES
            if (trajectory_marker_array_msg_.markers.size() > ii)
                trajectory_marker_array_msg_.markers[ii].action = visualization_msgs::Marker::MODIFY;
            else
            {
                trajectory_marker_array_msg_.markers.push_back(vehicle_marker_);
                trajectory_marker_array_msg_.markers[ii].action = visualization_msgs::Marker::ADD;
                trajectory_marker_array_msg_.markers[ii].id = ii;
            }
            trajectory_marker_array_msg_.markers[ii].header.stamp = marker_stamp;
            trajectory_marker_array_msg_.markers[ii].header.frame_id = "map";
            trajectory_marker_array_msg_.markers[ii].pose.position.x = frm_ptr->getPPtr()->getVector()(0);
            trajectory_marker_array_msg_.markers[ii].pose.position.y = frm_ptr->getPPtr()->getVector()(1);
            trajectory_marker_array_msg_.markers[ii].pose.orientation = tf::createQuaternionMsgFromYaw( frm_ptr->getOPtr()->getVector()(0) );
            trajectory_marker_array_msg_.markers[ii].color.a = 0.5; //Show with little transparency

            // CONSTRAINTS
            ctr_list.clear();
            frm_ptr->getConstraintList(ctr_list);
            for (auto c_ptr : ctr_list)
            {
                switch (c_ptr->getCategory())
                {
                    // Odometry
                    case wolf::CTR_FRAME:
                    {
                        // from
                        point1.x = frm_ptr->getPPtr()->getVector()(0);
                        point1.y = frm_ptr->getPPtr()->getVector()(1);
                        point1.z = 0.25;
                        // to
                        point2.x = c_ptr->getFrameOtherPtr()->getPPtr()->getVector()(0);
                        point2.y = c_ptr->getFrameOtherPtr()->getPPtr()->getVector()(1);
                        point2.z = 0.25;
                        break;
                    }
                    // Landmarks
                    case wolf::CTR_LANDMARK:
                    {
                        // from
                        point1.x = frm_ptr->getPPtr()->getVector()(0);
                        point1.y = frm_ptr->getPPtr()->getVector()(1);
                        point1.z = 0.25;
                        // to
                        point2.x = c_ptr->getLandmarkOtherPtr()->getPPtr()->getVector()(0);
                        point2.y = c_ptr->getLandmarkOtherPtr()->getPPtr()->getVector()(1);
                        point2.z = 1.5;
                        break;
                    }
                }
                constraints_marker_msg_.header.stamp = marker_stamp;
                constraints_marker_msg_.points.push_back(point1);
                constraints_marker_msg_.points.push_back(point2);
            }

            ii++;
        }
    }

    // MARKERS LANDMARKS
    ii = 0;
    landmark_marker_array_msg_.markers.clear();
    for (auto l_ptr : *(problem_.getMapPtr()->getLandmarkListPtr()))
    {
        landmark_marker_.header.stamp = marker_stamp;
        landmark_marker_.color.r = (double)l_ptr->getHits()/10;
        landmark_marker_.color.g = 0;
        landmark_marker_.color.b = 1 - (double)l_ptr->getHits()/10;
        landmark_marker_.color.a = 0.5;//0.3 + 0.7*((double)l_ptr->getHits()/10);
        landmark_marker_.id = 2*ii;


        if (l_ptr->getType() == wolf::LANDMARK_CORNER)
        {
            landmark_marker_.scale.x = 1.5;
            landmark_marker_.scale.y = 0.2;

            landmark_marker_.pose.position.x = l_ptr->getPPtr()->getVector()(0) + landmark_marker_.scale.x / 2 * cos(l_ptr->getOPtr()->getVector()(0));
            landmark_marker_.pose.position.y = l_ptr->getPPtr()->getVector()(1) + landmark_marker_.scale.x / 2 * sin(l_ptr->getOPtr()->getVector()(0));
            landmark_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(l_ptr->getOPtr()->getVector()(0));


        }
        else if (l_ptr->getType() == wolf::LANDMARK_CONTAINER)
        {
            landmark_marker_.scale.x = l_ptr->getDescriptor()(1);
            landmark_marker_.scale.y = l_ptr->getDescriptor()(0);

            landmark_marker_.pose.position.x = l_ptr->getPPtr()->getVector()(0);
            landmark_marker_.pose.position.y = l_ptr->getPPtr()->getVector()(1);
            landmark_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(l_ptr->getOPtr()->getVector()(0));

        }
        landmark_marker_array_msg_.markers.push_back(landmark_marker_);

        landmark_text_marker_.pose.position = landmark_marker_.pose.position;
        landmark_text_marker_.pose.position.z = 3;
        landmark_text_marker_.id = 2*ii+1;
        landmark_text_marker_.text = std::to_string(l_ptr->id());

        landmark_marker_array_msg_.markers.push_back(landmark_text_marker_);

        ii++;
    }


    trajectory_publisher_.publish(trajectory_marker_array_msg_);
    landmarks_publisher_.publish(landmark_marker_array_msg_);
    constraints_publisher_.publish(constraints_marker_msg_);
}

void WolfNode::publishProcessorMarkers(const unsigned int _lidar_id)
{
    //ROS_INFO("================ publishProcessorMarkers ==================");

    // MARKERS FEATURES
    unsigned int ii = 0;
    features_marker_array_msg_[_lidar_id].markers.clear();
    for (auto feat_ptr : laser_processor_ptr_[_lidar_id]->getNewFeaturesListLast())
        if (feat_ptr->getType() == wolf::FEATURE_CORNER)
        {
            feature_marker_.header.frame_id = laser_frame_name_[_lidar_id];
            feature_marker_.header.stamp = ros::Time::now();
            feature_marker_.id = features_marker_array_msg_[_lidar_id].markers.size();
            feature_marker_.color.r = 0;
            feature_marker_.color.g = 1;
            feature_marker_.color.b = 1;
            feature_marker_.pose.position.x = feat_ptr->getMeasurement(0) + feature_marker_.scale.x / 2 * cos(feat_ptr->getMeasurement(2));
            feature_marker_.pose.position.y = feat_ptr->getMeasurement(1) + feature_marker_.scale.x / 2 * sin(feat_ptr->getMeasurement(2));
            feature_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(feat_ptr->getMeasurement(2));

            features_marker_array_msg_[_lidar_id].markers.push_back(feature_marker_);
            features_marker_array_msg_[_lidar_id].markers.back().action = visualization_msgs::Marker::ADD;
        }

    for (auto feat_ptr : *(laser_processor_ptr_[_lidar_id]->getLastPtr()->getFeatureListPtr()))
        if (feat_ptr->getType() == wolf::FEATURE_CORNER)
        {
            feature_marker_.header.frame_id = laser_frame_name_[_lidar_id];
            feature_marker_.header.stamp = ros::Time::now();
            feature_marker_.id = features_marker_array_msg_[_lidar_id].markers.size();
            feature_marker_.color.r = 0;
            feature_marker_.color.g = 1;
            feature_marker_.color.b = 0;
            feature_marker_.pose.position.x = feat_ptr->getMeasurement(0) + feature_marker_.scale.x / 2 * cos(feat_ptr->getMeasurement(2));
            feature_marker_.pose.position.y = feat_ptr->getMeasurement(1) + feature_marker_.scale.x / 2 * sin(feat_ptr->getMeasurement(2));
            feature_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(feat_ptr->getMeasurement(2));

            features_marker_array_msg_[_lidar_id].markers.push_back(feature_marker_);
            features_marker_array_msg_[_lidar_id].markers.back().action = visualization_msgs::Marker::ADD;
        }
    features_publisher_[_lidar_id].publish(features_marker_array_msg_[_lidar_id]);
}

void WolfNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("WolfNode::odometry_callback: New Message Received");

    if (msg->header.seq - last_odom_seq_ > 1)
        ROS_ERROR("WolfNode::odometryCallback: LOST ODOMETRY MESSAGE!");

    if (last_odom_stamp_ != ros::Time(0))
    {
        wolf::Scalar dt = (msg->header.stamp - last_odom_stamp_).toSec();
        odom_capture_ptr_->setTimeStamp(wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec));
        odom_capture_ptr_->setData(Eigen::Vector2s(msg->twist.twist.linear.x*dt, msg->twist.twist.angular.z*dt));
        odom_capture_ptr_->process();
    }
    last_odom_stamp_ = msg->header.stamp;
    last_odom_seq_ = msg->header.seq;
}

const wolf::Problem& WolfNode::getProblem()
{
    return problem_;
}

void WolfNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("WolfNode::laserCallback: New Message Received");
    //std::cout << "stamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nsec << std::endl;

    //get the id from the message header
    unsigned int lidar_id = laser_frame_2_idx_[msg->header.frame_id];
    //std::cout << "laser_callback() starts. lidar_id: " << lidar_id << std::endl;

    // Check if the sensor exists
    if ( laser_sensor_ptr_[lidar_id] != nullptr )
    {
        //sets the extrinsics if they are not set yet
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

        // sets laser scan params if they are not set yet
        if (!laser_intrinsics_set_[lidar_id])
            loadLaserIntrinsics(lidar_id, msg);

        //create a new capture in the Wolf environment (only for messages after origin stamp)
        if ( msg->header.stamp > origin_stamp_ )
        {
            wolf::CaptureLaser2D* new_capture = new wolf::CaptureLaser2D(wolf::TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                                         laser_sensor_ptr_[lidar_id], msg->ranges);
            new_capture->process();
            publishProcessorMarkers(lidar_id);
        }
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
    std::cout << "WolfNode::loadSensorExtrinsics: waiting transform " << _sensor_frame << std::endl;
    if ( tfl_.waitForTransform(base_frame_name_, _sensor_frame, ros::Time(0), ros::Duration(1.)) )
    {
        tf::StampedTransform base_2_sensor;

        //look up for transform at TF
        tfl_.lookupTransform(base_frame_name_, _sensor_frame, ros::Time(0), base_2_sensor);

        //Set mounting frame. Fill translation part
        _extrinsics << base_2_sensor.getOrigin().x(), base_2_sensor.getOrigin().y(), tf::getYaw(base_2_sensor.getRotation());
        std::cout << _sensor_frame << ": " << _extrinsics.transpose() << std::endl;

        return true;
    }
    return false;
}

bool WolfNode::setOrigin()
{
    //look up for first transform base 2 odom
    origin_stamp_ = ros::Time::now() - tfl_.getCacheLength();
    while ( !tfl_.canTransform(base_frame_name_, odom_frame_name_, origin_stamp_) )
    {
        origin_stamp_ += ros::Duration(0.01);
        if (origin_stamp_ >=  ros::Time::now())
            return false;
    }

    // Get first odometry
    tfl_.lookupTransform(base_frame_name_, odom_frame_name_, origin_stamp_, T_base2odom_);
    std::cout << "WolfNode::setOrigin: First odometry: " << T_base2odom_.inverse().getOrigin().getX() << " " << T_base2odom_.inverse().getOrigin().getY() << " " << tf::getYaw(T_base2odom_.inverse().getRotation()) << std::endl;
    std::cout << "stamp: " << T_base2odom_.stamp_.sec << "." << T_base2odom_.stamp_.nsec << std::endl;

    // set wolf origin
    Eigen::Vector3s prior = Eigen::Vector3s(T_base2odom_.inverse().getOrigin().getX(), T_base2odom_.inverse().getOrigin().getY(), tf::getYaw(T_base2odom_.inverse().getRotation()));//prior pose of base in map
    problem_.setOrigin(prior, origin_cov_, wolf::TimeStamp(T_base2odom_.stamp_.sec, T_base2odom_.stamp_.nsec));

    // set transformations
    T_map2base_ = T_base2odom_.inverse();
    T_map2odom_.setOrigin( tf::Vector3(0,0,0) );
    T_map2odom_.setRotation( tf::createQuaternionFromYaw(0) );
    T_map2odom_.stamp_ = ros::Time::now();

    //broadcast T_map2odom_
    tfb_.sendTransform( T_map2odom_ );

    return true;

}


