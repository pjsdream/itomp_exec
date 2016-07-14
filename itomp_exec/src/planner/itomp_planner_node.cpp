#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>
#include <functional>

#include <std_msgs/Float64MultiArray.h>


namespace itomp_exec
{

ITOMPPlannerNode::ITOMPPlannerNode(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle)
    : moveit_robot_model_(robot_model)
    , node_handle_(node_handle)
    , planning_scene_(node_handle)
{
    initialize();
}

ITOMPPlannerNode::ITOMPPlannerNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // load robot model from robot_description
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit_robot_model_ = robot_model_loader.getModel();

    initialize();
}

void ITOMPPlannerNode::initialize()
{
    ros::Rate rate(0.5);

    // initialize virtual human arm publisher
    virtual_human_arm_request_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/future_obstacle_publisher/virtual_human_arm_request", 1);

    // initialize planning scene visualization publisher
    planning_scene_visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("planning_scene", 1);

    // initialize robot model from moveit model
    robot_model_.reset( new BoundingSphereRobotModel );
    robot_model_->initFromMoveitRobotModel(moveit_robot_model_);
            
    // initialize trajectory_execution_manager with robot model
    // see how it is initialized with ros params at http://docs.ros.org/indigo/api/moveit_ros_planning/html/trajectory__execution__manager_8cpp_source.html#l00074
    std::string controller;
    if (node_handle_.getParam("moveit_controller_manager", controller))
    {
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_));
    }
    else if (node_handle_.getParam("/move_group/moveit_controller_manager", controller))
    {
        // copy parameters from "/move_group" namespace to "~"
        bool moveit_manage_controllers = false;
        bool allowed_execution_duration_scaling = false;
        bool allowed_goal_duration_margin = false;
        bool controller_list_declared = false;
        XmlRpc::XmlRpcValue controller_list;
        double value;
        
        node_handle_.setParam("moveit_controller_manager", controller);
        
        if (node_handle_.getParam("/move_group/controller_list", controller_list))
        {
            controller_list_declared = true;
            node_handle_.setParam("controller_list", controller_list);
        }
        
        if (node_handle_.getParam("/move_group/moveit_manage_controllers", value))
        {
            moveit_manage_controllers = true;
            node_handle_.setParam("moveit_manage_controllers", value);
        }
        
        if (node_handle_.getParam("/move_group/allowed_execution_duration_scaling", value))
        {
            allowed_execution_duration_scaling = true;
            node_handle_.setParam("allowed_execution_duration_scaling", value);
        }
        
        if (node_handle_.getParam("/move_group/allowed_goal_duration_margin", value))
        {
            allowed_goal_duration_margin = true;
            node_handle_.setParam("allowed_goal_duration_margin", value);
        }
        
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_));
        
        node_handle_.deleteParam("moveit_controller_manager");
        
        if (controller_list_declared)
            node_handle_.deleteParam("controller_list");
        
        if (moveit_manage_controllers)
            node_handle_.deleteParam("moveit_manage_controllers");
        
        if (allowed_execution_duration_scaling)
            node_handle_.deleteParam("allowed_execution_duration_scaling");
        
        if (allowed_goal_duration_margin)
            node_handle_.deleteParam("allowed_goal_duration_margin");
    }
    else
    {
        ROS_WARN("Controller is not defined. MoveItFakeControllerManager is used.");
        node_handle_.setParam("moveit_controller_manager", "moveit_fake_controller_manager/MoveItFakeControllerManager");
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_, true));
        node_handle_.deleteParam("moveit_controller_manager");
    }
    
    // load ITOMP parameters
    loadParams();

    ROS_INFO("Sleep 0.5 sec for initializing publisher");
    rate.sleep();
}

void ITOMPPlannerNode::loadParams()
{
    // load options
    node_handle_.param("num_trajectories", options_.num_trajectories, 8);
    node_handle_.param("trajectory_duration", options_.trajectory_duration, 5.0);
    node_handle_.param("num_milestones", options_.num_milestones, 10);
    node_handle_.param("num_interpolation_samples", options_.num_interpolation_samples, 10);
    node_handle_.param("planning_timestep", options_.planning_timestep, 0.5);
    node_handle_.param("dynamic_obstacle_max_speed", options_.conservative_algorithm.dynamic_obstacle_max_speed, 0.1);
    node_handle_.param("dynamic_obstacle_duration", options_.conservative_algorithm.dynamic_obstacle_duration, 1.0);
    node_handle_.param("goal_tolerance", options_.goal_tolerance, 1e-6);

    // load ITOMP algorithm
    std::string itomp_algorithm;
    node_handle_.param("itomp_algorithm", itomp_algorithm, std::string("fixed_trajectory_duration"));

    if (itomp_algorithm == "fixed_trajectory_duration")
        options_.itomp_algorithm = ITOMPFixedTrajectoryDuration;

    else if (itomp_algorithm == "flexible_trajectory_duration")
        options_.itomp_algorithm = ITOMPFlexibleTrajectoryDuration;
    
    // load static obstacles
    XmlRpc::XmlRpcValue static_obstacles;
    if (node_handle_.getParam("static_obstacles", static_obstacles))
    {
        for (int i=0; i<static_obstacles.size(); i++)
        {
            XmlRpc::XmlRpcValue& static_obstacle = static_obstacles[i];
            
            if (static_obstacle.hasMember("type"))
            {
                std::string type = static_cast<std::string>(static_obstacle["type"]);

                if (type == "mesh")
                {
                    if (static_obstacle.hasMember("file"))
                    {
                        std::string filename = static_cast<std::string>(static_obstacle["file"]);
                        Eigen::Affine3d transformation = Eigen::Affine3d::Identity();

                        if (static_obstacle.hasMember("position"))
                        {
                            Eigen::Vector3d p;
                            XmlRpc::XmlRpcValue& position = static_obstacle["position"];
                            for (int i=0; i<3; i++)
                                p(i) = static_cast<double>(position[i]);

                            transformation.translate(p);
                        }

                        if (static_obstacle.hasMember("orientation"))
                        {
                            // TODO
                        }

                        if (static_obstacle.hasMember("scale"))
                        {
                            // TODO
                        }

                        addStaticObstacle(filename, transformation);
                    }
                    else
                    {
                        ROS_ERROR("ITOMP static_obstacle parameter does not have 'file' field.");
                        continue;
                    }
                }
                else if (type == "sphere")
                {
                    Eigen::Vector3d p(0., 0., 0.);
                    double radius = 1.;

                    if (static_obstacle.hasMember("radius"))
                        radius = static_cast<double>(static_obstacle["radius"]);

                    if (static_obstacle.hasMember("position"))
                    {
                        XmlRpc::XmlRpcValue& position = static_obstacle["position"];
                        for (int i=0; i<3; i++)
                            p(i) = static_cast<double>(position[i]);
                    }

                    planning_scene_.addStaticSphereObstacle(p, radius);
                }
            }
            else
            {
                ROS_ERROR("ITOMP static_obstacle parameter does not have 'type' field.");
                continue;
            }
        }
    }
    planning_scene_.setVisualizationTopic("planning_scene");
    
    // load cost weights
    options_.cost_weights.clear();
    XmlRpc::XmlRpcValue cost_weights;
    if (node_handle_.getParam("cost_weights", cost_weights))
    {
        for (XmlRpc::XmlRpcValue::iterator it=cost_weights.begin(); it != cost_weights.end(); it++)
        {
            const double weight = static_cast<double>(it->second);
            options_.cost_weights.push_back(std::make_pair(it->first, weight));
        }
    }
}

void ITOMPPlannerNode::printParams()
{
    ROS_INFO("ITOMP-exec parameters:");
    
    ROS_INFO(" * trajectory_duration: %lf", options_.trajectory_duration);
    ROS_INFO(" * planning_timestep: %lf", options_.planning_timestep);
    ROS_INFO(" * num_milestones: %d", options_.num_milestones);
    ROS_INFO(" * num_interpolation_samples: %d", options_.num_interpolation_samples);
}

void ITOMPPlannerNode::printControllers()
{
    ROS_INFO("ITOMP-exec controllers:");
    std::vector<std::string> controllers;
    trajectory_execution_manager_->getControllerManager()->getControllersList(controllers);
    for (int i=0; i<controllers.size(); i++)
        ROS_INFO(" * %s", controllers[i].c_str());
    
    ROS_INFO("ITOMP-exec active controllers:");   
    std::vector<std::string> active_controllers;
    trajectory_execution_manager_->getControllerManager()->getActiveControllers(active_controllers);
    for (int i=0; i<active_controllers.size(); i++)
        ROS_INFO(" * %s", active_controllers[i].c_str());
}

void ITOMPPlannerNode::printCostWeights()
{
    ROS_INFO("ITOMP-exec cost weights:");
    
    for (int i=0; i<options_.cost_weights.size(); i++)
        ROS_INFO(" * %s: %lf", options_.cost_weights[i].first.c_str(), options_.cost_weights[i].second);
}

void ITOMPPlannerNode::addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation)
{
    planning_scene_.addStaticObstacle(mesh_filename, transformation);
}

void ITOMPPlannerNode::addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations)
{
    planning_scene_.addStaticObstacles(mesh_filenames, transformations);
}

void ITOMPPlannerNode::setMotionPlanRequest(const planning_interface::MotionPlanRequest& req)
{
    planning_group_name_ = req.group_name;
    
    // initialize start state
    start_state_.setRobotModel(robot_model_);
    start_state_.setPlanningGroup(planning_group_name_);
    start_state_.initFromMoveitRobotStateMsg(req.start_state);
    
    // goal poses. Only take the first constraint
    goal_link_positions_.clear();
    for (int i=0; i<req.goal_constraints[0].position_constraints.size(); i++)
    {
        const std::string& name = req.goal_constraints[0].position_constraints[i].link_name;
        Eigen::Vector3d position;
        tf::vectorMsgToEigen(req.goal_constraints[0].position_constraints[i].target_point_offset, position);
        goal_link_positions_.push_back(std::make_pair(name, position));
    }
    
    goal_link_orientations_.clear();
    for (int i=0; i<req.goal_constraints[0].orientation_constraints.size(); i++)
    {
        const std::string& name = req.goal_constraints[0].orientation_constraints[i].link_name;
        Eigen::Quaterniond orientation;
        tf::quaternionMsgToEigen(req.goal_constraints[0].orientation_constraints[i].orientation, orientation);
        goal_link_orientations_.push_back(std::make_pair(name, orientation));
    }
}

Eigen::Affine3d ITOMPPlannerNode::getRobotRootTransform()
{
    const std::string root_link_name = robot_model_->getFrameId();

    Eigen::Affine3d transform;
    transform.setIdentity();

    tf::StampedTransform tf_transform;
    ros::Time time;
    std::string error_string;
    if (transform_listener_.getLatestCommonTime("map", root_link_name, time, &error_string) != tf::NO_ERROR)
    {
        ROS_ERROR("TF error: %s", error_string.c_str());
        ROS_ERROR("Set transform from map to [%s] to identity", root_link_name.c_str());
    }
    else
    {
        transform_listener_.lookupTransform("map", root_link_name, time, tf_transform);

        tf::Point tf_translation = tf_transform.getOrigin();
        tf::Quaternion tf_quaternion = tf_transform.getRotation();

        const Eigen::Vector3d translation(tf_translation.x(), tf_translation.y(), tf_translation.z());
        const Eigen::Quaterniond quaternion(tf_quaternion.w(), tf_quaternion.x(), tf_quaternion.y(), tf_quaternion.z());
        transform.translate(translation).rotate(quaternion);
    }

    return transform;
}

void ITOMPPlannerNode::visualizePlanningScene()
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.a = 1.;

    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;

    const Spheres& static_obstacle_spheres = planning_scene_.getStaticSphereObstacles();
    const Spheres& dynamic_obstacle_spheres = planning_scene_.getDynamicSphereObstacles();

    // static obstacles as green
    marker.ns = "planning_scene_static";
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    for (int i=0; i<static_obstacle_spheres.size(); i++)
    {
        const Sphere& sphere = static_obstacle_spheres[i];

        marker.id = i;

        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2.;

        marker_array.markers.push_back(marker);
    }

    // dynamic obstacles as red
    marker.ns = "planning_scene_dynamic";
    marker.color.r = 1.;
    marker.color.g = 0.;
    for (int i=0; i<dynamic_obstacle_spheres.size(); i++)
    {
        const Sphere& sphere = dynamic_obstacle_spheres[i];

        marker.id = i;

        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = sphere.radius * 2.;

        marker_array.markers.push_back(marker);
    }

    // future dynamic obstacles as transparent red
    marker.ns = "planning_scene_dynamic_future";
    marker.color.a = 0.5;
    for (int i=0; i<dynamic_obstacle_spheres.size(); i++)
    {
        const Sphere& sphere = dynamic_obstacle_spheres[i];

        marker.id = i;

        tf::pointEigenToMsg(sphere.position, marker.pose.position);
        marker.scale.x = marker.scale.y = marker.scale.z = (sphere.radius + options_.conservative_algorithm.dynamic_obstacle_duration * options_.conservative_algorithm.dynamic_obstacle_max_speed) * 2.;

        marker_array.markers.push_back(marker);
    }

    planning_scene_visualization_publisher_.publish(marker_array);
}

static void optimizeSingleTrajectoryCleanup(void* optimizer)
{
    ITOMPOptimizer* casted_optimizer = (ITOMPOptimizer*)optimizer;
    casted_optimizer->optimizeThreadCleanup();
}

static void* optimizeSingleTrajectory(void* optimizer)
{
    Thread::self()->cleanupPush(optimizeSingleTrajectoryCleanup, optimizer);

    ITOMPOptimizer* casted_optimizer = (ITOMPOptimizer*)optimizer;
    casted_optimizer->optimize();

    return NULL;
}

void ITOMPPlannerNode::optimizeAllTrajectories()
{
    double optimization_time_limit = 0.;
    for (int i=0; i<optimizers_.size(); i++)
    {
        double time_limit = optimizers_[i].getOptimizationTimeLimit();

        if (optimization_time_limit < time_limit)
            optimization_time_limit = time_limit;
    }

    for (int i=0; i<optimizers_.size(); i++)
        optimizer_threads_[i] = new Thread(&optimizeSingleTrajectory, &optimizers_[i]);

    // cancel all threads after optimization
    // the created threads are canceled
    ros::Duration(optimization_time_limit).sleep();

    for (int i=0; i<optimizers_.size(); i++)
        optimizer_threads_[i]->cancel();

    for (int i=0; i<optimizers_.size(); i++)
    {
        optimizer_threads_[i]->join();
        delete optimizer_threads_[i];
    }
}

bool ITOMPPlannerNode::planAndExecute(planning_interface::MotionPlanResponse& res)
{
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(moveit_robot_model_, planning_group_name_));

    initializeOptimizers();

    switch (options_.itomp_algorithm)
    {
    case ITOMPFixedTrajectoryDuration:
        return planAndExecuteFixedTrajectoryDuration();

    case ITOMPFlexibleTrajectoryDuration:
        return planAndExecuteFlexibleTrajectoryDuration();

    default:
        ROS_ERROR("Unknown ITOMP algorithm enum [%d]", options_.itomp_algorithm);
        return false;
    }
}

void ITOMPPlannerNode::initializeOptimizers()
{
    const double optimization_time_fraction = 0.80;
    const double optimization_time = options_.planning_timestep * optimization_time_fraction;
    
    double trajectory_duration = options_.trajectory_duration;

    // compute robot's root transformation from tf listener
    const Eigen::Affine3d robot_root_transform = getRobotRootTransform();

    // initialize optimizers
    optimizers_.resize(options_.num_trajectories);
    optimizer_threads_.resize(options_.num_trajectories);
    for (int i=0; i<options_.num_trajectories; i++)
    {
        ITOMPOptimizer& optimizer = optimizers_[i];

        // initialize planning scene
        optimizer.setPlanningScene(&planning_scene_);

        // initialize publishers
        optimizer.setVisualizationTopic(node_handle_, std::string("trajectory_") + std::to_string(i));

        optimizer.setRobotRootLinkTransform(robot_root_transform);
        optimizer.setUseNumericalDerivative(false);
        optimizer.setNumInterpolationSamples(options_.num_interpolation_samples);
        optimizer.setRobotModel(robot_model_);
        optimizer.setOptimizationTimeLimit(optimization_time);
        optimizer.setPlanningTimestep(options_.planning_timestep);
        optimizer.setDynamicObstacleMaxSpeed(options_.conservative_algorithm.dynamic_obstacle_max_speed);
        optimizer.setDynamicObstacleDuration(options_.conservative_algorithm.dynamic_obstacle_duration);

        // initialize trajectory only with start state
        optimizer.setPlanningRobotStartState(start_state_, trajectory_duration, options_.num_milestones);

        // initialize goal poses
        optimizer.clearGoalLinkPoses();
        for (int i=0; i<goal_link_positions_.size(); i++)
            optimizer.addGoalLinkPosition(goal_link_positions_[i].first, goal_link_positions_[i].second);
        for (int i=0; i<goal_link_orientations_.size(); i++)
            optimizer.addGoalLinkOrientation(goal_link_orientations_[i].first, goal_link_orientations_[i].second);

        // initialize cost weights
        optimizer.clearCostWeights();
        for (int i=0; i<options_.cost_weights.size(); i++)
            optimizer.setCostWeight(options_.cost_weights[i].first, options_.cost_weights[i].second);
    }

    // trajectory 0 starts with linear path. Other start with random milestones
    for (int i=1; i<optimizers_.size(); i++)
        optimizers_[i].initializeRandomMilestones();
}

bool ITOMPPlannerNode::planAndExecuteFixedTrajectoryDuration()
{
    const int states_per_second = 15;
    const int num_states_per_planning_timestep = (int)(options_.planning_timestep * states_per_second) + 1;

    double elapsed_time = 0.;
    ros::Rate rate( 1. / options_.planning_timestep );

    double trajectory_duration = options_.trajectory_duration;

    // virtual human arm request
    const double virtual_human_arm_delay = 1.5;
    const double virtual_human_arm_speed = 50.0;
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(virtual_human_arm_delay);
    msg.data.push_back(virtual_human_arm_speed);
    virtual_human_arm_request_publisher_.publish(msg);

    while (true)
    {
        ROS_INFO("Planning trajectory of %lf sec", trajectory_duration);

        // update dynamic environments

        // visualize updated planning scene
        visualizePlanningScene();

        // optimize during optimization_time
        optimizeAllTrajectories();

        // sleep until next timestep
        rate.sleep();

        // find the best trajectory
        int best_trajectory_index = 0;
        double best_trajectory_cost = optimizers_[0].cost();
        for (int i=1; i<optimizers_.size(); i++)
        {
            const double cost = optimizers_[i].trajectoryCost();

            if (cost < best_trajectory_cost)
            {
                best_trajectory_index = i;
                best_trajectory_cost = cost;
            }
        }

        // execute
        moveit_msgs::RobotTrajectory robot_trajectory_msg;
        optimizers_[best_trajectory_index].getRobotTrajectoryIntervalMsg(robot_trajectory_msg, 0, options_.planning_timestep, num_states_per_planning_timestep);
        trajectory_execution_manager_->pushAndExecute(robot_trajectory_msg);

        // TODO: record to response
        /*
        robot_trajectory::RobotTrajectory robot_trajectory(robot_model_, planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*start_state_, robot_trajectory_msg);
        res_->trajectory_->append(robot_trajectory, options_.planning_timestep);
        res_->error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        */

        // step forward one planning step
        if (trajectory_duration <= options_.planning_timestep + 1e-6)
            break;
        trajectory_duration -= options_.planning_timestep;
        elapsed_time += options_.planning_timestep;

        optimizers_[best_trajectory_index].stepForward(options_.planning_timestep);

        // copy the best trajectory to others
        for (int i=0; i<optimizers_.size(); i++)
        {
            if (i != best_trajectory_index)
                optimizers_[i].copyTrajectory( optimizers_[best_trajectory_index] );
        }
    }

    ROS_INFO("Waiting %lf sec for the last execution step", options_.planning_timestep);
    ros::Duration(options_.planning_timestep).sleep();

    return true;
}

bool ITOMPPlannerNode::planAndExecuteFlexibleTrajectoryDuration()
{
    const int states_per_second = 15;
    const int num_states_per_planning_timestep = (int)(options_.planning_timestep * states_per_second) + 1;

    double elapsed_trajectory_time = 0.;
    ros::Rate rate( 1. / options_.planning_timestep );

    double trajectory_duration = options_.trajectory_duration;

    // virtual human arm request
    const double virtual_human_arm_delay = 1.5;
    const double virtual_human_arm_speed = 50.0;
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(virtual_human_arm_delay);
    msg.data.push_back(virtual_human_arm_speed);
    virtual_human_arm_request_publisher_.publish(msg);

    ros::Time start_time = ros::Time::now();

    while (true)
    {
        const ros::Duration elapsed_ros_time = ros::Time::now() - start_time;
        ROS_INFO("Planning trajectory after %lf sec (ros time: %lf, error: %lf)", elapsed_trajectory_time, elapsed_ros_time.toSec(), elapsed_ros_time.toSec() - elapsed_trajectory_time);
        
        // update dynamic environments

        // visualize updated planning scene
        visualizePlanningScene();

        // optimize during optimization_time
        optimizeAllTrajectories();

        // find the best trajectory
        int best_trajectory_index = 0;
        double best_trajectory_cost = optimizers_[0].cost();
        for (int i=1; i<optimizers_.size(); i++)
        {
            const double cost = optimizers_[i].trajectoryCost();

            if (cost < best_trajectory_cost)
            {
                best_trajectory_index = i;
                best_trajectory_cost = cost;
            }
        }

        // DEBUG: test gradient for best trajectory
        optimizers_[best_trajectory_index].testGradients();

        // prepare the first timestep of trajectory for execution
        moveit_msgs::RobotTrajectory robot_trajectory_msg;
        optimizers_[best_trajectory_index].getRobotTrajectoryIntervalMsg(robot_trajectory_msg, 0, options_.planning_timestep, num_states_per_planning_timestep);
        
        // TODO: record to response
        /*
        robot_trajectory::RobotTrajectory robot_trajectory(robot_model_, planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*start_state_, robot_trajectory_msg);
        res_->trajectory_->append(robot_trajectory, options_.planning_timestep);
        res_->error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        */

        optimizers_[best_trajectory_index].stepForward(options_.planning_timestep);
        optimizers_[best_trajectory_index].extend(options_.planning_timestep);
        elapsed_trajectory_time += options_.planning_timestep;

        //ROS_INFO("Best trajectory cost: %lf", best_trajectory_cost);

        if (best_trajectory_cost < options_.goal_tolerance)
            break;

        /*
        if (optimizers_[best_trajectory_index].reachedGoalPose(options_.goal_tolerance))
            break;
            */

        // copy the best trajectory to others
        for (int i=0; i<optimizers_.size(); i++)
        {
            if (i != best_trajectory_index)
                optimizers_[i].copyTrajectory( optimizers_[best_trajectory_index] );
        }

        // sleep until next timestep then execute
        rate.sleep();
        trajectory_execution_manager_->pushAndExecute(robot_trajectory_msg);
    }

    ROS_INFO("Waiting %lf sec for the last execution step", options_.planning_timestep);
    ros::Duration(options_.planning_timestep).sleep();

    return true;
}

}
