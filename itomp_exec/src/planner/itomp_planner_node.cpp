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

    // initialize planning scene visualization publisher
    planning_scene_visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("planning_scene", 1);

    goal_constraint_visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("goal_constraint", 1);
    trajectory_visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("trajectory", 1);

    // initialize robot model from moveit model
    robot_model_.reset( new BoundingSphereRobotModel );
    robot_model_->initFromMoveitRobotModel(moveit_robot_model_);
            
    // initialize trajectory_execution_manager with robot model
    // see how it is initialized with ros params at http://docs.ros.org/indigo/api/moveit_ros_planning/html/trajectory__execution__manager_8cpp_source.html#l00074
    std::string controller;
    if (node_handle_.getParam("moveit_controller_manager", controller))
    {
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(moveit_robot_model_, true));
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
    node_handle_.param("goal_tolerance", options_.goal_tolerance, 0.03);

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
        const double weight = req.goal_constraints[0].position_constraints[i].weight;
        goal_link_positions_.push_back(GoalPositionConstraint(name, position, weight));
    }
    
    goal_link_orientations_.clear();
    for (int i=0; i<req.goal_constraints[0].orientation_constraints.size(); i++)
    {
        const std::string& name = req.goal_constraints[0].orientation_constraints[i].link_name;
        Eigen::Quaterniond orientation;
        tf::quaternionMsgToEigen(req.goal_constraints[0].orientation_constraints[i].orientation, orientation);
        const double weight = req.goal_constraints[0].orientation_constraints[i].weight;
        goal_link_orientations_.push_back(GoalOrientationConstraint(name, orientation, weight));
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

void ITOMPPlannerNode::visualizeGoalConstraints()
{
    const double radius = 0.05;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_model_->getFrameId();
    marker.header.stamp = ros::Time::now();

    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "goal_positions";

    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;

    marker.scale.x = radius * 2.;
    marker.scale.y = radius * 2.;
    marker.scale.z = radius * 2.;

    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;

    marker.type = visualization_msgs::Marker::SPHERE;

    for (int i=0; i<goal_link_positions_.size(); i++)
    {
        const std::string& link_name = goal_link_positions_[i].link_name;
        const Eigen::Vector3d& target_position = goal_link_positions_[i].position;

        tf::pointEigenToMsg(target_position, marker.pose.position);

        marker.id = i;

        marker_array.markers.push_back(marker);
    }

    goal_constraint_visualization_publisher_.publish(marker_array);
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
    {
        optimizer_threads_[i] = new Thread(&optimizeSingleTrajectory, &optimizers_[i]);
    }

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

bool ITOMPPlannerNode::planAndExecute(moveit_msgs::RobotTrajectory& robot_trajectory)
{
    response_robot_trajectory_ = &robot_trajectory;
    response_robot_trajectory_->joint_trajectory.points.clear();

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
            optimizer.addGoalLinkPosition(goal_link_positions_[i].link_name, goal_link_positions_[i].position, goal_link_positions_[i].weight);
        for (int i=0; i<goal_link_orientations_.size(); i++)
            optimizer.addGoalLinkOrientation(goal_link_orientations_[i].link_name, goal_link_orientations_[i].orientation, goal_link_orientations_[i].weight);

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
    const int states_per_second = 10;
    const int num_states_per_planning_timestep = (int)(options_.planning_timestep * states_per_second) + 1;

    double elapsed_trajectory_time = 0.;
    ros::Rate rate( 1. / options_.planning_timestep );

    ros::Time start_time = ros::Time::now();

    planning_scene_.enable();
    while (true)
    {
        const ros::Duration elapsed_ros_time = ros::Time::now() - start_time;
        ROS_INFO("Planning trajectory after %lf sec (ros time: %lf, error: %lf)", elapsed_trajectory_time, elapsed_ros_time.toSec(), elapsed_ros_time.toSec() - elapsed_trajectory_time);

        // update dynamic environments

        // visualize updated planning scene
        visualizePlanningScene();

        // visualize goal constraint (the base link may be changed)
        visualizeGoalConstraints();

        // optimize during optimization_time
        optimizeAllTrajectories();

        if (elapsed_trajectory_time >= 5. || elapsed_trajectory_time <= 1.)
            planning_scene_.disable();
        else
            planning_scene_.enable();

        // find the best trajectory
        int best_trajectory_index = -1;
        double best_trajectory_cost = 0.;
        for (int i=0; i<optimizers_.size(); i++)
        {
            const double cost = optimizers_[i].trajectoryCost();

            if (best_trajectory_index == -1 || cost < best_trajectory_cost)
            {
                best_trajectory_index = i;
                best_trajectory_cost = cost;
            }
        }

        // DEBUG: test gradient for best trajectory
        //optimizers_[best_trajectory_index].testGradients();

        // prepare the first timestep of trajectory for execution
        moveit_msgs::RobotTrajectory robot_trajectory_msg;
        optimizers_[best_trajectory_index].getRobotTrajectoryIntervalMsg(robot_trajectory_msg, 0, options_.planning_timestep, num_states_per_planning_timestep);

        // TODO: record to response
        /*
        robot_trajectory::RobotTrajectory robot_trajectory(moveit_robot_model_, planning_group_name_);
        robot_trajectory.setRobotTrajectoryMsg(*start_state_, robot_trajectory_msg);
        res_->trajectory_->append(robot_trajectory, options_.planning_timestep);
        res_->error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        */

        // response
        response_robot_trajectory_->joint_trajectory.joint_names = robot_trajectory_msg.joint_trajectory.joint_names;
        response_robot_trajectory_->joint_trajectory.points.insert(
                    response_robot_trajectory_->joint_trajectory.points.end(),
                    robot_trajectory_msg.joint_trajectory.points.begin(),
                    robot_trajectory_msg.joint_trajectory.points.end()
                    );

        optimizers_[best_trajectory_index].stepForward(options_.planning_timestep);
        optimizers_[best_trajectory_index].extend(options_.planning_timestep);
        elapsed_trajectory_time += options_.planning_timestep;

        //ROS_INFO("Best trajectory cost: %lf", best_trajectory_cost);

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

        // initialize with random milestones except one
        for (int i=1; i<optimizers_.size(); i++)
        {
            optimizers_[i].initializeRandomMilestones();
        }

        // visualize endeffector trajectory
        visualizeTrajectory("ee_link", 10000);

        // sleep until next timestep then execute
        rate.sleep();

        robot_trajectory_msg.joint_trajectory.header.stamp = ros::Time::now();
        trajectory_execution_manager_->pushAndExecute(robot_trajectory_msg);

        if (best_trajectory_cost < options_.goal_tolerance)
            break;
<<<<<<< Updated upstream
=======

        // print best trajectory cost
        ROS_INFO("best_trajectory_cost: %lf  tolerance: %lf", best_trajectory_cost, options_.goal_tolerance);
        //optimizers_[best_trajectory_index].printCosts();
>>>>>>> Stashed changes
    }

    ROS_INFO("Waiting %lf sec for the last execution step", options_.planning_timestep);
    ros::Duration(options_.planning_timestep).sleep();

    return true;
}

void ITOMPPlannerNode::visualizeTrajectory(const std::string& endeffector_link_name, int step)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_model_->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;

    for (int i=0; i<marker_array_ns_id_.size(); i++)
    {
        marker.ns = marker_array_ns_id_[i].first;
        marker.id = marker_array_ns_id_[i].second;
        marker_array.markers.push_back(marker);
    }

    trajectory_visualization_publisher_.publish(marker_array);

    marker_array_ns_id_.clear();
    marker_array.markers.clear();

    // add robots
    marker.ns = "endeffector";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.03;

    std_msgs::ColorRGBA color0;
    color0.r = 1.0;
    color0.g = 0.5;
    color0.b = 0.0;
    color0.a = 1.0;

    std_msgs::ColorRGBA color1;
    color1.r = 0.0;
    color1.g = 0.0;
    color1.b = 1.0;
    color1.a = 1.0;

    const std::vector<std::string>& joint_names = response_robot_trajectory_->joint_trajectory.joint_names;
    const int endeffector_index = robot_model_->getJointIndexByLinkName(endeffector_link_name);

    geometry_msgs::Point previous_point;

    double ee_length = 0.;

    for (int i=0; i<response_robot_trajectory_->joint_trajectory.points.size(); i++)
    {
        const double t = (double)i / (response_robot_trajectory_->joint_trajectory.points.size() - 1);

        const std::vector<double>& positions = response_robot_trajectory_->joint_trajectory.points[i].positions;

        Eigen::VectorXd joint_positions = start_state_.getDefaultJointPositions();

        for (int j=0; j<joint_names.size(); j++)
            joint_positions( robot_model_->getJointIndexByName(joint_names[j]) ) = positions[j];

        std::vector<Eigen::Affine3d> transforms;

        robot_model_->getLinkTransforms(joint_positions, transforms);

        if (i % step == 0 && step < 1000)
        {
            const int last_index = marker_array.markers.size();
            robot_model_->pushVisualLinkVisualizationMarkers(transforms, "trajectory_" + std::to_string(i), marker_array);

            for (int j=last_index; j<marker_array.markers.size(); j++)
            {
                marker_array.markers[j].color.r = 0.5;
                marker_array.markers[j].color.g = 0.5;
                marker_array.markers[j].color.b = 0.5;
                marker_array.markers[j].color.a = 0.5;
            }
        }

        // endeffector trajectory
        geometry_msgs::Point point;
        tf::pointEigenToMsg(transforms[endeffector_index].translation(), point);

        std_msgs::ColorRGBA color;
        color.r = (1. - t) * color0.r + t * color1.r;
        color.g = (1. - t) * color0.g + t * color1.g;
        color.b = (1. - t) * color0.b + t * color1.b;
        color.a = (1. - t) * color0.a + t * color1.a;

        marker.points.push_back(point);
        marker.colors.push_back(color);

        if (i)
            ee_length += (Eigen::Vector3d(point.x, point.y, point.z) - Eigen::Vector3d(previous_point.x, previous_point.y, previous_point.z)).norm();

        previous_point = point;
    }

    ROS_INFO("ee length: %lf", ee_length);
    ROS_INFO("minimum distance to human: %lf", getMinimumDistanceToHuman());

    marker_array.markers.push_back(marker);

    trajectory_visualization_publisher_.publish(marker_array);

    for (int i=0; i<marker_array.markers.size(); i++)
        marker_array_ns_id_.push_back( std::make_pair(marker_array.markers[i].ns, marker_array.markers[i].id) );
}

double ITOMPPlannerNode::getMinimumDistanceToHuman()
{
    ITOMPOptimizer& optimizer = optimizers_[0];

    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_robot_joints = optimizer.getNumRobotJoints();
    const Spheres& dynamic_obstacle_spheres = optimizer.getDynamicObstacleSpheres();

    double result = 100.;

    for (int i=0; i<num_interpolated_variables; i++)
    {
        for (int j=0; j<num_robot_joints; j++)
        {
            const Spheres& robot_spheres = optimizer.getInterpolatedLinkCollisionSpheres(i, j);

            for (int k=0; k<robot_spheres.size(); k++)
            {
                const Sphere& robot_sphere = robot_spheres[k];

                for (int l=0; l<dynamic_obstacle_spheres.size(); l++)
                {
                    const Sphere& obstacle_sphere = dynamic_obstacle_spheres[l];

                    const double r = robot_sphere.radius + obstacle_sphere.radius;
                    const double d_squared = (robot_sphere.position - obstacle_sphere.position).squaredNorm();

                    result = std::min(result, std::sqrt(d_squared) - r);
                }
            }
        }
    }

    return result;
}

void ITOMPPlannerNode::clearTrajectoryVisualization()
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_model_->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;

    for (int i=0; i<marker_array_ns_id_.size(); i++)
    {
        marker.ns = marker_array_ns_id_[i].first;
        marker.id = marker_array_ns_id_[i].second;
        marker_array.markers.push_back(marker);
    }

    trajectory_visualization_publisher_.publish(marker_array);

    marker_array_ns_id_.clear();
    marker_array.markers.clear();
}

void ITOMPPlannerNode::measureCostComputationTime()
{
    const int num_evaluations = 1000000;

    ros::Time start_time = ros::Time::now();
    ITOMPOptimizer& optimizer = optimizers_[0];
    CollisionCost* cost_function = optimizer.getCollisionCostFunction();
    for (int i=0; i<num_evaluations; i++)
        cost_function->addCost();

    double one_evaluation_time = (ros::Time::now() - start_time).toSec() / num_evaluations / options_.num_interpolation_samples / options_.num_milestones;

    ROS_INFO("one evaluation time: %lf ms", one_evaluation_time * 1000. * 112 * robot_model_->getNumBoundingSpheres());
}

}
