#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>


// DEBUG
#include <itomp_exec/cost/collision_cost.h>


namespace itomp_exec
{

ITOMPPlannerNode::ITOMPPlannerNode(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle)
    : moveit_robot_model_(robot_model)
    , node_handle_(node_handle)
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
    // initialize publishers
    optimizer_.setVisualizationTopic(node_handle_, "trajectory");
    ROS_INFO("Sleep 0.5 sec for initializing publisher");
    ros::Duration(0.5).sleep();

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
}

void ITOMPPlannerNode::loadParams()
{
    // load options
    node_handle_.param("trajectory_duration", options_.trajectory_duration, 5.0);
    node_handle_.param("num_milestones", options_.num_milestones, 10);
    node_handle_.param("num_interpolation_samples", options_.num_interpolation_samples, 10);
    node_handle_.param("planning_timestep", options_.planning_timestep, 0.5);
    
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

                    // add to optimizer planning scene
                    optimizer_.addStaticObstalceSphere(radius, p);
                }
            }
            else
            {
                ROS_ERROR("ITOMP static_obstacle parameter does not have 'type' field.");
                continue;
            }
        }

        planning_scene_.setVisualizationTopic("planning_scene");
    }
    
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

bool ITOMPPlannerNode::planAndExecute(planning_interface::MotionPlanResponse& res)
{
    // visualize static obstacles before planning
    planning_scene_.visualizeScene();

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(moveit_robot_model_, planning_group_name_));

    const double optimization_time_fraction = 0.90;
    const double optimization_time = options_.planning_timestep * optimization_time_fraction;
    const int states_per_second = 15;
    const int num_states_per_planning_timestep = (int)(options_.planning_timestep * states_per_second) + 1;
    
    double trajectory_duration = options_.trajectory_duration;

    ros::WallRate rate( 1. / options_.planning_timestep );

    // initialize optimizer
    optimizer_.setUseNumericalDerivative(false);
    optimizer_.setNumInterpolationSamples(options_.num_interpolation_samples);
    optimizer_.setRobotModel(robot_model_);
    optimizer_.setPlanningRobotStartState(start_state_, trajectory_duration, options_.num_milestones);
    optimizer_.setOptimizationTimeLimit(optimization_time);
    
    // initialize goal poses
    for (int i=0; i<goal_link_positions_.size(); i++)
        optimizer_.addGoalLinkPosition(goal_link_positions_[i].first, goal_link_positions_[i].second);
    for (int i=0; i<goal_link_orientations_.size(); i++)
        optimizer_.addGoalLinkOrientation(goal_link_orientations_[i].first, goal_link_orientations_[i].second);
    
    // initialize cost weights
    for (int i=0; i<options_.cost_weights.size(); i++)
        optimizer_.setCostWeight(options_.cost_weights[i].first, options_.cost_weights[i].second);

    while (true)
    {
        ROS_INFO("Planning trajectory of %lf sec", trajectory_duration);
        
        // update dynamic environments
        // TODO: timeout, topic name

        // optimize during optimization_time
        optimizer_.optimize();

        // TODO: execute
        moveit_msgs::RobotTrajectory robot_trajectory_msg;
        optimizer_.getRobotTrajectoryIntervalMsg(robot_trajectory_msg, 0, options_.planning_timestep, num_states_per_planning_timestep);
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

        optimizer_.stepForward(options_.planning_timestep);

        // sleep until next optimization
        rate.sleep();
    }
    
    ros::Duration(options_.planning_timestep).sleep();

    return true;
}

}
