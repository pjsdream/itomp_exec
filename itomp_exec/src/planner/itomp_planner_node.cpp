#include <itomp_exec/planner/itomp_planner_node.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>


// DEBUG
#include <itomp_exec/cost/collision_cost.h>


namespace itomp_exec
{

ITOMPPlannerNode::ITOMPPlannerNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    initialize();
}

void ITOMPPlannerNode::initialize()
{
    // publishers
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    
    ros::Duration sleep_time(1.0);
    ROS_INFO("Waiting %.2lf sec due to publisher advertising delay", sleep_time.toSec());
    sleep_time.sleep();
    
    // load robot model from robot_description
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    
    // initialize planning scene
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    
    // initialize trajectory_execution_manager with robot model
    // see how it is initialized with ros params at http://docs.ros.org/indigo/api/moveit_ros_planning/html/trajectory__execution__manager_8cpp_source.html#l00074
    std::string controller;
    if (node_handle_.getParam("moveit_controller_manager", controller))
    {
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(robot_model_));
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
        
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(robot_model_));
        
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
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(robot_model_, true));
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
    node_handle_.param("num_trajectories", options_.num_trajectories, 8);
    node_handle_.param("planning_timestep", options_.planning_timestep, 0.5);
    
    // load static obstacles
    XmlRpc::XmlRpcValue static_obstacles;
    if (node_handle_.getParam("static_obstacles", static_obstacles))
    {
        std::vector<std::string> filenames;
        std::vector<Eigen::Affine3d> transformations;
        
        for (int i=0; i<static_obstacles.size(); i++)
        {
            XmlRpc::XmlRpcValue& static_obstacle = static_obstacles[i];
            
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
                
                filenames.push_back(filename);
                transformations.push_back(transformation);
            }
            else
            {
                ROS_ERROR("ITOMP static_obstacle parameter does not have 'file' field.");
                continue;
            }
        }
        
        addStaticObstacles(filenames, transformations);
    }
    
    // load cost weights
    options_.cost_weights.clear();
    XmlRpc::XmlRpcValue cost_weights;
    if (node_handle_.getParam("cost_weights", cost_weights))
    {
        for (int i=0; i<cost_weights.size(); i++)
        {
            XmlRpc::XmlRpcValue& cost_weight = cost_weights[i];
            
            for (XmlRpc::XmlRpcValue::iterator it=cost_weight.begin(); it != cost_weight.end(); it++)
            {
                const double weight = static_cast<double>(it->second);
                options_.cost_weights.push_back(std::make_pair(it->first, weight));
            }
        }
    }
}

void ITOMPPlannerNode::printParams()
{
    ROS_INFO("ITOMP-exec parameters:");
    
    ROS_INFO(" * trajectory_duration: %lf", options_.trajectory_duration);
    ROS_INFO(" * num_milestones: %d", options_.num_milestones);
    ROS_INFO(" * num_interpolation_samples: %d", options_.num_interpolation_samples);
    ROS_INFO(" * num_trajectories: %d", options_.num_trajectories);
    ROS_INFO(" * planning_timestep: %lf", options_.planning_timestep);
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
    std::vector<std::string> mesh_filenames;
    mesh_filenames.push_back(mesh_filename);
    
    std::vector<Eigen::Affine3d> transformations;
    transformations.push_back(transformation);
    
    addStaticObstacles(mesh_filenames, transformations);
}

void ITOMPPlannerNode::addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations)
{    
    // collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = robot_model_->getModelFrame();
    collision_object.header.stamp = ros::Time::now();
    collision_object.id = "environment";
    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    
    const Eigen::Vector3d scale(1., 1., 1.);
    
    for (int i=0; i<mesh_filenames.size(); i++)
    {
        // mesh generation
        shapes::Mesh* shape = shapes::createMeshFromResource(mesh_filenames[i], scale);
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        delete shape;
        
        // pose computation
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(transformations[i], pose);
        
        // collision object
        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);
    }
    
    // planning scene msg
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_msg.is_diff = true;
    planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
    
    // publish planning scene msg to see in RViz
    planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void ITOMPPlannerNode::setMotionPlanRequest(const planning_interface::MotionPlanRequest& req)
{
}

bool ITOMPPlannerNode::plan(planning_interface::MotionPlanResponse& res)
{
    res_ = &res;
    execution_while_planning_ = false;
    return planAndExecute();
}

bool ITOMPPlannerNode::planAndExecute(planning_interface::MotionPlanResponse& res)
{
    res_ = &res;
    execution_while_planning_ = true;
    return planAndExecute();
}

void* ITOMPPlannerNode::optimizerThreadStartRoutine(void* arg)
{
    ROS_INFO("Optimization thread created");
    
    ITOMPOptimizer* optimizer = (ITOMPOptimizer*)arg;
    optimizer->optimize();
    
    return 0;
}

bool ITOMPPlannerNode::planAndExecute()
{
    const double optimization_time_fraction = 0.90;
    const double optimization_time = options_.planning_timestep * optimization_time_fraction;
    
    double trajectory_duration = options_.trajectory_duration;
    
    ros::WallDuration optimization_sleep_time(optimization_time);
    ros::WallRate rate( 1. / options_.planning_timestep );
    
    // initialize optimizers and threads    
    trajectories_.resize(options_.num_trajectories);
    for (int i=0; i<trajectories_.size(); i++)
        trajectories_[i].reset(new Trajectory());
    
    optimizers_.resize(trajectories_.size());
    for (int i=0; i<optimizers_.size(); i++)
    {
        optimizers_[i] = new ITOMPOptimizer(trajectories_[i]);
        optimizers_[i]->setOptimizationTimeLimit(optimization_time);
        optimizers_[i]->generateCostFunctions(options_.cost_weights);
    }
    
    threads_.resize(optimizers_.size());
    
    rate.reset();
    while (true)
    {
        ROS_INFO("Planning trajectory of %lf sec", trajectory_duration);
        
        // TODO: update dynamic environments
        
        // threading optimizations
        for (int i=0; i<optimizers_.size(); i++)
        {
            if (pthread_create(&threads_[i], NULL, &optimizerThreadStartRoutine, optimizers_[i]) != 0)
                ROS_ERROR("Error occurred creating optimizer thread %d");
        }
        
        optimization_sleep_time.sleep();
        
        for (int i=0; i<optimizers_.size(); i++)
        {
            if (pthread_join(threads_[i], NULL) != 0)
            {
                ROS_ERROR("Error occurred joining optimizer thread %d");
            }
        }
        
        // TODO: execute and update trajectories
        
        // step forward one planning step
        if (trajectory_duration <= options_.planning_timestep)
            break;
        
        trajectory_duration -= options_.planning_timestep;
        
        // sleep until next optimization
        rate.sleep();
    }
    
    clearTrajectories();
    clearOptimizers();
    
    return true;
}

void ITOMPPlannerNode::clearOptimizers()
{
    for (int i=0; i<optimizers_.size(); i++)
        delete optimizers_[i];
    optimizers_.clear();
}

void ITOMPPlannerNode::clearTrajectories()
{
    for (int i=0; i<trajectories_.size(); i++)
        trajectories_[i].reset();
    trajectories_.clear();
}

}
