#include <itomp_exec/planner/itomp_planner.h>
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

ItompPlanner::ItompPlanner(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    robot_model_.initFromMoveitRobotModel(robot_model);

    initializeTrajectory();
    initializeOptimizer();
}

ItompPlanner::ItompPlanner(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_.initFromMoveitRobotModel( robot_model_loader.getModel() );

    initializeTrajectory();
    initializeOptimizer();
}

ItompPlanner::~ItompPlanner()
{
    for (std::map<int, Cost*>::iterator it = cost_functions_.begin(); it != cost_functions_.end(); it++)
        delete it->second;
}

void ItompPlanner::setTimestep(double timestep)
{
    trajectory_->setTimestep(timestep);
}

void ItompPlanner::setTrajectoryDuration(double duration)
{
    trajectory_->setTrajectoryDuration(duration);
}

void ItompPlanner::initializeTrajectory()
{
    trajectory_ = new ItompTrajectory(&robot_model_);
}

void ItompPlanner::initializeOptimizer()
{
    optimizer_ = new ItompOptimizer(trajectory_);

    SmoothnessCost* smoothness_cost = new SmoothnessCost();
    CollisionCost* collision_cost = new CollisionCost();

    cost_functions_[0] = smoothness_cost;
    cost_functions_[1] = collision_cost;

    optimizer_->setCostFunction(0, smoothness_cost);
    optimizer_->setCostFunction(1, collision_cost);
}

void ItompPlanner::planForOneTimestep()
{
    optimizer_->printCostFunctions();
    optimizer_->optimize(0.8 * timestep_);
}

}
