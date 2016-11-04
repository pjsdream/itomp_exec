#include <itomp_exec/planner/itomp_planner.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <functional>

#include <std_msgs/Float64MultiArray.h>


namespace itomp_exec
{

ItompPlanner::ItompPlanner(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
    , robot_model_(0)
{
    trajectory_ = new ItompTrajectory();
    optimizer_ = new ItompOptimizer(trajectory_);

    SmoothnessCost* smoothness_cost = new SmoothnessCost();
    CollisionCost* collision_cost = new CollisionCost();

    cost_functions_[0] = smoothness_cost;
    cost_functions_[1] = collision_cost;

    optimizer_->setCostFunction(0, smoothness_cost);
    optimizer_->setCostFunction(1, collision_cost);
}

ItompPlanner::~ItompPlanner()
{
    for (std::map<int, Cost*>::iterator it = cost_functions_.begin(); it != cost_functions_.end(); it++)
        delete it->second;

    if (robot_model_ != 0)
        delete robot_model_;

    delete trajectory_;
    delete optimizer_;
}

void ItompPlanner::setRobotModel(const PlanningRobotModel *robot_model)
{
    if (robot_model_ != 0)
        delete robot_model_;

    robot_model_ = new PlanningRobotModel(*robot_model);

    trajectory_->setRobotModel(robot_model_);
}

void ItompPlanner::setTimestep(double timestep)
{
    trajectory_->setTimestep(timestep);
}

void ItompPlanner::setTrajectoryDuration(double duration)
{
    trajectory_->setTrajectoryDuration(duration);
}

void ItompPlanner::planForOneTimestep()
{
    optimizer_->printCostFunctions();
    optimizer_->optimize(0.8 * timestep_);
}

void ItompPlanner::enableVisualizeTrajectoryEachStep()
{
    optimizer_visualize_trajectory_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("optimizer/trajectory", 1);
    optimizer_->enableVisualizeTrajectoryEachStep(&optimizer_visualize_trajectory_publisher_);

    // wait for initializing publisher
    ros::Duration(0.5).sleep();
}

void ItompPlanner::disableVisualizeTrajectoryEachStep()
{
    optimizer_->disableVisualizeTrajectoryEachStep();
}

}
