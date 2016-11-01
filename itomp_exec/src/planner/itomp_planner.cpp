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
}

ItompPlanner::ItompPlanner(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_.initFromMoveitRobotModel( robot_model_loader.getModel() );

    initializeTrajectory();
}

void ItompPlanner::setTimestep(double timestep)
{
    timestep_ = timestep;
}

void ItompPlanner::setTrajectoryDuration(double duration)
{
    trajectory_duration_ = duration;
}

void ItompPlanner::initializeTrajectory()
{
    trajectory_ = new ItompTrajectory(&robot_model_);
}

void ItompPlanner::planForOneTimestep()
{
}

}
