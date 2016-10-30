#ifndef ITOMP_EXEC_ITOMP_PLANNER_NODE_H
#define ITOMP_EXEC_ITOMP_PLANNER_NODE_H


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <itomp_exec/scene/planning_scene.h>
#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/robot/bounding_sphere_robot_model.h>
#include <itomp_exec/robot/robot_state.h>

#include <pcml/FutureObstacleDistributions.h>

#include <tf/transform_listener.h>

#include <ros/ros.h>

// pthread wrapper
#include <itomp_exec/util/thread.h>


namespace itomp_exec
{

// The MoveIt class "planning_interface::PlannerManager" is not appropriate for the purpose of replanning.
// Thus, a new class for replanning is defined.
class ItompPlanner
{
public:

    ItompPlanner(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ItompPlanner(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));

    void setTimestep(double timestep);
    void setTrajectoryDuration(double duration);

    void planForOneTimestep();


private:

    ros::NodeHandle node_handle_;

    RobotModel robot_model_;

    double timestep_;
    double trajectory_duration_;
};

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_NODE_H
