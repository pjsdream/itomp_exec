#ifndef ITOMP_EXEC_ITOMP_PLANNER_H
#define ITOMP_EXEC_ITOMP_PLANNER_H


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <itomp_exec/scene/planning_scene.h>
#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/robot/planning_robot_model.h>
#include <itomp_exec/trajectory/itomp_trajectory.h>
#include <itomp_exec/cost/cost_functions.h>
#include <itomp_exec/scene/planning_scene.h>

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

    ItompPlanner(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ~ItompPlanner();

    void setRobotModel(const RobotModel* robot_model, const std::string& planning_group);
    void setRobotModel(const PlanningRobotModel* robot_model);
    void setRobotStartState(const PlanningRobotState& robot_start_state);
    void setRobotBaseTransform(const Eigen::Affine3d& robot_base_transform);
    void setPlanningScene(const PlanningScene* planning_scene);
    void setTimestep(double timestep);
    void setTrajectoryDuration(double duration);
    void setNumWaypoints(int num_waypoints);
    void setCostFunction(int id, Cost* cost);

    void enableVisualizeTrajectoryEachStep();
    void disableVisualizeTrajectoryEachStep();

    void planForOneTimestep();

    void printCostFunctions();

    void visualizePlanningScene();
    void visualizeRobotBoundingSpheres(const PlanningRobotState& robot_state, const std::string& ns);

private:

    ros::NodeHandle node_handle_;

    const PlanningRobotModel* robot_model_;
    const PlanningScene* planning_scene_;
    Eigen::Affine3d robot_base_transform_;

    double timestep_;
    double trajectory_duration_;

    ItompOptimizer* optimizer_;
    ItompTrajectory* trajectory_;
    Thread* optimizer_thread_;

    std::map<int, Cost*> cost_functions_;

    // visualization
    void initializePublishers();
    ros::Publisher visualize_publisher_;
    ros::Publisher optimizer_visualize_trajectory_publisher_; // thread-safe publisher
};

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_H
