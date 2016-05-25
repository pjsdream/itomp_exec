#ifndef ITOMP_EXEC_ITOMP_PLANNER_NODE_H
#define ITOMP_EXEC_ITOMP_PLANNER_NODE_H


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <ros/ros.h>


namespace itomp_exec
{

// planning_interface::PlannerManager is not appropriate for the purpose of replanning.
// ITOMP provides 
class ITOMPPlannerNode
{
public:
    
    struct ITOMPPlannerOptions
    {
        double trajectory_duration;
        int num_milestones;
        int num_interpolation_samples;
    };
    
public:
    
    ITOMPPlannerNode(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    
    void loadParams();
    
    void setRobotModel(const robot_model::RobotModelConstPtr& model);
    void setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);
    void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);
    
    bool plan(planning_interface::MotionPlanResponse& res);
    bool planAndExecute(planning_interface::MotionPlanResponse& res);
    
    // DEBUG
    void printParams();

private:
    
    void recursePlan();
    
    ros::NodeHandle node_handle_;
    
    robot_model::RobotModelConstPtr robot_model_;
    planning_scene::PlanningSceneConstPtr planning_scene_;
    planning_interface::MotionPlanRequest req_;
    ITOMPPlannerOptions options_;
    
    trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

    // result trajectory after plan
    planning_interface::MotionPlanResponse* res_;
    
    // auxiliary variable of whether plan is executed while planning or not
    bool execution_while_planning_;
};

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_NODE_H
