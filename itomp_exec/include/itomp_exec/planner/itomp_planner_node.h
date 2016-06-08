#ifndef ITOMP_EXEC_ITOMP_PLANNER_NODE_H
#define ITOMP_EXEC_ITOMP_PLANNER_NODE_H


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <itomp_exec/scene/planning_scene.h>
#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/robot/robot_model.h>
#include <itomp_exec/robot/robot_state.h>

#include <pcml/FutureObstacleDistributions.h>

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
        double planning_timestep;
        
        std::vector<std::pair<std::string, double> > cost_weights;
        std::vector<std::string> collision_sphere_link_names;
    };
    
public:

    ITOMPPlannerNode(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ITOMPPlannerNode(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));

    inline const PlanningScene& getPlaningScene() const
    {
        return planning_scene_;
    }

    inline robot_model::RobotModelConstPtr getRobotModel() const
    {
        return moveit_robot_model_;
    }

    inline double getTrajectoryDuration() const
    {
        return options_.trajectory_duration;
    }
    
    inline double getPlanningTimestep() const
    {
        return options_.planning_timestep;
    }
    
    inline void setTrajectoryDuration(double trajectory_duration)
    {
        options_.trajectory_duration = trajectory_duration;
    }
    
    inline void setPlanningTimestep(double planning_timestep)
    {
        options_.planning_timestep = planning_timestep;
    }

    void setRobotModel(const robot_model::RobotModelConstPtr& robot_model);

    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation = Eigen::Affine3d::Identity());
    void addStaticObstacles(const std::vector<std::string>& mesh_filename, const std::vector<Eigen::Affine3d>& transformation);
    
    void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);

    bool planAndExecute(planning_interface::MotionPlanResponse& res);
    
    // DEBUG
    void printParams();
    void printControllers();
    void printCostWeights();

private:
    
    // initialization functions
    void initialize();
    void loadParams();
    void loadStaticObstacles();
    
    // ros & moveit stuffs
    ros::NodeHandle node_handle_;
    robot_model::RobotModelConstPtr moveit_robot_model_;
    trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

    RobotModelPtr robot_model_;
    RobotState start_state_;
    ITOMPOptimizer optimizer_;

    // planning environment
    std::string planning_group_name_;
    PlanningScene planning_scene_;

    // ITOMP options
    ITOMPPlannerOptions options_;
};

typedef std::shared_ptr<ITOMPPlannerNode> ITOMPPlannerNodePtr;
typedef std::shared_ptr<const ITOMPPlannerNode> ITOMPPlannerNodeConstPtr;

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_NODE_H
