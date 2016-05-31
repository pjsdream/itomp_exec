#ifndef ITOMP_EXEC_ITOMP_PLANNER_NODE_H
#define ITOMP_EXEC_ITOMP_PLANNER_NODE_H


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/cost/cost.h>
#include <pthread.h>

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
        int num_trajectories;
        double planning_timestep;
        
        std::vector<std::pair<std::string, double> > cost_weights;
    };
    
public:
    
    ITOMPPlannerNode(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    
    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation = Eigen::Affine3d::Identity());
    void addStaticObstacles(const std::vector<std::string>& mesh_filename, const std::vector<Eigen::Affine3d>& transformation);
    
    void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);
    
    std::vector<std::pair<std::string, Eigen::Vector3d> > getGoalLinkPositions() const;
    std::vector<std::pair<std::string, Eigen::Quaterniond> > getGoalLinkOrientations() const;
    const Trajectory& getTrajectoryTemplate() const;
    
    bool plan(planning_interface::MotionPlanResponse& res);
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
    void clearTrajectories();
    void clearOptimizers();
    
    // plan
    bool planAndExecute();
    static void* optimizerThreadStartRoutine(void* arg);
    
    // optimizer
    std::vector<Cost*> cost_types_;
    std::vector<TrajectoryPtr> trajectories_;
    std::vector<ITOMPOptimizer*> optimizers_;
    std::vector<pthread_t> threads_;
    
    // ros stuffs
    ros::NodeHandle node_handle_;
    ros::Publisher planning_scene_diff_publisher_;
    
    // moveit stuffs
    robot_model::RobotModelConstPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
    
    // planning requests
    std::string planning_group_name_;
    robot_state::RobotState* start_state_;
    std::vector<std::pair<std::string, Eigen::Vector3d> > goal_link_positions_;
    std::vector<std::pair<std::string, Eigen::Quaterniond> > goal_link_orientations_;
    
    // ITOMP options
    ITOMPPlannerOptions options_;

    planning_interface::MotionPlanResponse* res_; //!< to be saved the result function after plan function
    
    // auxiliary variable of whether plan is executed while planning or not
    bool execution_while_planning_;
};

typedef boost::shared_ptr<ITOMPPlannerNode> ITOMPPlannerNodePtr;
typedef boost::shared_ptr<const ITOMPPlannerNode> ITOMPPlannerNodeConstPtr;

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_NODE_H
