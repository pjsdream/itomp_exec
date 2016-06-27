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


namespace itomp_exec
{

// planning_interface::PlannerManager is not appropriate for the purpose of replanning.
// ITOMP provides 
class ITOMPPlannerNode
{
public:

    enum ITOMPAlgorithm
    {
        ITOMPFixedTrajectoryDuration = 0,
        ITOMPFlexibleTrajectoryDuration,
    };

    struct ITOMPPlannerOptions
    {
        int num_trajectories;
        double trajectory_duration;
        int num_milestones;
        int num_interpolation_samples;
        double planning_timestep;

        ITOMPAlgorithm itomp_algorithm;
        
        std::vector<std::pair<std::string, double> > cost_weights;
        std::vector<std::string> collision_sphere_link_names;

        // ITOMP dynamic obstacle conservative bound
        struct
        {
            double dynamic_obstacle_max_speed;
            double dynamic_obstacle_duration;
        } conservative;
    };
    
public:

    ITOMPPlannerNode(robot_model::RobotModelConstPtr robot_model, const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ITOMPPlannerNode(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));

    inline const PlanningScene& getPlanningScene() const
    {
        return planning_scene_;
    }

    inline robot_model::RobotModelConstPtr getMoveitRobotModel() const
    {
        return moveit_robot_model_;
    }

    inline BoundingSphereRobotModelPtr getRobotModel() const
    {
        return robot_model_;
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

    void optimizeAllTrajectories();

    Eigen::Affine3d getRobotRootTransform();

    tf::TransformListener transform_listener_;
    
    // ros & moveit stuffs
    ros::NodeHandle node_handle_;
    robot_model::RobotModelConstPtr moveit_robot_model_;
    trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

    BoundingSphereRobotModelPtr robot_model_;
    RobotState start_state_;

    // multithreaded optimizers
    std::vector<ITOMPOptimizer> optimizers_;
    std::vector<pthread_t> optimizer_threads_;
    
    std::vector<std::pair<std::string, Eigen::Vector3d> > goal_link_positions_;
    std::vector<std::pair<std::string, Eigen::Quaterniond> > goal_link_orientations_;
    
    // planning environment
    std::string planning_group_name_;
    PlanningScene planning_scene_; //!< spheres should be added to optimizer independantly

    // ITOMP options
    ITOMPPlannerOptions options_;

    // virtual human
    ros::Publisher virtual_human_arm_request_publisher_;
};

typedef std::shared_ptr<ITOMPPlannerNode> ITOMPPlannerNodePtr;
typedef std::shared_ptr<const ITOMPPlannerNode> ITOMPPlannerNodeConstPtr;

}


#endif // ITOMP_EXEC_ITOMP_PLANNER_NODE_H
