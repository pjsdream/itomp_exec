#ifndef ITOMP_EXEC_GOAL_POSE_COST_H
#define ITOMP_EXEC_GOAL_POSE_COST_H


#include <itomp_exec/cost/cost.h>
#include <moveit/robot_state/robot_state.h>


namespace itomp_exec
{

class GoalPoseCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(GoalPose)
    
private:

    static const double ratio_cosine_to_meter_;
    static const double ratio_radian_per_sec_to_meter_;
    
public:
        
    GoalPoseCost(double weight = 1.0);
    
    virtual void initialize(const ITOMPPlannerNode& planner_node);
    
    virtual double cost(const Trajectory& trajectory);
    virtual TrajectoryDerivative derivative(const Trajectory& trajectory);
    
private:
    
    template<typename T>
    void initializeJointsAffectingGoalLinkPoses(const Trajectory& trajectory_template, std::vector<std::vector<char> >& joints_affecting_goal_link_poses, const T& goal_links);
    
    robot_model::RobotModelConstPtr robot_model_;
    int num_planning_group_joints_;
    std::vector<std::string> planning_group_joint_names_;
    std::vector<const robot_model::JointModel*> planning_group_joint_models_;
    
    // pair of link index (optimization) and 
    std::vector<std::pair<std::string, Eigen::Vector3d> > goal_link_positions_;
    std::vector<std::pair<std::string, Eigen::Quaterniond> > goal_link_orientations_;
    
    std::vector<std::vector<char> > joints_affecting_goal_link_positions_;    // [goal_index][joint_index] = true or false
    std::vector<std::vector<char> > joints_affecting_goal_link_orientations_;
    
    // robot state contrainer for forward kinematics
    robot_state::RobotStatePtr robot_state_;
};

}


#endif // ITOMP_EXEC_GOAL_POSE_COST_H
