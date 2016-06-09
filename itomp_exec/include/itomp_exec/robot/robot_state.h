#ifndef ITOMP_EXEC_ROBOT_STATE_H
#define ITOMP_EXEC_ROBOT_STATE_H


#include <itomp_exec/robot/robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class RobotState
{
public:

    RobotState();
    
    inline void setRobotModel(const RobotModelPtr& robot_model)
    {
        robot_model_ = robot_model;
    }
    
    inline const std::vector<std::string>& getPlanningJointNames() const
    {
        return planning_joint_names_;
    }
    
    inline const std::vector<int>& getPlanningJointIndices() const
    {
        return planning_joint_indices_;
    }
    
    inline int getNumPlanningJoints() const
    {
        return planning_joint_positions_.rows();
    }
    
    inline const Eigen::VectorXd& getPlanningJointPositionLowerLimits() const
    {
        return planning_joint_position_lower_limits_;
    }
    
    inline const Eigen::VectorXd& getPlanningJointPositionUpperLimits() const
    {
        return planning_joint_position_upper_limits_;
    }
    
    inline const Eigen::VectorXd& getPlanningJointVelocityLowerLimits() const
    {
        return planning_joint_velocity_lower_limits_;
    }
    
    inline const Eigen::VectorXd& getPlanningJointVelocityUpperLimits() const
    {
        return planning_joint_velocity_upper_limits_;
    }
    
    inline const Eigen::VectorXd& getPlanningJointPositions() const
    {
        return planning_joint_positions_;
    }
    
    inline const Eigen::VectorXd& getPlanningJointVelocities() const
    {
        return planning_joint_velocities_;
    }
    
    inline void setPlanningJointPositions(const Eigen::VectorXd& positions)
    {
        planning_joint_positions_ = positions;
    }
    
    inline void setPlanningJointVelocities(const Eigen::VectorXd& velocities)
    {
        planning_joint_velocities_ = velocities;
    }
    
    inline const Eigen::VectorXd& getDefaultJointPositions() const
    {
        return default_joint_positions_;
    }
    
    void setPlanningGroup(const std::string& planning_group_name);
    
    void initFromMoveitRobotStateMsg(const moveit_msgs::RobotState& msg);

private:
    
    RobotModelPtr robot_model_;
    
    // information of planning group joints
    std::string planning_group_name_;
    std::vector<std::string> planning_joint_names_;
    std::vector<int> planning_joint_indices_;
    Eigen::VectorXd planning_joint_position_lower_limits_;
    Eigen::VectorXd planning_joint_position_upper_limits_;
    Eigen::VectorXd planning_joint_velocity_lower_limits_;
    Eigen::VectorXd planning_joint_velocity_upper_limits_;
    Eigen::VectorXd planning_joint_positions_;
    Eigen::VectorXd planning_joint_velocities_;
    
    // joint index to planning group joint index
    // v[i] = -1 if i-th joint is not in the planning group
    std::vector<int> joint_index_to_planning_joint_index_;
    
    // default values for all joints
    Eigen::VectorXd default_joint_positions_;
    Eigen::VectorXd default_joint_velocities_;
};

}


#endif // ITOMP_EXEC_ROBOT_STATE_H
