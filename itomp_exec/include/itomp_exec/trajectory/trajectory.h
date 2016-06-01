#ifndef ITOMP_EXEC_TRAJECTORY_H
#define ITOMP_EXEC_TRAJECTORY_H


#include <boost/shared_ptr.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <Eigen/Dense>
#include <ros/ros.h>


namespace itomp_exec
{

/**
 * Optimization values are stored like
 * [joint(0)_pos(t=0) joint(0)_vel(t=0) joint(1)_pos(t=0) joint(1)_vel(t=0) ... ]
 * 
 * @brief The Trajectory class
 */
class Trajectory
{
private:
    
    struct JointLimits
    {
        double lower;
        double upper;
        double lower_velocity;
        double upper_velocity;
    };

public:
    
    Trajectory(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    
    // visualization
    void setTrajectoryVisualizationTopic(const std::string& topic);
    void visualizeMilestones();
    void visualizeInterpolationSamples();
    
    /// Set robot model
    void setRobot(robot_model::RobotModelConstPtr& robot_model);
    
    /// Set planning group name, and then set other variables (lower/upper limits, etc.).
    void setRobotPlanningGroup(const std::string& group_name);
    
    inline void setTrajectoryDuration(double duration)
    {
        trajectory_duration_ = duration;
    }
    
    inline void setNumMilestones(int num_milestones)
    {
        num_milestones_ = num_milestones;
    }

    inline void setNumInterpolationSamples(int num_interpolation_samples)
    {
        num_interpolation_samples_ = num_interpolation_samples;
    }
    
    inline int getNumMilestones() const
    {
        return num_milestones_;
    }
    
    void initializeWithStartState(const robot_state::RobotState& start_state);
    
    void setOptimizationVariables(const Eigen::VectorXd& variables);
    
    Eigen::VectorXd getOptimizationVariables();
    Eigen::VectorXd getOptimizationVariableLowerLimits();
    Eigen::VectorXd getOptimizationVariableUpperLimits();
    
    /// assume robot_state is initialized with the same robot and default variables
    void setMilestoneVariablesPositionsToRobotState(robot_state::RobotState& robot_state, int milestone_index) const;
    void setMilestoneVariablesPositionsToRobotState(robot_state::RobotState& robot_state, int milestone_index, int interpolation_index) const;
    void setMilestoneVariablesPositionsToRobotState(robot_state::RobotState& robot_state, int milestone_index, double t) const; //!< t \in [0, 1]
    void getVariables(int milestone_index, double t, Eigen::VectorXd& positions, Eigen::VectorXd& velocities) const; //!< t \in [0, 1]
    
    /// set robot states with default whole body joint positions/velocities
    void setRobotStateWithStartState(robot_state::RobotState& robot_state) const;
    
    inline const Eigen::MatrixXd& getMilestoneVariables() const
    {
        return milestone_variables_;
    }
    
    inline Eigen::MatrixXd getMilestoneVariables()
    {
        return milestone_variables_;
    }
    
    inline const Eigen::VectorXd getMilestoneVariables(int index) const
    {
        return milestone_variables_.col(index);
    }
    
    inline Eigen::VectorXd getMilestoneVariables(int index)
    {
        return milestone_variables_.col(index);
    }
    
    inline const Eigen::VectorXd& getMilestoneStartVariables() const
    {
        return milestone_start_variables_;
    }
    
    inline Eigen::VectorXd getMilestoneStartVariables()
    {
        return milestone_start_variables_;
    }
    
    inline Eigen::VectorXd getMilestonePositions(int index)
    {
        return Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<2> >(milestone_variables_.col(index).data(), num_optimization_variables_at_point_);
    }
    
    /// returns time corresponding to milestone column index (exception: index==-1 <=> time=0)
    double getMilestoneTimeFromIndex(int index) const;
    
    /// each column has [joint(0).pos, joint(0).vel, joint(1).pos, joint(1).vel, ...] for each joint
    Eigen::MatrixXd getGroupJointPositionsAndVelocities();
    
    inline int getNumPlanningGroupJoints() const
    {
        return num_planning_group_joints_;
    }
    
    inline const std::vector<std::string>& getPlanningGroupJointNames() const
    {
        return planning_group_joint_names_;
    }
    
    inline const std::vector<const robot_model::JointModel*>& getPlanningGroupJointModels() const
    {
        return planning_group_joint_models_;
    }
    
    inline int getJointIndexInPlanningGroup(const std::string& joint_name) const
    {
        const std::map<std::string, int>::const_iterator it = planning_group_joint_name_to_index_map_.find(joint_name);
        return it != planning_group_joint_name_to_index_map_.end() ? it->second : -1;
    }
    
    // trajectory execution
    moveit_msgs::RobotTrajectory getPartialTrajectoryMsg(double t0, double t1, int num_states);
    void stepForward(double t);
    
    // DEBUG
    void printInfo();
    
private:
    
    // ROS visualization marker publishers
    ros::NodeHandle node_handle_;
    ros::Publisher trajectory_publisher_;
    robot_state::RobotStatePtr robot_state_for_visualization_;
    
    robot_model::RobotModelConstPtr robot_model_;
    std::string robot_planning_group_name_;
    
    double trajectory_duration_;
    int num_milestones_;
    int num_interpolation_samples_;
    
    // joint limits
    std::vector<int> planning_group_joint_indices_;  //!< index from planning group joint model to whole-body joint model
    std::vector<JointLimits> planning_group_joint_limits_;
    std::vector<std::string> planning_group_joint_names_;
    std::vector<const robot_model::JointModel*> planning_group_joint_models_;
    std::map<std::string, int> planning_group_joint_name_to_index_map_;
    
    // whole-body joint values, copied from start state
    Eigen::VectorXd default_whold_body_joint_positions_;
    Eigen::VectorXd default_whold_body_joint_velocities_;
    
    // milestone variables
    int num_planning_group_joints_;
    int num_optimization_variables_at_point_;
    Eigen::MatrixXd milestone_variables_; //!< each column <=> joint pos, vel at specific time
    Eigen::VectorXd milestone_start_variables_; //!< column vector for start state
    
    // DEBUG
    std::vector<std::string> optimization_variable_names_at_point_;
};

typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
typedef boost::shared_ptr<Trajectory const> TrajectoryConstPtr;


class TrajectoryDerivative
{
public:
    
    TrajectoryDerivative(const Trajectory& trajectory);
    TrajectoryDerivative(const Eigen::MatrixXd& milestone_variable_derivatives);
    TrajectoryDerivative& operator += (const TrajectoryDerivative& rhs);
    
    inline TrajectoryDerivative operator * (double rhs) const
    {
        return TrajectoryDerivative(milestone_variable_derivatives_ * rhs);
    }

    inline double& operator () (int i, int j)
    {
        return milestone_variable_derivatives_(i,j);
    }
    
    Eigen::VectorXd getOptimizationVariables();
    
private:
    
    Eigen::MatrixXd milestone_variable_derivatives_;
};

}


#endif // ITOMP_EXEC_TRAJECTORY_H
