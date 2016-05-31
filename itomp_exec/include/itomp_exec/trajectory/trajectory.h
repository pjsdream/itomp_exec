#ifndef ITOMP_EXEC_TRAJECTORY_H
#define ITOMP_EXEC_TRAJECTORY_H


#include <boost/shared_ptr.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>


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
    
    Trajectory();
    
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
    
    void initializeWithStartState(const robot_state::RobotState& start_state);
    
    void setOptimizationVariables(const Eigen::VectorXd& variables);
    
    Eigen::VectorXd getOptimizationVariables();
    Eigen::VectorXd getOptimizationVariableLowerLimits();
    Eigen::VectorXd getOptimizationVariableUpperLimits();
    
    inline const Eigen::MatrixXd& getMilestoneVariables() const
    {
        return milestone_variables_;
    }
    
    inline Eigen::MatrixXd getMilestoneVariables()
    {
        return milestone_variables_;
    }
    
    inline const Eigen::VectorXd& getMilestoneStartVariables() const
    {
        return milestone_start_variables_;
    }
    
    inline Eigen::VectorXd getMilestoneStartVariables()
    {
        return milestone_start_variables_;
    }
    
    /// returns time corresponding to milestone column index (exception: index==-1 <=> time=0)
    double getMilestoneTimeFromIndex(int index) const;
    
    /// each column has [joint(0).pos, joint(0).vel, joint(1).pos, joint(1).vel, ...] for each joint
    Eigen::MatrixXd getGroupJointPositionsAndVelocities();
    
    // DEBUG
    void printInfo();
    
private:
    
    robot_model::RobotModelConstPtr robot_model_;
    std::string robot_planning_group_name_;
    
    double trajectory_duration_;
    int num_milestones_;
    int num_interpolation_samples_;
    
    // joint limits
    std::vector<int> planning_group_joint_indices_;  //!< index from planning group joint model to whole-body joint model
    std::vector<JointLimits> planning_group_joint_limits_;
    
    // whole-body joint values, copied from start state
    Eigen::VectorXd default_whold_body_joint_positions_;
    Eigen::VectorXd default_whold_body_joint_velocities_;
    
    // milestone variables
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
    
    Eigen::VectorXd getOptimizationVariables();
    
private:
    
    Eigen::MatrixXd milestone_variable_derivatives_;
};

}


#endif // ITOMP_EXEC_TRAJECTORY_H
