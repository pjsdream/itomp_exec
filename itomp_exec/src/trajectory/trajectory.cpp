#include <itomp_exec/trajectory/trajectory.h>
#include <ros/console.h>
#include <ecl/geometry.hpp>

namespace itomp_exec
{

// Trajectory
Trajectory::Trajectory()
{
}

void Trajectory::printInfo()
{
    const Eigen::VectorXd lower_limits = getOptimizationVariableLowerLimits();
    const Eigen::VectorXd upper_limits = getOptimizationVariableUpperLimits();
    
    ROS_INFO("Optimization variable names:");
    int idx = 0;
    for (int i=0; i<num_milestones_; i++)
    {
        for (int j=0; j<optimization_variable_names_at_point_.size(); j++)
        {
            ROS_INFO(" * %30s(%2d): %lf ~ %lf", optimization_variable_names_at_point_[j].c_str(), i, lower_limits[idx], upper_limits[idx]);
            idx++;
        }
    }
}

void Trajectory::setRobot(robot_model::RobotModelConstPtr& robot_model)
{
    robot_model_ = robot_model;
}

void Trajectory::setRobotPlanningGroup(const std::string& group_name)
{
    robot_planning_group_name_ = group_name;
    
    const robot_model::JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name);
    
    // joint indices mapping
    const std::vector<const robot_model::JointModel*> whole_body_joints = robot_model_->getActiveJointModels();
    std::vector<std::string> whole_body_joint_names(whole_body_joints.size());
    for (int i=0; i<whole_body_joints.size(); i++)
        whole_body_joint_names[i] = whole_body_joints[i]->getName();
    
    const std::vector<std::string> group_joint_names = joint_group->getActiveJointModelNames();
    
    planning_group_joint_indices_.resize(group_joint_names.size());
    for (int i=0; i<group_joint_names.size(); i++)
        planning_group_joint_indices_[i] = std::find(whole_body_joint_names.begin(), whole_body_joint_names.end(), group_joint_names[i]) - whole_body_joint_names.begin();
    
    // joint limits
    const robot_model::JointBoundsVector group_joint_bounds = joint_group->getActiveJointModelsBounds();
    planning_group_joint_limits_.resize(group_joint_bounds.size());
    for (int i=0; i<group_joint_bounds.size(); i++)
    {
        planning_group_joint_limits_[i].lower = group_joint_bounds[i]->at(0).min_position_;
        planning_group_joint_limits_[i].upper = group_joint_bounds[i]->at(0).max_position_;
        planning_group_joint_limits_[i].lower_velocity = group_joint_bounds[i]->at(0).min_velocity_;
        planning_group_joint_limits_[i].upper_velocity = group_joint_bounds[i]->at(0).max_velocity_;
    }
    
    // optimization variable names
    optimization_variable_names_at_point_.clear();
    for (int i=0; i<group_joint_names.size(); i++)
    {
        optimization_variable_names_at_point_.push_back( group_joint_names[i] + "_pos" );
        optimization_variable_names_at_point_.push_back( group_joint_names[i] + "_vel" );
    }
    
    // optimization variables
    num_optimization_variables_at_point_ = planning_group_joint_limits_.size() * 2;
}

void Trajectory::initializeWithStartState(const robot_state::RobotState& start_state)
{
    const std::vector<const robot_model::JointModel*> joint_models = robot_model_->getActiveJointModels();
    
    default_whold_body_joint_positions_.resize(joint_models.size());
    default_whold_body_joint_velocities_.resize(joint_models.size());
    for (int i=0; i<joint_models.size(); i++)
    {
        const std::vector< std::string > & 	vnames = joint_models[i]->getVariableNames () ;
        for (int i=0; i<vnames.size(); i++) printf("(%d) %s\n", i, vnames[i].c_str());
        
        default_whold_body_joint_positions_[i] = start_state.getJointPositions(joint_models[i])[0];
        default_whold_body_joint_velocities_[i] = start_state.getJointVelocities(joint_models[i])[0];
    }
    
    milestone_variables_.resize(planning_group_joint_limits_.size() * 2, num_milestones_);
    milestone_start_variables_.resize(planning_group_joint_limits_.size() * 2);
    
    for (int i=0; i<planning_group_joint_limits_.size(); i++)
    {
        const int joint_idx = planning_group_joint_indices_[i];
        
        ecl::CubicPolynomial cubic = ecl::CubicPolynomial::DerivativeInterpolation(
                    0.0, default_whold_body_joint_positions_[joint_idx], default_whold_body_joint_velocities_[joint_idx],
                    trajectory_duration_, default_whold_body_joint_positions_[joint_idx], 0.0
                    );
        
        milestone_start_variables_(2*i)   = cubic(0.);
        milestone_start_variables_(2*i+1) = cubic.derivative(0.);
                
        for (int j=0; j<num_milestones_; j++)
        {
            const double t = getMilestoneTimeFromIndex(j);
            milestone_variables_(2*i  , j) = cubic(t);
            milestone_variables_(2*i+1, j) = cubic.derivative(t);
        }
    }
}

void Trajectory::setOptimizationVariables(const Eigen::VectorXd& variables)
{
    milestone_variables_ = Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(const_cast<double*>(variables.data()), num_optimization_variables_at_point_, num_milestones_);
}

Eigen::VectorXd Trajectory::getOptimizationVariables()
{
    return Eigen::Map<Eigen::VectorXd, Eigen::Aligned>(milestone_variables_.data(), milestone_variables_.rows() * milestone_variables_.cols());
}

Eigen::VectorXd Trajectory::getOptimizationVariableLowerLimits()
{
    Eigen::VectorXd lower_limits(planning_group_joint_limits_.size() * 2);
    
    int idx = 0;
    for (int i=0; i<planning_group_joint_limits_.size(); i++)
    {
        lower_limits(idx++) = planning_group_joint_limits_[i].lower;
        lower_limits(idx++) = planning_group_joint_limits_[i].lower_velocity;
    }
    
    return lower_limits.replicate(num_milestones_, 1);
}

Eigen::VectorXd Trajectory::getOptimizationVariableUpperLimits()
{
    Eigen::VectorXd upper_limits(planning_group_joint_limits_.size() * 2);
    
    int idx = 0;
    for (int i=0; i<planning_group_joint_limits_.size(); i++)
    {
        upper_limits(idx++) = planning_group_joint_limits_[i].upper;
        upper_limits(idx++) = planning_group_joint_limits_[i].upper_velocity;
    }
    
    return upper_limits.replicate(num_milestones_, 1);
}

double Trajectory::getMilestoneTimeFromIndex(int index) const
{
    return (double)(index+1) / num_milestones_ * trajectory_duration_;
}

// TrajectoryDerivative
TrajectoryDerivative::TrajectoryDerivative(const Trajectory& trajectory)
{
    const Eigen::MatrixXd& milestone_variables = trajectory.getMilestoneVariables();
    milestone_variable_derivatives_.resize(milestone_variables.rows(), milestone_variables.cols());
    milestone_variable_derivatives_.setZero();
}

TrajectoryDerivative::TrajectoryDerivative(const Eigen::MatrixXd& milestone_variable_derivatives)
{
    milestone_variable_derivatives_ = milestone_variable_derivatives;
}

TrajectoryDerivative& TrajectoryDerivative::operator += (const TrajectoryDerivative& rhs)
{
    milestone_variable_derivatives_ += rhs.milestone_variable_derivatives_;
    return *this;
}

Eigen::VectorXd TrajectoryDerivative::getOptimizationVariables()
{
    return Eigen::Map<Eigen::VectorXd, Eigen::Aligned>(milestone_variable_derivatives_.data(), milestone_variable_derivatives_.rows() * milestone_variable_derivatives_.cols());
}

}
