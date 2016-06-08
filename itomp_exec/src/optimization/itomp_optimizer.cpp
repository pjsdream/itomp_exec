#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <visualization_msgs/MarkerArray.h>

#include <functional>

#include <ros/ros.h>


namespace itomp_exec
{

ITOMPOptimizer::ITOMPOptimizer()
    : trajectory_duration_(0.)
{
}

ITOMPOptimizer::~ITOMPOptimizer()
{
}

void ITOMPOptimizer::setPlanningRobotStartState(const RobotState& start_state, const std::string& planning_group_name, double trajectory_duration, int num_milestones)
{
    trajectory_duration_ = trajectory_duration;
    
    start_state_ = start_state;
    start_state_.setPlanningGroup(planning_group_name);
    
    start_milestone_.resize(planning_group_joint_indices_.size() * 2);
    milestones_.resize(planning_group_joint_indices_.size() * 2, num_milestones);
    for (int i=0; i<planning_group_joint_indices_.size(); i++)
    {
    }
}

void ITOMPOptimizer::stepForward(double time)
{
    if (trajectory_duration_ <= time + 1e-6)
    {
        trajectory_duration_ = 0.;
        return;
    }

    // TODO: milestones

    trajectory_duration_ -= time;
}

void ITOMPOptimizer::optimize()
{
    ros::WallTime start_time = ros::WallTime::now();
    
    column_vector initial_variables = convertEigenToDlibVector( getOptimizationVariables() );
    column_vector lower = convertEigenToDlibVector( getOptimizationVariableLowerLimits() );
    column_vector upper = convertEigenToDlibVector( getOptimizationVariableUpperLimits() );
    
    const int optimization_max_iter = 10;
    
    while ((ros::WallTime::now() - start_time).toSec() < optimization_time_limit_)
    {
        dlib::find_min_box_constrained(
                    dlib::bfgs_search_strategy(),
                    dlib::objective_delta_stop_strategy(1e-7, optimization_max_iter),
                    std::bind(&ITOMPOptimizer::optimizationCost, this, std::placeholders::_1),
                    std::bind(&ITOMPOptimizer::optimizationCostDerivative, this, std::placeholders::_1),
                    initial_variables,
                    lower,
                    upper
                    );
        
        // derivative test
        /*
        const int cost_index = 1;
        trajectory_->setOptimizationVariables( convertDlibToEigenVector(initial_variables) );
        const double eps = 1e-6;
        const double c = cost_functions_[cost_index]->cost(*trajectory_);
        column_vector d = convertEigenToDlibVector( cost_functions_[cost_index]->derivative(*trajectory_).getOptimizationVariables() );
        trajectory_->setOptimizationVariables( convertDlibToEigenVector(initial_variables + d * eps) );
        const double cd = cost_functions_[cost_index]->cost(*trajectory_);
        trajectory_->setOptimizationVariables( convertDlibToEigenVector(initial_variables) );
        
        printf("%lf -> %lf, (diff: %lf)\n", c, cd, (cd - c) / eps);
        if (cd - c < 0)
            cost_functions_[cost_index]->derivative(*trajectory_);
        */
        
        // visualize trajectory
        visualizeMilestones();
        visualizeInterpolationSamples();
    }
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariables()
{
    return Eigen::Map<Eigen::VectorXd>(milestones_.data(), milestones_.rows() * milestones_.cols());
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariableLowerLimits()
{
    return optimization_variable_lower_limits_;
}

Eigen::VectorXd ITOMPOptimizer::getOptimizationVariableUpperLimits()
{
    return optimization_variable_upper_limits_;
}

double ITOMPOptimizer::cost()
{
    // TODO
    return 0.;
}

// dlib functions
double ITOMPOptimizer::optimizationCost(const column_vector& variables)
{
    // TODO
    return 0.;
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    // TODO
    Eigen::VectorXd v(variables.size());
    v.setZero();
    return convertEigenToDlibVector( v );
}

const Eigen::VectorXd ITOMPOptimizer::convertDlibToEigenVector(const column_vector& v)
{
    return Eigen::Map<Eigen::VectorXd>((double*)v.begin(), v.size());
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::convertEigenToDlibVector(const Eigen::VectorXd& v)
{
    const int n = v.rows();
    column_vector r(n);
    memcpy(r.begin(), v.data(), sizeof(double) * n);
    return r;
}

void ITOMPOptimizer::setVisualizationTopic(ros::NodeHandle node_handle, const std::string& topic)
{
    milestone_visualization_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

void ITOMPOptimizer::visualizeMilestones()
{
    // TODO
}

void ITOMPOptimizer::visualizeInterpolationSamples()
{
    // TODO
}

}
