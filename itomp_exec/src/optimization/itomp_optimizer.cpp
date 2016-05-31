#include <itomp_exec/optimization/itomp_optimizer.h>
#include <itomp_exec/cost/cost_factory.h>
#include <itomp_exec/planner/itomp_planner_node.h>

#include <functional>

#include <ros/ros.h>


namespace itomp_exec
{

ITOMPOptimizer::ITOMPOptimizer(const TrajectoryPtr& trajectory)
    : trajectory_(trajectory)
{
}

ITOMPOptimizer::~ITOMPOptimizer()
{
    trajectory_.reset();
    
    for (int i=0; i<cost_functions_.size(); i++)
        delete cost_functions_[i];
}

void ITOMPOptimizer::generateCostFunctions(const std::vector<std::pair<std::string, double> > cost_weights)
{
    for (int i=0; i<cost_weights.size(); i++)
    {
        Cost* cost = CostFactory::newCost(cost_weights[i].first, cost_weights[i].second);
        if (cost != 0)
            cost_functions_.push_back(cost);
    }
}

void ITOMPOptimizer::initializeCostFunctions(const ITOMPPlannerNode& planner_node)
{
    for (int i=0; i<cost_functions_.size(); i++)
        cost_functions_[i]->initialize(planner_node);
}

void ITOMPOptimizer::optimize()
{
    ros::WallTime start_time = ros::WallTime::now();
    
    column_vector initial_variables = convertEigenToDlibVector( trajectory_->getOptimizationVariables() );
    column_vector lower = convertEigenToDlibVector( trajectory_->getOptimizationVariableLowerLimits() );
    column_vector upper = convertEigenToDlibVector( trajectory_->getOptimizationVariableUpperLimits() );
    
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
    }
}

double ITOMPOptimizer::cost()
{
    const int n = cost_functions_.size();
    double cost = 0.;
    
    for (int i=0; i<n; i++)
        cost += cost_functions_[i]->cost(*trajectory_);
    
    return cost;
}

// dlib functions
double ITOMPOptimizer::optimizationCost(const column_vector& variables)
{
    trajectory_->setOptimizationVariables( convertDlibToEigenVector(variables) );
    
    const int n = cost_functions_.size();
    double cost = 0.;
    
    for (int i=0; i<n; i++)
        cost += cost_functions_[i]->cost(*trajectory_);
    
    return cost;
}

const ITOMPOptimizer::column_vector ITOMPOptimizer::optimizationCostDerivative(const column_vector& variables)
{
    trajectory_->setOptimizationVariables( convertDlibToEigenVector(variables) );
    
    const int n = cost_functions_.size();
    TrajectoryDerivative trajectory_derivative(*trajectory_);
    
    for (int i=0; i<n; i++)
        trajectory_derivative += cost_functions_[i]->derivative(*trajectory_);
    
    // TODO: convert from trajectory derivative to optimization variables
    return convertEigenToDlibVector( trajectory_derivative.getOptimizationVariables() );
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

}
