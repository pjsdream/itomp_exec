#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/trajectory/trajectory.h>
#include <itomp_exec/cost/cost.h>
#include <dlib/optimization.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class ITOMPOptimizer
{
private:
    
    typedef dlib::matrix<double,0,1> column_vector;
    
public:
    
    ITOMPOptimizer(const TrajectoryPtr& trajectory);
    ~ITOMPOptimizer();
    
    inline void setOptimizationTimeLimit(double time)
    {
        optimization_time_limit_ = time;
    }
    
    void generateCostFunctions(const std::vector<std::pair<std::string, double> > cost_weights);
    
    void optimize();
    
private:
    
    // dlib function value/derivative evaluation
    double cost(const column_vector& variables);
    const column_vector cost_derivative(const column_vector& variables);
    static const Eigen::VectorXd convertDlibToEigenVector(const column_vector& v);
    static const column_vector convertEigenToDlibVector(const Eigen::VectorXd& v);
    
    TrajectoryPtr trajectory_;
    
    // Optimizer class owns the pointers to cost evaluators
    std::vector<Cost*> cost_functions_;
    
    double optimization_time_limit_; //!< if 0, no time limit. optimization ends until it finds a minimum.
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
