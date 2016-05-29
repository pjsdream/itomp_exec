#ifndef ITOMP_EXEC_TRAJECTORY_H
#define ITOMP_EXEC_TRAJECTORY_H


#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>


namespace itomp_exec
{

class Trajectory
{
public:
    
    Trajectory();
    
    void setOptimizationVariables(const Eigen::VectorXd& variables);
    
    Eigen::VectorXd getOptimizationVariables();
    
    Eigen::VectorXd getOptimizationVariableLowerLimits();
    Eigen::VectorXd getOptimizationVariableUpperLimits();
    
private:
};

class TrajectoryDerivative
{
public:
    
    TrajectoryDerivative();
    TrajectoryDerivative& operator += (const TrajectoryDerivative& rhs);
    
    Eigen::VectorXd getOptimizationVariables();
    
private:
};

typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
typedef boost::shared_ptr<Trajectory const> TrajectoryConstPtr;

}


#endif // ITOMP_EXEC_TRAJECTORY_H
