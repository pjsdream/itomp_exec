#ifndef ITOMP_EXEC_SMOOTHNESS_COST_H
#define ITOMP_EXEC_SMOOTHNESS_COST_H


#include <itomp_exec/cost/cost.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class SmoothnessCost : public Cost
{
public:
        
    SmoothnessCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    virtual void addCost();
    virtual void addDerivative();

private:

    Eigen::Matrix4d H2_;
};

}


#endif // ITOMP_EXEC_SMOOTHNESS_COST_H
