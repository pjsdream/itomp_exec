#ifndef ITOMP_EXEC_SMOOTHNESS_COST_H
#define ITOMP_EXEC_SMOOTHNESS_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class SmoothnessCost : public Cost
{
    ITOMP_COST_DERIVED_CLASS_DECL(Smoothness)
    
public:
        
    SmoothnessCost(double weight = 1.0);
    
    virtual double cost(TrajectoryConstPtr trajectory);
    virtual TrajectoryDerivative derivative(const TrajectoryConstPtr trajectory);
    
private:
};

}


#endif // ITOMP_EXEC_SMOOTHNESS_COST_H
