#ifndef ITOMP_EXEC_TIME_COST_H
#define ITOMP_EXEC_TIME_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class TimeCost : public Cost
{
public:

    TimeCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    virtual void addCost();
    virtual void addDerivative();

private:
};

}


#endif // ITOMP_EXEC_TIME_COST_H
