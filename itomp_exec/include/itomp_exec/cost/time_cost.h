#ifndef ITOMP_EXEC_TIME_COST_H
#define ITOMP_EXEC_TIME_COST_H


#include <itomp_exec/cost/cost.h>


namespace itomp_exec
{

class TimeCost : public Cost
{
private:

    static const double ratio_cosine_to_meter_;

public:

    TimeCost(ITOMPOptimizer& optimizer, double weight = 1.0);

    inline virtual std::string getString()
    {
        return "TimeCost";
    }

    virtual void addCost();
    virtual void addDerivative();

private:
};

}


#endif // ITOMP_EXEC_TIME_COST_H
