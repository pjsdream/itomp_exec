#ifndef ITOMP_EXEC_COST_H
#define ITOMP_EXEC_COST_H


#include <string>


namespace itomp_exec
{

class ITOMPOptimizer;
class Numerical;

class Cost
{
public:
    
    Cost(ITOMPOptimizer& optimizer, double weight = 1.0);

    inline double getWeight()
    {
        return weight_;
    }

    inline ITOMPOptimizer& getOptimizer()
    {
        return optimizer_;
    }

    inline virtual std::string getString()
    {
        return "Cost";
    }

    virtual void addCost();
    virtual void addDerivative();

    void addDerivativeByInterpolationIndex(int joint_index, int interpolation_index, double derivative);

private:

    double weight_;
    ITOMPOptimizer& optimizer_;
};

}


#endif // ITOMP_EXEC_COST_H
