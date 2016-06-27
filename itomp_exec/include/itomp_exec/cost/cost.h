#ifndef ITOMP_EXEC_COST_H
#define ITOMP_EXEC_COST_H


namespace itomp_exec
{

class ITOMPOptimizer;

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

    virtual void addCost();
    virtual void addDerivative();

private:

    double weight_;
    ITOMPOptimizer& optimizer_;
};

}


#endif // ITOMP_EXEC_COST_H
