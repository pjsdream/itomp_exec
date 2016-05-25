#ifndef ITOMP_EXEC_COST_H
#define ITOMP_EXEC_COST_H


namespace itomp_exec
{

class TrajectoryDerivative
{
public:
    
    TrajectoryDerivative();
    
private:
};

class Cost
{
public:
    
    Cost();
    
    virtual double cost();
    virtual TrajectoryDerivative derivative();
    
private:
};

}


#endif // ITOMP_EXEC_COST_H
