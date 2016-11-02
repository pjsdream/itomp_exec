#ifndef ITOMP_EXEC_COST_H
#define ITOMP_EXEC_COST_H


#include <ros/ros.h>


namespace itomp_exec
{

class ItompOptimizer;

class Cost
{
public:
    
    Cost(double weight);

    virtual void printInfo();

protected:

    double weight_;
};

}


#endif // ITOMP_EXEC_COST_H
