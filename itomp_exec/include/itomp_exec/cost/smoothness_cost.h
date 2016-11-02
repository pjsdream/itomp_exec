#ifndef ITOMP_EXEC_SMOOTHNESS_COST_H
#define ITOMP_EXEC_SMOOTHNESS_COST_H


#include <itomp_exec/cost/cost.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class SmoothnessCost : public Cost
{
public:

    SmoothnessCost(double weight = 1.);

    virtual void printInfo();

private:

};

}


#endif // ITOMP_EXEC_SMOOTHNESS_COST_H
