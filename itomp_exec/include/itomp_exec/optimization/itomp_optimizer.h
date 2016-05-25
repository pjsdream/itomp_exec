#ifndef ITOMP_EXEC_ITOMP_OPTIMIZER_H
#define ITOMP_EXEC_ITOMP_OPTIMIZER_H


#include <itomp_exec/trajectory/trajectory.h>


namespace itomp_exec
{

class ITOMPOptimizer
{
public:
    
    ITOMPOptimizer(const TrajectoryPtr& trajectory);
    ~ITOMPOptimizer();
    
    void optimize();
    
private:
    
    TrajectoryPtr trajectory_;
};

}


#endif // ITOMP_EXEC_ITOMP_OPTIMIZER_H
