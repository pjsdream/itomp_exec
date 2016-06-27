#include <itomp_exec/cost/future_obstacle_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <eigen_conversions/eigen_msg.h>


namespace itomp_exec
{

FutureObstacleCost::FutureObstacleCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

}
