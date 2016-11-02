#include <itomp_exec/optimization/itomp_optimizer.h>
#include <visualization_msgs/MarkerArray.h>
#include <itomp_exec/util/gaussian_quadrature.h>
#include <itomp_exec/optimization/optimization_stop_strategy.h>
#include <itomp_exec/optimization/optimization_search_strategy.h>
#include <itomp_exec/optimization/find_min_box_constrained_thread_safe.h>
#include <eigen_conversions/eigen_msg.h>

#include <random>

#include <ros/ros.h>

#include <omp.h>


namespace itomp_exec
{

ItompOptimizer::ItompOptimizer(ItompTrajectory* trajectory)
    : trajectory_(trajectory)
    , trajectory_publisher_(0)
{
}

ItompOptimizer::~ItompOptimizer()
{
}

void ItompOptimizer::enableVisualizeTrajectoryEachStep(ros::Publisher* publisher)
{
    trajectory_publisher_ = publisher;
}

void ItompOptimizer::disableVisualizeTrajectoryEachStep()
{
    trajectory_publisher_ = 0;
}

void ItompOptimizer::setCostFunction(int id, Cost *cost)
{
    cost_functions_[id] = cost;
}

void ItompOptimizer::printCostFunctions()
{
    ROS_INFO("Cost functions:");

    for (std::map<int, Cost*>::iterator it = cost_functions_.begin(); it != cost_functions_.end(); it++)
    {
        ROS_INFO("Cost function id %d", it->first);
        it->second->printInfo();
    }

    ROS_INFO("");
}

void ItompOptimizer::optimize(double time_limit)
{
}

}
