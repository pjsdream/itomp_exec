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
{
}

ItompOptimizer::~ItompOptimizer()
{
}

}
