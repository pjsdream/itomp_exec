#include <itomp_exec/cost/future_obstacle_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>
#include <eigen_conversions/eigen_msg.h>


namespace itomp_exec
{

FutureObstacleCost::FutureObstacleCost(double weight)
    : Cost(weight)
{
}

void FutureObstacleCost::initialize(const ITOMPPlannerNode &planner_node)
{
    // copy the future obstacle data
    pcml::FutureObstacleDistributionsConstPtr future_obstacle_distributions = planner_node.getRecentFutureObstacleDistributions();
    
    if (future_obstacle_distributions != 0)
    {
        for (int i=0; i<future_obstacle_distributions->obstacles.size(); i++)
        {
            const pcml::FutureObstacleDistribution& distribution = future_obstacle_distributions->obstacles[i];
            Eigen::Vector3d center;
            tf::pointMsgToEigen(distribution.obstacle_point, center);
            const Eigen::Matrix3d covariance = Eigen::Map<Eigen::Matrix3d, Eigen::Unaligned, Eigen::Stride<1, 3> >(const_cast<double*>(&distribution.obstacle_covariance[0]));
            const double weight = distribution.weight;
            
            // SVD decompositions to retrieve axes
        }
    }
}

double FutureObstacleCost::cost(const Trajectory& trajectory)
{
    const int num_milestones = trajectory.getNumMilestones();
    const int num_interpolation_samples = trajectory.getNumInterpolationSamples();
    
    for (int i=0; i<num_milestones; i++)
    {
        for (int j = -1; j < num_interpolation_samples; j++)
        {
        }
    }
}

}
