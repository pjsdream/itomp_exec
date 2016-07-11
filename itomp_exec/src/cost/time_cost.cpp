#include <itomp_exec/cost/time_cost.h>
#include <itomp_exec/optimization/itomp_optimizer.h>


namespace itomp_exec
{

TimeCost::TimeCost(ITOMPOptimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

void TimeCost::addCost()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    double& cost = optimizer.cost();

    const int num_interpolated_variables = optimizer.getNumInterpolatedConfigurations();
    const int num_robot_joints = optimizer.getNumRobotJoints();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        const double interpolated_time = optimizer.getInterpolatedTime(i);

        for (int j=0; j<num_robot_joints; j++)
        {
            const ITOMPOptimizer::GoalLinkPose& goal_link_pose = optimizer.getGoalLinkPose(i);

            const double& position_weight = goal_link_pose.position_weight;
            const double& orientation_weight = goal_link_pose.orientation_weight;

            if (position_weight != 0.)
            {
                const Eigen::Vector3d& link_transform = optimizer.getInterpolatedLinkTransform(i, j).translation();
            }

            if (orientation_weight != 0.)
            {
                const Eigen::Quaterniond& link_orientation( optimizer.getInterpolatedLinkTransform(i, j).linear() );
            }
        }
    }
}

void TimeCost::addDerivative()
{
    ITOMPOptimizer& optimizer = getOptimizer();
    const double weight = getWeight();

    Eigen::MatrixXd& milestone_derivative = optimizer.milestoneDerivative();
}

}
