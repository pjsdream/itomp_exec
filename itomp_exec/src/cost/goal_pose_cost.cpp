#include <itomp_exec/cost/goal_pose_cost.h>
#include <itomp_exec/planner/itomp_planner_node.h>


namespace itomp_exec
{

GoalPoseCost::GoalPoseCost(double weight)
    : Cost(weight)
{
}

void GoalPoseCost::initialize(const ITOMPPlannerNode& planner_node)
{
    const std::vector<std::pair<std::string, Eigen::Vector3d> > goal_link_positions = planner_node.getGoalLinkPositions();
    const std::vector<std::pair<std::string, Eigen::Quaterniond> > goal_link_orientations = planner_node.getGoalLinkOrientations();
    const Trajectory& trajectory_template = planner_node.getTrajectoryTemplate();
    
    goal_link_positions_.resize(goal_link_positions.size());
    for (int i=0; i<goal_link_positions.size(); i++)
    {
        const std::string& name = goal_link_positions[i].first;
        const Eigen::Vector3d& position = goal_link_positions[i].second;
    }
}

double GoalPoseCost::cost(const Trajectory& trajectory)
{
    return 0.0;
}

TrajectoryDerivative GoalPoseCost::derivative(const Trajectory& trajectory)
{
    return TrajectoryDerivative(trajectory);
}

}
