#include <itomp_exec/trajectory/itomp_trajectory.h>


namespace itomp_exec
{

ItompTrajectory::ItompTrajectory()
{
}

ItompTrajectory::~ItompTrajectory()
{
}

void ItompTrajectory::setRobotModel(const PlanningRobotModel* robot_model)
{
    robot_model_ = robot_model;
}

void ItompTrajectory::setRobotStartState(const PlanningRobotState& robot_start_state)
{
}

void ItompTrajectory::setPlanningGroup(const std::string& planning_group)
{
}

void ItompTrajectory::setNumWaypoints(int num_waypoints)
{
}

void ItompTrajectory::setTimestep(double timestep)
{
    timestep_ = timestep;
}

void ItompTrajectory::setTrajectoryDuration(double trajectory_duration)
{
    trajectory_duration_ = trajectory_duration;
}

void ItompTrajectory::initializeTrajectory()
{
}

std::vector<PlanningRobotState> ItompTrajectory::getInterpolatedFirstTimestepTrajectory(int num_interpolation_midpoints)
{
}

std::vector<PlanningRobotState> ItompTrajectory::getInterpolatedWholeTrajectory(int num_interpolation_midpoints)
{
}

void ItompTrajectory::extendByOneTimestep()
{
}

PlanningRobotState ItompTrajectory::planningJointValuesToRobotState()
{
}

}
