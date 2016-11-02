#include <itomp_exec/trajectory/itomp_trajectory.h>


namespace itomp_exec
{

ItompTrajectory::ItompTrajectory(const RobotModel* robot_model)
    : robot_model_(robot_model)
{
}

ItompTrajectory::~ItompTrajectory()
{
}

void ItompTrajectory::setRobotStartState(const RobotState& robot_start_state)
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

std::vector<RobotState> ItompTrajectory::getInterpolatedFirstTimestepTrajectory(int num_interpolation_midpoints)
{
}

std::vector<RobotState> ItompTrajectory::getInterpolatedWholeTrajectory(int num_interpolation_midpoints)
{
}

void ItompTrajectory::extendByOneTimestep()
{
}

RobotState ItompTrajectory::planningJointValuesToRobotState()
{
}

}
