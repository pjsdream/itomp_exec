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
    num_joints_ = robot_model->numJoints();
}

void ItompTrajectory::setRobotStartState(const PlanningRobotState& robot_start_state)
{
}

void ItompTrajectory::setNumWaypoints(int num_waypoints)
{
    num_waypoints_ = num_waypoints;
}

void ItompTrajectory::setTimestep(double timestep)
{
    timestep_ = timestep;
}

void ItompTrajectory::setTrajectoryDuration(double trajectory_duration)
{
    trajectory_duration_ = trajectory_duration;
}

void ItompTrajectory::setWaypointVariables(const Eigen::MatrixXd& waypoints)
{
    Q_ = waypoints;
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

Eigen::MatrixXd& ItompTrajectory::getWaypointVariables()
{
    return Q_;
}

const Eigen::MatrixXd& ItompTrajectory::getWaypointVariables() const
{
    return Q_;
}

const PlanningRobotModel* ItompTrajectory::getRobotModel() const
{
    return robot_model_;
}

int ItompTrajectory::numWaypoints() const
{
    return num_waypoints_;
}

double ItompTrajectory::getTrajectoryDuration() const
{
    return trajectory_duration_;
}

}
