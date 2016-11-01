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

}
