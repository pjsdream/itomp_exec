#include <itomp_exec/robot/planning_robot_state.h>


namespace itomp_exec
{

PlanningRobotState::PlanningRobotState(const PlanningRobotModel* robot_model)
    : robot_model_(robot_model)
    , joint_positions_(robot_model->numJoints())
    , joint_velocities_(robot_model->numJoints())
{
    setZero();
}

void PlanningRobotState::setZero()
{
    joint_positions_.setZero();
    joint_velocities_.setZero();
}

}
