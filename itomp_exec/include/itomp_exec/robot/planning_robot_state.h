#ifndef ITOMP_EXEC_PLANNING_ROBOT_STATE_H
#define ITOMP_EXEC_PLANNING_ROBOT_STATE_H


#include <itomp_exec/robot/planning_robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class PlanningRobotState
{
public:

    PlanningRobotState(const PlanningRobotModel* robot_model);

    void setZero();

private:
    
    const PlanningRobotModel* robot_model_;

    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
};

}


#endif // ITOMP_EXEC_PLANNING_ROBOT_STATE_H
