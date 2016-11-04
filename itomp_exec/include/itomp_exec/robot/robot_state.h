#ifndef ITOMP_EXEC_ROBOT_STATE_H
#define ITOMP_EXEC_ROBOT_STATE_H


#include <itomp_exec/robot/robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class RobotState
{
public:

    RobotState(const RobotModel* robot_model);

private:
    
    const RobotModel* robot_model_;
};

}


#endif // ITOMP_EXEC_ROBOT_STATE_H
