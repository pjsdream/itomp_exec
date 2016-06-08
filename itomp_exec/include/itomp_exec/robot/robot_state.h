#ifndef ITOMP_EXEC_ROBOT_STATE_H
#define ITOMP_EXEC_ROBOT_STATE_H


#include <itomp_exec/robot/robot_model.h>


namespace itomp_exec
{

class RobotState
{
public:

    RobotState();
    
    inline void setRobotModel(RobotModelConstPtr& robot_model)
    {
        robot_model_ = robot_model;
    }

private:
    
    RobotModelConstPtr robot_model_;
};

}


#endif // ITOMP_EXEC_ROBOT_STATE_H
