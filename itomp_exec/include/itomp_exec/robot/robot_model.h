#ifndef ITOMP_EXEC_ROBOT_MODEL_H
#define ITOMP_EXEC_ROBOT_MODEL_H


#include <moveit/robot_model/robot_model.h>


namespace itomp_exec
{

class RobotModel
{
public:

public:

    RobotModel();

    void initFromMoveitRobotModel(robot_model::RobotModelConstPtr robot_model);

private:
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;

}


#endif // ITOMP_EXEC_ROBOT_MODEL_H
