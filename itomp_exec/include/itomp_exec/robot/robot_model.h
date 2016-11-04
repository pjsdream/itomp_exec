#ifndef ITOMP_EXEC_ROBOT_MODEL_H
#define ITOMP_EXEC_ROBOT_MODEL_H


#include <moveit/robot_model/robot_model.h>


namespace itomp_exec
{

class RobotModel
{
public:
    
    struct JointLimit
    {
        double min_position;
        double max_position;
        double min_velocity;
        double max_velocity;
    };

public:

    RobotModel();
    RobotModel(const moveit::core::RobotModelPtr& robot_model);
    ~RobotModel();

    void initWithMoveitRobotModel(const robot_model::RobotModelPtr& robot_model);

private:

    std::string robot_name_;
    std::string robot_frame_name_; // tf root link name
};

}


#endif // ITOMP_EXEC_ROBOT_MODEL_H
