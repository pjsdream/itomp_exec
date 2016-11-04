#include <itomp_exec/robot/robot_model.h>

#include <geometric_shapes/mesh_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <ros/ros.h>


namespace itomp_exec
{

RobotModel::RobotModel()
{
}

RobotModel::RobotModel(const moveit::core::RobotModelPtr& robot_model)
{
    initWithMoveitRobotModel(robot_model);
}

RobotModel::~RobotModel()
{
}

void RobotModel::initWithMoveitRobotModel(const robot_model::RobotModelPtr& robot_model)
{
    robot_name_ = robot_model->getName();
    robot_frame_name_ = robot_model->getModelFrame();
}

}
