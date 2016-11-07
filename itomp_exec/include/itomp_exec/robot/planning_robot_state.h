#ifndef ITOMP_EXEC_PLANNING_ROBOT_STATE_H
#define ITOMP_EXEC_PLANNING_ROBOT_STATE_H


#include <itomp_exec/robot/planning_robot_model.h>
#include <moveit_msgs/RobotState.h>
#include <itomp_exec/shape/sphere.h>
#include <Eigen/Dense>

#include <ros/ros.h>


namespace itomp_exec
{

class PlanningRobotState
{
private:

    struct Mesh
    {
        std::string mesh_filename;
        Eigen::Affine3d transform;
    };

    typedef std::vector<Mesh> Meshes;

public:

    PlanningRobotState(const PlanningRobotModel* robot_model);

    void setZero();

    void setJointPosition(const std::string& joint_name, double position);

    Spheres getBoundingSpheres() const;
    Meshes getMeshes() const;

    void visualizeRobot(ros::Publisher* publisher, const std::string& ns) const;
    void visualizeBoundingSpheres(ros::Publisher* publisher, const std::string& ns) const;

private:
    
    const PlanningRobotModel* robot_model_;

    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;

    void getBoundingSpheresTraverse(const PlanningRobotLinkGroup* link_group, const Eigen::Affine3d& transform, Spheres& spheres) const;
    void getMeshesTraverse(const PlanningRobotLinkGroup* link_group, const Eigen::Affine3d& transform, Meshes& meshes) const;
};

}


#endif // ITOMP_EXEC_PLANNING_ROBOT_STATE_H
