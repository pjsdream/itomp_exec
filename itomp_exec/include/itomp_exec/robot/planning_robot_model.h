#ifndef ITOMP_EXEC_PLANNING_ROBOT_MODEL_H
#define ITOMP_EXEC_PLANNING_ROBOT_MODEL_H


#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


namespace itomp_exec
{

class PlanningRobotJoint;

class PlanningRobotLinkGroup
{
public:

    void setJointOriginTransform(const Eigen::Affine3d& transform);

    void addLink(const robot_model::LinkModel* link, const Eigen::Affine3d& transform);
    void addChildJoint(const PlanningRobotJoint* child_joint);

private:

    Eigen::Affine3d joint_origin_transform_;

    std::vector<const robot_model::LinkModel*> links_;
    EigenSTL::vector_Affine3d link_transforms_;

    std::vector<const PlanningRobotJoint*> child_joints_;
};


class PlanningRobotJoint
{
public:

    PlanningRobotJoint(const robot_model::JointModel* joint, const PlanningRobotLinkGroup* parent_link_group, const PlanningRobotLinkGroup* child_link_group);

    const robot_model::JointModel* getJoint() const;

private:

    const robot_model::JointModel* joint_;

    const PlanningRobotLinkGroup* parent_link_group_;
    const PlanningRobotLinkGroup* child_link_group_;
};


class PlanningRobotModel
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

    PlanningRobotModel(const robot_model::RobotState& robot_state, const std::string& planning_group);
    PlanningRobotModel(const robot_model::RobotState& robot_state, const std::vector<std::string>& joint_names);
    ~PlanningRobotModel();

    int numJoints() const;

    const std::vector<JointLimit>& getJointLimits() const;

protected:

    void initialize();
    void initializeTraverse(const robot_model::LinkModel* link, PlanningRobotLinkGroup* link_group, Eigen::Affine3d transform);

    robot_model::RobotState robot_default_state_;
    robot_model::RobotModelConstPtr robot_model_;
    std::vector<std::string> joint_names_;

    std::vector<PlanningRobotLinkGroup> link_groups_;
    std::vector<PlanningRobotJoint> joints_;
    std::vector<JointLimit> joint_limits_;
};

}


#endif // ITOMP_EXEC_PLANNING_ROBOT_MODEL_H
