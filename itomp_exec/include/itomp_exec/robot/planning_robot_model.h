#ifndef ITOMP_EXEC_PLANNING_ROBOT_MODEL_H
#define ITOMP_EXEC_PLANNING_ROBOT_MODEL_H


#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <itomp_exec/shape/sphere.h>
#include <geometric_shapes/shapes.h>


namespace itomp_exec
{

class PlanningRobotJoint;

class PlanningRobotLinkGroup
{
public:

    void setJointOriginTransform(const Eigen::Affine3d& transform);

    void addLink(const robot_model::LinkModel* link, const Eigen::Affine3d& transform);
    void addChildJoint(const PlanningRobotJoint* child_joint);

    const Spheres& getBoundingVolumes() const;
    std::vector<std::pair<std::string, Eigen::Affine3d> > getMeshes() const;
    const Eigen::Affine3d& getJointOriginTransform() const;

    const std::vector<const PlanningRobotJoint*>& getChildJoints() const;

private:

    std::vector<const PlanningRobotJoint*> child_joints_;

    Eigen::Affine3d joint_origin_transform_;

    std::vector<const robot_model::LinkModel*> links_;
    EigenSTL::vector_Affine3d link_transforms_;

    // bounding spheres, taking into account link transformations
    Spheres getLinkBoundingSpheres(const robot_model::LinkModel* link);
    Spheres getMeshBoundingSpheres(const shapes::Mesh* mesh);
    Spheres bounding_spheres_;
};


class PlanningRobotJoint
{
public:

    PlanningRobotJoint(int joint_index, const robot_model::JointModel* joint, const PlanningRobotLinkGroup* parent_link_group, const PlanningRobotLinkGroup* child_link_group);

    const robot_model::JointModel* getJoint() const;
    int getJointIndex() const;
    const PlanningRobotLinkGroup* getChildLinkGroup() const;

private:

    const robot_model::JointModel* joint_;

    int joint_index_;

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
    int getJointIndex(const std::string& joint_name) const;

    const PlanningRobotLinkGroup* getRootLinkGroup() const;

protected:

    void initialize();
    void initializeTraverse(const robot_model::LinkModel* link, PlanningRobotLinkGroup* link_group, Eigen::Affine3d transform);

    robot_model::RobotState robot_default_state_;
    robot_model::RobotModelConstPtr robot_model_;
    std::vector<std::string> joint_names_;

    std::vector<PlanningRobotLinkGroup*> link_groups_;
    std::vector<PlanningRobotJoint*> joints_;
    std::vector<JointLimit> joint_limits_;
};

}


#endif // ITOMP_EXEC_PLANNING_ROBOT_MODEL_H
