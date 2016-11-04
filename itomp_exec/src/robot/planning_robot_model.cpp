#include <itomp_exec/robot/planning_robot_model.h>

#include <geometric_shapes/mesh_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <ros/ros.h>


namespace itomp_exec
{

void PlanningRobotLinkGroup::setJointOriginTransform(const Eigen::Affine3d& transform)
{
    joint_origin_transform_ = transform;
}

void PlanningRobotLinkGroup::addLink(const robot_model::LinkModel* link, const Eigen::Affine3d& transform)
{
    links_.push_back(link);
    link_transforms_.push_back(transform);
}

void PlanningRobotLinkGroup::addChildJoint(const PlanningRobotJoint* child_joint)
{
    child_joints_.push_back(child_joint);
}


PlanningRobotJoint::PlanningRobotJoint(const moveit::core::JointModel *joint, const PlanningRobotLinkGroup *parent_link_group, const PlanningRobotLinkGroup *child_link_group)
    : joint_(joint)
    , parent_link_group_(parent_link_group)
    , child_link_group_(child_link_group)
{
}


PlanningRobotModel::PlanningRobotModel(const robot_model::RobotState& robot_state, const std::string& planning_group)
    : PlanningRobotModel( robot_state, robot_state.getRobotModel()->getJointModelGroup(planning_group)->getJointModelNames() )
{
}

PlanningRobotModel::PlanningRobotModel(const robot_model::RobotState& robot_state, const std::vector<std::string>& joint_names)
    : robot_default_state_(robot_state)
{
    robot_model_ = robot_state.getRobotModel();
    joint_names_ = joint_names;

    initialize();
}

PlanningRobotModel::~PlanningRobotModel()
{
}

void PlanningRobotModel::initialize()
{
    link_groups_.push_back(PlanningRobotLinkGroup());
    link_groups_.back().setJointOriginTransform( robot_model_->getRootLink()->getJointOriginTransform() );
    initializeTraverse(robot_model_->getRootLink(), &link_groups_[0], Eigen::Affine3d::Identity());
}

void PlanningRobotModel::initializeTraverse(const moveit::core::LinkModel *link, PlanningRobotLinkGroup* link_group, Eigen::Affine3d transform)
{
    link_group->addLink(link, transform);

    const std::vector<const robot_model::JointModel*>& child_joints = link->getChildJointModels();

    for (int i=0; i<child_joints.size(); i++)
    {
        const robot_model::JointModel* child_joint = child_joints[i];

        if (std::find(joint_names_.begin(), joint_names_.end(), child_joint->getName()) == joint_names_.end())
        {
            Eigen::Affine3d transform_joint;
            child_joint->computeTransform(robot_default_state_.getJointPositions(child_joint), transform_joint);
            Eigen::Affine3d child_transform = transform * child_joint->getChildLinkModel()->getJointOriginTransform() * transform_joint;

            initializeTraverse(child_joint->getChildLinkModel(), link_group, child_transform);
        }

        else
        {
            link_groups_.push_back(PlanningRobotLinkGroup());
            link_groups_.back().setJointOriginTransform( transform * link->getJointOriginTransform() );
            joints_.push_back(PlanningRobotJoint(child_joint, link_group, &link_groups_.back()));
            link_groups_.back().addChildJoint(&joints_.back());
            initializeTraverse(child_joint->getChildLinkModel(), &link_groups_.back(), Eigen::Affine3d::Identity());
        }
    }
}

int PlanningRobotModel::numJoints()
{
    return joints_.size();
}

}
