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

    Spheres link_bounding_spheres = getLinkBoundingSpheres(link);
    for (int i=0; i<link_bounding_spheres.size(); i++)
    {
        Sphere& sphere = link_bounding_spheres[i];
        sphere.position = transform * sphere.position;
        bounding_spheres_.push_back(sphere);
    }
}

void PlanningRobotLinkGroup::addChildJoint(const PlanningRobotJoint* child_joint)
{
    child_joints_.push_back(child_joint);
}

const std::vector<const PlanningRobotJoint*>& PlanningRobotLinkGroup::getChildJoints() const
{
    return child_joints_;
}

Spheres PlanningRobotLinkGroup::getLinkBoundingSpheres(const moveit::core::LinkModel *link)
{
    Spheres bounding_spheres;

    const EigenSTL::vector_Affine3d& collision_origin_transforms = link->getCollisionOriginTransforms();
    const std::vector<shapes::ShapeConstPtr>& collision_shapes = link->getShapes();

    for (int i=0; i<collision_shapes.size(); i++)
    {
        const shapes::Shape* shape = collision_shapes[i].get();

        switch (shape->type)
        {
        case shapes::MESH:
            const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape);
            Spheres mesh_bounding_spheres = getMeshBoundingSpheres(mesh);
            for (int j=0; j<mesh_bounding_spheres.size(); j++)
            {
                Sphere sphere = mesh_bounding_spheres[j];
                sphere.position = collision_origin_transforms[i] * sphere.position;
                bounding_spheres.push_back(sphere);
            }
            break;
        }
    }

    return bounding_spheres;
}

Spheres PlanningRobotLinkGroup::getMeshBoundingSpheres(const shapes::Mesh* mesh)
{
    // compute AABB
    double x[6];
    x[0] = x[1] = mesh->vertices[0];
    x[2] = x[3] = mesh->vertices[1];
    x[4] = x[5] = mesh->vertices[2];

    for (int i=1; i<mesh->vertex_count; i++)
    {
        if (x[0] > mesh->vertices[3*i+0]) x[0] = mesh->vertices[3*i+0];
        if (x[1] < mesh->vertices[3*i+0]) x[1] = mesh->vertices[3*i+0];
        if (x[2] > mesh->vertices[3*i+1]) x[2] = mesh->vertices[3*i+1];
        if (x[3] < mesh->vertices[3*i+1]) x[3] = mesh->vertices[3*i+1];
        if (x[4] > mesh->vertices[3*i+2]) x[4] = mesh->vertices[3*i+2];
        if (x[5] < mesh->vertices[3*i+2]) x[5] = mesh->vertices[3*i+2];
    }

    double s[3] = {x[1] - x[0], x[3] - x[2], x[5] - x[4]};
    if (s[0] > s[1]) std::swap(s[0], s[1]);
    if (s[1] > s[2]) std::swap(s[1], s[2]);
    if (s[0] > s[1]) std::swap(s[0], s[1]);

    const double r = s[1] / ((int)(s[1] / s[0])) / 2.;
    const double d = 2. * r;

    int n[3] = {std::ceil((x[1] - x[0]) / d), std::ceil((x[3] - x[2]) / d), std::ceil((x[5] - x[4]) / d)};
    Spheres spheres;
    for (int i=0; i<n[0]; i++)
    {
        for (int j=0; j<n[1]; j++)
        {
            for (int k=0; k<n[2]; k++)
            {
                const double a = n[0] == 1 ? (x[0] + x[1]) / 2. : (x[0] + r) + i * (x[1] - x[0] - d) / (n[0] - 1);
                const double b = n[1] == 1 ? (x[2] + x[3]) / 2. : (x[2] + r) + j * (x[3] - x[2] - d) / (n[1] - 1);
                const double c = n[2] == 1 ? (x[4] + x[5]) / 2. : (x[4] + r) + k * (x[5] - x[4] - d) / (n[2] - 1);
                Sphere sphere;
                sphere.radius = r;
                sphere.position = Eigen::Vector3d(a, b, c);
                spheres.push_back(sphere);
            }
        }
    }

    return spheres;
}

const Spheres& PlanningRobotLinkGroup::getBoundingVolumes() const
{
    return bounding_spheres_;
}

std::vector<std::pair<std::string, Eigen::Affine3d> > PlanningRobotLinkGroup::getMeshes() const
{
    std::vector<std::pair<std::string, Eigen::Affine3d> > meshes;

    for (int i=0; i<links_.size(); i++)
    {
        std::pair<std::string, Eigen::Affine3d> mesh;
        const robot_model::LinkModel* link = links_[i];

        mesh.first = link->getVisualMeshFilename();
        if (mesh.first != "")
        {
            Eigen::Affine3d mesh_transform = link->getVisualMeshOrigin();
            mesh_transform.scale( link->getVisualMeshScale() );
            mesh.second = link_transforms_[i] * mesh_transform;

            meshes.push_back(mesh);
        }
    }

    return meshes;
}

const Eigen::Affine3d& PlanningRobotLinkGroup::getJointOriginTransform() const
{
    return joint_origin_transform_;
}


PlanningRobotJoint::PlanningRobotJoint(int joint_index, const moveit::core::JointModel *joint, const PlanningRobotLinkGroup *parent_link_group, const PlanningRobotLinkGroup *child_link_group)
    : joint_index_(joint_index)
    , joint_(joint)
    , parent_link_group_(parent_link_group)
    , child_link_group_(child_link_group)
{
}

const robot_model::JointModel* PlanningRobotJoint::getJoint() const
{
    return joint_;
}

int PlanningRobotJoint::getJointIndex() const
{
    return joint_index_;
}

const PlanningRobotLinkGroup* PlanningRobotJoint::getChildLinkGroup() const
{
    return child_link_group_;
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

int PlanningRobotModel::getJointIndex(const std::string& joint_name) const
{
    for (int i=0; i<joints_.size(); i++)
    {
        if (joints_[i]->getJoint()->getName() == joint_name)
            return i;
    }

    return -1;
}

void PlanningRobotModel::initialize()
{
    link_groups_.push_back(new PlanningRobotLinkGroup());
    link_groups_.back()->setJointOriginTransform( robot_model_->getRootLink()->getJointOriginTransform() );
    initializeTraverse(robot_model_->getRootLink(), link_groups_[0], Eigen::Affine3d::Identity());

    // joint limits
    for (int i=0; i<joints_.size(); i++)
    {
        const robot_model::VariableBounds& bound = joints_[i]->getJoint()->getVariableBounds( joints_[i]->getJoint()->getVariableNames()[0] );

        JointLimit joint_limit;

        joint_limit.min_position = bound.min_position_;
        joint_limit.max_position = bound.max_position_;
        joint_limit.min_velocity = bound.min_velocity_;
        joint_limit.max_velocity = bound.max_velocity_;

        joint_limits_.push_back(joint_limit);
    }
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
            link_groups_.push_back(new PlanningRobotLinkGroup());
            link_groups_.back()->setJointOriginTransform( transform * child_joint->getChildLinkModel()->getJointOriginTransform() );
            joints_.push_back(new PlanningRobotJoint(joints_.size(), child_joint, link_group, link_groups_.back()));
            link_group->addChildJoint(joints_.back());
            initializeTraverse(child_joint->getChildLinkModel(), link_groups_.back(), Eigen::Affine3d::Identity());
        }
    }
}

int PlanningRobotModel::numJoints() const
{
    return joints_.size();
}

const std::vector<PlanningRobotModel::JointLimit>& PlanningRobotModel::getJointLimits() const
{
    return joint_limits_;
}

const PlanningRobotLinkGroup* PlanningRobotModel::getRootLinkGroup() const
{
    return link_groups_[0];
}

}
