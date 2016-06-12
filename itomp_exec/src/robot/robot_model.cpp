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

RobotModel::~RobotModel()
{
    for (int i=0; i<link_visual_meshes_.size(); i++)
        delete link_visual_meshes_[i];
}

void RobotModel::initFromMoveitRobotModel(robot_model::RobotModelConstPtr robot_model)
{
    frame_id_ = robot_model->getModelFrame();

    // allocate some arrays
    const std::vector<const robot_model::JointModel*>& joint_models = robot_model->getJointModels();
    num_joints_ = joint_models.size();
    joint_axes_.resize(Eigen::NoChange, num_joints_);
    joint_limits_.resize(num_joints_);
    joint_names_.resize(num_joints_);
    children_joint_indices_.resize(num_joints_);
    parent_joint_indices_.resize(num_joints_);
    joint_default_positions_.resize(num_joints_);
    joint_default_velocities_.resize(num_joints_);
    joint_default_positions_.setZero();
    joint_default_velocities_.setZero();
    
    // initialize joints in arrays
    root_joint_index_ = initializeJointsRecursive( robot_model->getRootJoint() );
    parent_joint_indices_[root_joint_index_] = -1;
    
    // initialize planning groups
    const std::vector<const robot_model::JointModelGroup*>& planning_group_joint_models = robot_model->getJointModelGroups();
    const int num_planning_group_joints = planning_group_joint_models.size();
    
    for (int i=0; i<num_planning_group_joints; i++)
    {
        const robot_model::JointModelGroup* joint_model_group = planning_group_joint_models[i];
        const std::string& group_name = joint_model_group->getName();
        
        planning_group_index_map_[ group_name ] = i;
        planning_group_joint_indices_.push_back(std::vector<int>());
        
        const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getJointModels();
        for (int j=0; j<joint_models.size(); j++)
        {
            const robot_model::JointModel* joint_model = joint_models[j];
            const std::string& joint_name = joint_model->getName();

            planning_group_joint_indices_[i].push_back( joint_index_map_[joint_name] );
        }
    }
}

int RobotModel::initializeJointsRecursive(const moveit::core::JointModel *joint_model)
{
    const int joint_index = joint_types_.size();
    
    const std::string& joint_name = joint_model->getName();
    joint_index_map_[ joint_name ] = joint_index;
    joint_names_[ joint_index ] = joint_name;
    
    //joint_default_positions_(joint_index) = ;
    
    if (joint_model->getVariableCount() >= 1)
    {
        const robot_model::VariableBounds bounds = joint_model->getVariableBounds( joint_model->getVariableNames()[0] );
        joint_limits_[joint_index].min_position = bounds.min_position_;
        joint_limits_[joint_index].max_position = bounds.max_position_;
        joint_limits_[joint_index].min_velocity = bounds.min_velocity_;
        joint_limits_[joint_index].max_velocity = bounds.max_velocity_;
    }
    
    switch (joint_model->getType())
    {
    case robot_model::JointModel::FIXED:
    {
        joint_types_.push_back(FIXED);
    }
        break;
        
    case robot_model::JointModel::REVOLUTE:
    {
        joint_types_.push_back(REVOLUTE);
        
        const robot_model::RevoluteJointModel* revolute_joint_model = dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model);
        joint_axes_.col(joint_index) = revolute_joint_model->getAxis();
    }
        break;
        
    case robot_model::JointModel::PRISMATIC:
    {
        joint_types_.push_back(PRISMATIC);
        
        const robot_model::PrismaticJointModel* prismatic_joint_model = dynamic_cast<const  robot_model::PrismaticJointModel*>(joint_model);
        joint_axes_.col(joint_index) = prismatic_joint_model->getAxis();
    }
        break;
        
    default:
        ROS_ERROR("RobotModel: Undefined initialization for joint type [%s]", joint_model->getTypeName().c_str());
    }
    
    // links
    const robot_model::LinkModel* link_model = joint_model->getChildLinkModel();
    
    // link name
    link_index_map_[ link_model->getName() ] = joint_index;
    
    // link origin
    link_origin_transforms_.push_back( link_model->getJointOriginTransform() );
    
    // link visual mesh
    const std::string& visual_mesh_filename = link_model->getVisualMeshFilename();
    link_visual_mesh_filenames_.push_back(visual_mesh_filename);

    const Eigen::Affine3d& visual_mesh_transform = link_model->getVisualMeshOrigin();
    const Eigen::Vector3d& visual_mesh_scale = link_model->getVisualMeshScale();
    shapes::Mesh* mesh = 0;
    if (!visual_mesh_filename.empty())
        mesh = shapes::createMeshFromResource(visual_mesh_filename, visual_mesh_scale);
    link_visual_meshes_.push_back(mesh);
    link_visual_transforms_.push_back(visual_mesh_transform);
    
    // link collision shapes
    link_collision_shapes_.push_back( link_model->getShapes() );
    link_collision_transforms_.push_back( link_model->getCollisionOriginTransforms() );
    
    // recurse
    const std::vector<const robot_model::JointModel*>& children = joint_model->getChildLinkModel()->getChildJointModels();
    for (int i=0; i<children.size(); i++)
    {
        const int child_joint_index = initializeJointsRecursive(children[i]);
        children_joint_indices_[ joint_index ].push_back(child_joint_index);
        parent_joint_indices_[ child_joint_index ] = joint_index;
    }
    
    return joint_index;
}

std::vector<int> RobotModel::getDescendantJointIndices(int joint_index) const
{
    std::vector<int> descendant_joint_indices;

    descendant_joint_indices.push_back(joint_index);
    for (int i=0; i<descendant_joint_indices.size(); i++)
    {
        joint_index = descendant_joint_indices[i];
        for (int j=0; j<children_joint_indices_[joint_index].size(); j++)
            descendant_joint_indices.push_back(children_joint_indices_[joint_index][j]);
    }

    return descendant_joint_indices;
}

void RobotModel::getLinkTransforms(const Eigen::VectorXd& joint_positions, std::vector<Eigen::Affine3d>& link_transforms) const
{
    link_transforms.resize(num_joints_);
    
    for (int i=0; i<num_joints_; i++)
    {
        const int parent_joint_index = parent_joint_indices_[i];
        Eigen::Affine3d joint_transform = Eigen::Affine3d::Identity();
        
        switch (joint_types_[i])
        {
        case FIXED:
            break;
            
        case REVOLUTE:
        {
            Eigen::AngleAxisd rotation(joint_positions(i), joint_axes_.col(i));
            joint_transform.rotate(rotation);
        }
            break;
            
        case PRISMATIC:
            joint_transform.translate(joint_positions[i] * joint_axes_.col(i));
            break;
        }
        
        if (i==0)
            link_transforms[i].matrix().noalias() = link_origin_transforms_[i].matrix() * joint_transform.matrix();
        else
            link_transforms[i].matrix().noalias() = link_transforms[parent_joint_index].matrix() * link_origin_transforms_[i].matrix() * joint_transform.matrix();
    }
}

void RobotModel::pushVisualLinkVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker.color.r = 0.;
    marker.color.g = 0.;
    marker.color.b = 0.;
    marker.color.a = 0.;
    marker.scale.x = 1.;
    marker.scale.y = 1.;
    marker.scale.z = 1.;

    for (int i=0; i<link_transforms.size(); i++)
    {
        if (link_visual_meshes_[i] != 0)
        {
            marker.id = i;
            marker.mesh_resource = link_visual_mesh_filenames_[i];
            marker.mesh_use_embedded_materials = true;

            Eigen::Affine3d transform = link_transforms[i] * link_visual_transforms_[i];
            Eigen::Quaterniond q(transform.linear());

            tf::pointEigenToMsg(transform.translation(), marker.pose.position);
            tf::quaternionEigenToMsg(q, marker.pose.orientation);

            msg.markers.push_back(marker);
        }
    }
}

void RobotModel::pushCollisionLinkVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const
{
    // TODO
    for (int i=0; i<link_transforms.size(); i++)
    {
    }
}

}
