#include <itomp_exec/robot/planning_robot_state.h>

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>


namespace itomp_exec
{

PlanningRobotState::PlanningRobotState(const PlanningRobotModel* robot_model)
    : robot_model_(robot_model)
    , joint_positions_(robot_model->numJoints())
    , joint_velocities_(robot_model->numJoints())
{
    setZero();
}

void PlanningRobotState::setZero()
{
    joint_positions_.setZero();
    joint_velocities_.setZero();
}

void PlanningRobotState::setJointPosition(const std::string& joint_name, double position)
{
    joint_positions_[ robot_model_->getJointIndex(joint_name) ] = position;
}

Spheres PlanningRobotState::getBoundingSpheres() const
{
    Spheres spheres;
    getBoundingSpheresTraverse( robot_model_->getRootLinkGroup(), Eigen::Affine3d::Identity(), spheres);
    return spheres;
}

void PlanningRobotState::getBoundingSpheresTraverse(const PlanningRobotLinkGroup* link_group, const Eigen::Affine3d& transform, Spheres& spheres) const
{
    Spheres link_group_spheres = link_group->getBoundingVolumes();
    for (int i=0; i<link_group_spheres.size(); i++)
    {
        Sphere sphere = link_group_spheres[i];
        sphere.position = transform * sphere.position;
        spheres.push_back(sphere);
    }

    const std::vector<const PlanningRobotJoint*>& child_joints = link_group->getChildJoints();

    for (int i=0; i<child_joints.size(); i++)
    {
        const PlanningRobotJoint* child_joint = child_joints[i];
        const PlanningRobotLinkGroup* child_link_group = child_joint->getChildLinkGroup();

        Eigen::Affine3d joint_transform;
        child_joint->getJoint()->computeTransform(&joint_positions_[child_joint->getJointIndex()], joint_transform);
        Eigen::Affine3d new_transform = transform * child_link_group->getJointOriginTransform() * joint_transform;

        getBoundingSpheresTraverse(child_link_group, new_transform, spheres);
    }
}

PlanningRobotState::Meshes PlanningRobotState::getMeshes() const
{
    Meshes meshes;
    getMeshesTraverse( robot_model_->getRootLinkGroup(), Eigen::Affine3d::Identity(), meshes);
    return meshes;
}

void PlanningRobotState::getMeshesTraverse(const PlanningRobotLinkGroup* link_group, const Eigen::Affine3d& transform, Meshes& meshes) const
{
    std::vector<std::pair<std::string, Eigen::Affine3d> > link_meshes = link_group->getMeshes();
    for (int i=0; i<link_meshes.size(); i++)
    {
        Mesh mesh;
        mesh.mesh_filename = link_meshes[i].first;
        mesh.transform = transform * link_meshes[i].second;
        meshes.push_back(mesh);
    }

    const std::vector<const PlanningRobotJoint*>& child_joints = link_group->getChildJoints();

    for (int i=0; i<child_joints.size(); i++)
    {
        const PlanningRobotJoint* child_joint = child_joints[i];
        const PlanningRobotLinkGroup* child_link_group = child_joint->getChildLinkGroup();

        Eigen::Affine3d joint_transform;
        child_joint->getJoint()->computeTransform(&joint_positions_[child_joint->getJointIndex()], joint_transform);
        Eigen::Affine3d new_transform = transform * child_link_group->getJointOriginTransform() * joint_transform;

        getMeshesTraverse(child_link_group, new_transform, meshes);
    }
}

void PlanningRobotState::visualizeRobot(ros::Publisher* publisher, const std::string& ns) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = 1.;
    marker.scale.y = 1.;
    marker.scale.z = 1.;

    marker.color.r = 0.;
    marker.color.g = 0.;
    marker.color.b = 0.;
    marker.color.a = 0.;

    std::vector<Mesh> meshes = getMeshes();
    for (int i=0; i<meshes.size(); i++)
    {
        marker.id = i;

        marker.mesh_resource = meshes[i].mesh_filename;
        tf::poseEigenToMsg(meshes[i].transform, marker.pose);

        marker_array.markers.push_back(marker);
    }

    publisher->publish(marker_array);
}

void PlanningRobotState::visualizeBoundingSpheres(ros::Publisher* publisher, const std::string& ns) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = 1.;
    marker.color.g = 0.;
    marker.color.b = 0.;
    marker.color.a = 1.;

    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;

    Spheres spheres = getBoundingSpheres();
    for (int i=0; i<spheres.size(); i++)
    {
        marker.id = i;

        marker.scale.x = marker.scale.y = marker.scale.z = spheres[i].radius * 2.;
        marker.pose.position.x = spheres[i].position(0);
        marker.pose.position.y = spheres[i].position(1);
        marker.pose.position.z = spheres[i].position(2);

        marker_array.markers.push_back(marker);
    }

    publisher->publish(marker_array);
}

}
