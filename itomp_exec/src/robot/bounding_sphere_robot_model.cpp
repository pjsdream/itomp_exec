#include <itomp_exec/robot/bounding_sphere_robot_model.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>


namespace itomp_exec
{

#if 0
BoundingSphereRobotModel::AABB BoundingSphereRobotModel::getAABB(const shapes::Mesh* mesh)
{
    AABB aabb;

    const int num_vertices = mesh->vertex_count;
    const double* vertices = mesh->vertices;

    aabb.x[0] = vertices[0];
    aabb.x[1] = vertices[0];
    aabb.y[0] = vertices[1];
    aabb.y[1] = vertices[1];
    aabb.z[0] = vertices[2];
    aabb.z[1] = vertices[2];

    for (int i=1; i<num_vertices; i++)
    {
        const double x = vertices[3*i];
        const double y = vertices[3*i+1];
        const double z = vertices[3*i+2];

        if (aabb.x[0] > x) aabb.x[0] = x;
        if (aabb.x[1] < x) aabb.x[1] = x;
        if (aabb.y[0] > y) aabb.y[0] = y;
        if (aabb.y[1] < y) aabb.y[1] = y;
        if (aabb.z[0] > z) aabb.z[0] = z;
        if (aabb.z[1] < z) aabb.z[1] = z;
    }

    return aabb;
}

BoundingSphereRobotModel::AABB BoundingSphereRobotModel::getAABB(const shapes::Box* box)
{
    AABB aabb;

    aabb.x[0] = -box->size[0]/2.;
    aabb.x[1] =  box->size[0]/2.;
    aabb.y[0] = -box->size[1]/2.;
    aabb.y[1] =  box->size[1]/2.;
    aabb.z[0] = -box->size[2]/2.;
    aabb.z[1] =  box->size[2]/2.;

    return aabb;
}

BoundingSphereRobotModel::BoundingSphereRobotModel()
{
}

void BoundingSphereRobotModel::initFromMoveitRobotModel(robot_model::RobotModelConstPtr robot_model)
{
    RobotModel::initFromMoveitRobotModel(robot_model);

    // compute bounding spheres
    link_collision_spheres_.resize(link_collision_shapes_.size());
    for (int i=0; i<link_collision_shapes_.size(); i++)
    {
        for (int j=0; j<link_collision_shapes_[i].size(); j++)
        {
            shapes::ShapeConstPtr shape = link_collision_shapes_[i][j];

            switch (shape->type)
            {
            case shapes::MESH:
            {
                const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape.get());
                AABB aabb = getAABB(mesh);
                addCollisionSpheresFromAABB(aabb, link_collision_transforms_[i][j], link_collision_spheres_[i]);
            }
                break;

            case shapes::BOX:
            {
                const shapes::Box* box = dynamic_cast<const shapes::Box*>(shape.get());
                AABB aabb = getAABB(box);
                addCollisionSpheresFromAABB(aabb, link_collision_transforms_[i][j], link_collision_spheres_[i]);
            }
                break;

            default:
                ROS_WARN("Undefined behavior for collision shape type [%d]", shape->type);
            }
        }
    }

    attached_spheres_.resize(num_joints_);
}

void BoundingSphereRobotModel::addCollisionSpheresFromAABBAlongXAxis(const AABB& aabb, const Eigen::Affine3d& transform, Spheres& spheres)
{
    const double lengths[3] =
    {
        aabb.x[1] - aabb.x[0],
        aabb.y[1] - aabb.y[0],
        aabb.z[1] - aabb.z[0],
    };

    Sphere sphere;
    sphere.radius = std::sqrt(lengths[1] * lengths[1] + lengths[2] * lengths[2]) / 2.;

    const double y = (aabb.y[0] + aabb.y[1]) / 2.;
    const double z = (aabb.z[0] + aabb.z[1]) / 2.;

    const int cnt = lengths[0] / (2. * sphere.radius) + 1;

    if (cnt == 1)
    {
        sphere.position = Eigen::Vector3d((aabb.x[0] + aabb.x[1]) / 2., y, z);
        spheres.push_back(sphere);
    }
    else
    {
        const double x0 = aabb.x[0] + sphere.radius;
        const double x1 = aabb.x[1] - sphere.radius;

        for (int i=0; i<cnt; i++)
        {
            const double t = (double)i / (cnt - 1);
            const double x = (1. - t) * x0 + t * x1;

            sphere.position = Eigen::Vector3d(x, y, z);
            spheres.push_back(sphere);
        }
    }
}

void BoundingSphereRobotModel::addCollisionSpheresFromAABB(const AABB& aabb, const Eigen::Affine3d& transform, Spheres& spheres)
{
    const double lengths[3] =
    {
        aabb.x[1] - aabb.x[0],
        aabb.y[1] - aabb.y[0],
        aabb.z[1] - aabb.z[0],
    };

    // bounding sphere with one sphere
    /*
    Sphere sphere;
    sphere.radius = std::sqrt(lengths[0] * lengths[0] + lengths[1] * lengths[1] + lengths[2] * lengths[2]) / 2.;
    sphere.position = transform * Eigen::Vector3d(
                (aabb.x[0] + aabb.x[1]) / 2.0,
                (aabb.y[0] + aabb.y[1]) / 2.0,
                (aabb.z[0] + aabb.z[1]) / 2.0
            );
            */

    // bounding sphere with multiple sphere
    if (lengths[0] >= lengths[1] && lengths[0] >= lengths[2])
    {
        addCollisionSpheresFromAABBAlongXAxis(aabb, transform, spheres);
    }
    else if (lengths[1] >= lengths[0] && lengths[1] >= lengths[2])
    {
        AABB rotated_aabb;
        rotated_aabb.x[0] = aabb.y[0];
        rotated_aabb.x[1] = aabb.y[1];
        rotated_aabb.y[0] = aabb.z[0];
        rotated_aabb.y[1] = aabb.z[1];
        rotated_aabb.z[0] = aabb.x[0];
        rotated_aabb.z[1] = aabb.x[1];

        const int num_spheres = spheres.size();
        addCollisionSpheresFromAABBAlongXAxis(rotated_aabb, transform, spheres);

        for (int i=num_spheres; i<spheres.size(); i++)
        {
            const Eigen::Vector3d position = spheres[i].position;
            spheres[i].position = Eigen::Vector3d(position(2), position(0), position(1));
        }
    }
    else
    {
        AABB rotated_aabb;
        rotated_aabb.x[0] = aabb.z[0];
        rotated_aabb.x[1] = aabb.z[1];
        rotated_aabb.y[0] = aabb.x[0];
        rotated_aabb.y[1] = aabb.x[1];
        rotated_aabb.z[0] = aabb.y[0];
        rotated_aabb.z[1] = aabb.y[1];

        const int num_spheres = spheres.size();
        addCollisionSpheresFromAABBAlongXAxis(rotated_aabb, transform, spheres);

        for (int i=num_spheres; i<spheres.size(); i++)
        {
            const Eigen::Vector3d position = spheres[i].position;
            spheres[i].position = Eigen::Vector3d(position(1), position(2), position(0));
        }
    }
}

void BoundingSphereRobotModel::attachSphere(int link_index, const Eigen::Vector3d& position, double radius)
{
    Sphere sphere;
    sphere.position = position;
    sphere.radius = radius;

    attached_spheres_[link_index].push_back(sphere);
}

void BoundingSphereRobotModel::detachSpheres(int link_index)
{
    attached_spheres_[link_index].clear();
}

void BoundingSphereRobotModel::getCollisionSpheres(const std::vector<Eigen::Affine3d>& link_transforms, std::vector<Spheres>& spheres) const
{
    spheres.resize(num_joints_);

    for (int i=0; i<link_collision_spheres_.size(); i++)
    {
        spheres[i].clear();

        // collision spheres
        for (int j=0; j<link_collision_spheres_[i].size(); j++)
        {
            const Sphere& sphere = link_collision_spheres_[i][j];

            Sphere transformed_sphere;
            transformed_sphere.radius = sphere.radius;
            transformed_sphere.position = link_transforms[i] * sphere.position;

            spheres[i].push_back(transformed_sphere);
        }

        // attached spheres
        for (int j=0; j<attached_spheres_[i].size(); j++)
        {
            const Sphere& sphere = attached_spheres_[i][j];

            Sphere transformed_sphere;
            transformed_sphere.radius = sphere.radius;
            transformed_sphere.position = link_transforms[i] * sphere.position;

            spheres[i].push_back(transformed_sphere);
        }
    }
}

void BoundingSphereRobotModel::pushVisualLinkVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const
{
    RobotModel::pushVisualLinkVisualizationMarkers(link_transforms, ns, msg);
    pushAttachedSpheresVisualizationMarkers(link_transforms, ns, msg);
}

void BoundingSphereRobotModel::pushCollisionSpheresVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 1.;
    marker.color.a = 1.;

    marker.id = 0;
    for (int i=0; i<link_collision_spheres_.size(); i++)
    {
        for (int j=0; j<link_collision_spheres_[i].size(); j++)
        {
            const Sphere& sphere = link_collision_spheres_[i][j];

            marker.scale.x = sphere.radius * 2.;
            marker.scale.y = sphere.radius * 2.;
            marker.scale.z = sphere.radius * 2.;

            const Eigen::Affine3d& transform = link_transforms[i];
            const Eigen::Vector3d& position = transform * sphere.position;
            const Eigen::Quaterniond q(transform.linear());

            tf::pointEigenToMsg(position, marker.pose.position);
            tf::quaternionEigenToMsg(q, marker.pose.orientation);

            msg.markers.push_back(marker);
            marker.id++;
        }
    }

    pushAttachedSpheresVisualizationMarkers(link_transforms, ns, msg);
}

void BoundingSphereRobotModel::pushAttachedSpheresVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const
{
    // attached spheres
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 1.;
    marker.color.a = 1.;

    marker.id = msg.markers.size();

    for (int i=0; i<attached_spheres_.size(); i++)
    {
        for (int j=0; j<attached_spheres_[i].size(); j++)
        {
            const Sphere& sphere = attached_spheres_[i][j];
            Eigen::Affine3d transform = link_transforms[i];
            transform.translate(sphere.position);

            marker.scale.x = sphere.radius * 2.;
            marker.scale.y = sphere.radius * 2.;
            marker.scale.z = sphere.radius * 2.;

            tf::poseEigenToMsg(transform, marker.pose);

            msg.markers.push_back(marker);
            marker.id++;
        }
    }
}
#endif

}
