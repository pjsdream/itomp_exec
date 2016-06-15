#include <itomp_exec/scene/planning_scene.h>

#include <geometric_shapes/mesh_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcml/FutureObstacleDistributions.h>

namespace itomp_exec
{

PlanningScene::PlanningScene(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
    , transform_listener_(node_handle)
{
    disableFutureDynamicObstacles();
}

PlanningScene::~PlanningScene()
{
    for (int i=0; i<shapes_.size(); i++)
        delete shapes_[i];
}

void PlanningScene::addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation)
{
    std::vector<std::string> mesh_filenames;
    mesh_filenames.push_back(mesh_filename);

    std::vector<Eigen::Affine3d> transformations;
    transformations.push_back(transformation);

    addStaticObstacles(mesh_filenames, transformations);
}

void PlanningScene::addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations)
{
    const int num_objects = mesh_filenames.size();

    for (int i=0; i<num_objects; i++)
    {
        mesh_filenames_.push_back(mesh_filenames[i]);
        shapes_.push_back(shapes::createMeshFromResource(mesh_filenames[i]));
        transformations_.push_back(transformations[i]);
    }
}

void PlanningScene::addStaticSphereObstacle(const Eigen::Vector3d& position, double radius)
{
    Eigen::Affine3d affine;
    affine.setIdentity();
    affine.translate(position);

    shapes_.push_back(new shapes::Sphere(radius));
    transformations_.push_back(affine);
}

Spheres PlanningScene::getStaticSphereObstacles() const
{
    std::vector<Sphere> spheres;
    Sphere sphere;
    
    for (int i=0; i<shapes_.size(); i++)
    {
        const shapes::Shape* shape = shapes_[i];
        
        if (shape->type == shapes::SPHERE)
        {
            const shapes::Sphere* sphere_shape = dynamic_cast<const shapes::Sphere*>(shape);
            const double radius = sphere_shape->radius;
            const Eigen::Vector3d position = transformations_[i].translation();
            
            sphere.radius = radius;
            sphere.position = position;
            
            spheres.push_back(sphere);
        }
    }
    
    return spheres;
}

Spheres PlanningScene::getDynamicSphereObstacles() const
{
    pcml::FutureObstacleDistributions obstacles = future_obstacle_listener_.getObstaclesAtCurrentTime();
    std::string frame_id = future_obstacle_listener_.getFrameId();

    Eigen::Affine3d transform;
    transform.setIdentity();

    tf::StampedTransform tf_transform;
    ros::Time time;
    std::string error_string;
    if (transform_listener_.getLatestCommonTime("base_link", frame_id, time, &error_string) != tf::NO_ERROR)
    {
        ROS_ERROR("TF error: %s", error_string.c_str());
        ROS_ERROR("Set transform to identity");
    }
    else
    {
        transform_listener_.lookupTransform("base_link", frame_id, time, tf_transform);

        tf::Point tf_translation = tf_transform.getOrigin();
        tf::Quaternion tf_quaternion = tf_transform.getRotation();

        const Eigen::Vector3d translation(tf_translation.x(), tf_translation.y(), tf_translation.z());
        const Eigen::Quaterniond quaternion(tf_quaternion.w(), tf_quaternion.x(), tf_quaternion.y(), tf_quaternion.z());
        transform.translate(translation).rotate(quaternion);
    }

    Spheres spheres;
    for (int i=0; i<obstacles.obstacles.size(); i++)
    {
        const pcml::FutureObstacleDistribution& obstacle = obstacles.obstacles[i];

        Eigen::Vector3d position;
        tf::pointMsgToEigen(obstacle.obstacle_point, position);

        const double radius = obstacle.radius;

        spheres.push_back(Sphere(radius, transform * position));
    }

    return spheres;
}

void PlanningScene::setVisualizationTopic(const std::string &topic)
{
    static_scene_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

void PlanningScene::visualizeScene()
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "base_link";
    marker.ns = "scene";
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.;
    marker.scale.y = 1.;
    marker.scale.z = 1.;
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;

    for (int i=0; i<shapes_.size(); i++)
    {
        const shapes::Shape* shape = shapes_[i];

        marker.id = i;
        tf::poseEigenToMsg(transformations_[i], marker.pose);

        switch (shape->type)
        {
        case shapes::MESH:
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.scale.x = 1.;
            marker.scale.y = 1.;
            marker.scale.z = 1.;
            marker.mesh_resource = mesh_filenames_[i];
            marker.mesh_use_embedded_materials = false;
            marker_array.markers.push_back(marker);
        }
            break;

        case shapes::SPHERE:
        {
            marker.type = visualization_msgs::Marker::SPHERE;
            const shapes::Sphere* sphere = dynamic_cast<const shapes::Sphere*>(shape);
            const double radius = sphere->radius;
            marker.scale.x = radius * 2.;
            marker.scale.y = radius * 2.;
            marker.scale.z = radius * 2.;
            marker_array.markers.push_back(marker);
        }
            break;

        default:
            ROS_WARN("Planning scene visualization for type [%d] is not supported.", shape->type);
        }
    }

    static_scene_publisher_.publish(marker_array);
}

}
