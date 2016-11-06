#include <itomp_exec/scene/planning_scene.h>

#include <geometric_shapes/mesh_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <stdlib.h>

namespace itomp_exec
{

PlanningScene::PlanningScene()
{
}

PlanningScene::~PlanningScene()
{
    for (int i=0; i<shapes_.size(); i++)
    {
        if (mesh_filenames_[i] != "")
            delete shapes_[i];
    }
}

void PlanningScene::addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transform)
{
    mesh_filenames_.push_back(mesh_filename);
    shapes_.push_back(shapes::createMeshFromResource(mesh_filename));
    transforms_.push_back(transform);
    types_.push_back(Obstacle);
}

void PlanningScene::addStaticObstacle(const shapes::Shape* shape, const Eigen::Affine3d& transform)
{
    mesh_filenames_.push_back("");
    shapes_.push_back(shape);
    transforms_.push_back(transform);
    types_.push_back(Obstacle);
}

void PlanningScene::addObject(const std::string& mesh_filename, const Eigen::Affine3d& transform)
{
    mesh_filenames_.push_back(mesh_filename);
    shapes_.push_back(shapes::createMeshFromResource(mesh_filename));
    transforms_.push_back(transform);
    types_.push_back(Object);
}

void PlanningScene::addObject(const shapes::Shape* shape, const Eigen::Affine3d& transform)
{
    mesh_filenames_.push_back("");
    shapes_.push_back(shape);
    transforms_.push_back(transform);
    types_.push_back(Object);
}

void PlanningScene::visualize(ros::Publisher* publisher) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    for (int i=0; i<shapes_.size(); i++)
    {
        const shapes::Shape* shape = shapes_[i];

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;

        switch (types_[i])
        {
        case Obstacle:
            marker.ns = "obstacle_" + std::to_string(i);
            marker.color.r = 0.;
            marker.color.g = 1.;
            marker.color.b = 0.;
            marker.color.a = 1.;
            break;

        case Object:
            marker.ns = "object_" + std::to_string(i);
            marker.color.r = 0.;
            marker.color.g = 0.;
            marker.color.b = 1.;
            marker.color.a = 1.;
            break;
        }

        switch (shape->type)
        {
        case shapes::ShapeType::BOX:
            const shapes::Box* box = dynamic_cast<const shapes::Box*>(shape);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = box->size[0];
            marker.scale.y = box->size[1];
            marker.scale.z = box->size[2];
            break;
        }

        tf::poseEigenToMsg(transforms_[i], marker.pose);

        marker_array.markers.push_back(marker);
    }

    publisher->publish(marker_array);
}

}
