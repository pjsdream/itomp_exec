#include <itomp_exec/scene/planning_scene.h>

#include <geometric_shapes/mesh_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

namespace itomp_exec
{

PlanningScene::PlanningScene()
{
}

PlanningScene::~PlanningScene()
{
    for (int i=0; i<shapes_.size(); i++)
        delete shapes_[i];
}

void PlanningScene::addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transform)
{
    mesh_filenames_.push_back(mesh_filename);
    shapes_.push_back(shapes::createMeshFromResource(mesh_filename));
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

void PlanningScene::visualize(ros::Publisher* publisher) const
{
}

}
