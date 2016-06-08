#include <itomp_exec/scene/planning_scene.h>

#include <geometric_shapes/mesh_operations.h>

namespace itomp_exec
{

PlanningScene::PlanningScene()
{
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
        meshes_.push_back(shapes::createMeshFromResource(mesh_filenames[i]));
        transformations_.push_back(transformations[i]);
    }
}

}
