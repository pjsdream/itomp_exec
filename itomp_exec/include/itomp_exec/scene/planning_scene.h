#ifndef ITOMP_EXEC_PLANNING_SCENE_H
#define ITOMP_EXEC_PLANNING_SCENE_H


#include <geometric_shapes/shapes.h>
#include <Eigen/Dense>


namespace itomp_exec
{

class PlanningScene
{
public:

    PlanningScene();

    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation);
    void addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations);

private:

    std::vector<shapes::Mesh*> meshes_;
    std::vector<Eigen::Affine3d> transformations_;
};

}


#endif // ITOMP_EXEC_PLANNING_SCENE_H
