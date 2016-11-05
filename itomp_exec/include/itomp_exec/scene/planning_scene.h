#ifndef ITOMP_EXEC_PLANNING_SCENE_H
#define ITOMP_EXEC_PLANNING_SCENE_H


#include <geometric_shapes/shapes.h>
#include <itomp_exec/shape/sphere.h>
#include <pcml/util/future_obstacle_listener.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_listener.h>


namespace itomp_exec
{

class PlanningScene
{
public:

    PlanningScene();
    ~PlanningScene();

    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation);
    void addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations);

private:

    // static obstacles
    std::vector<shapes::Shape*> shapes_;
    std::vector<std::string> mesh_filenames_;
    std::vector<Eigen::Affine3d> transformations_;
};

}


#endif // ITOMP_EXEC_PLANNING_SCENE_H
