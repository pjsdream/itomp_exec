#ifndef ITOMP_EXEC_PLANNING_SCENE_H
#define ITOMP_EXEC_PLANNING_SCENE_H


#include <geometric_shapes/shapes.h>
#include <itomp_exec/shape/sphere.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


namespace itomp_exec
{

class PlanningScene
{
private:

    enum Type
    {
        Obstacle = 0,
        Object,
    };

public:

    PlanningScene();
    ~PlanningScene();

    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transform);
    void addStaticObstacle(const shapes::Shape* shape, const Eigen::Affine3d& transform);
    void addObject(const std::string& mesh_filename, const Eigen::Affine3d& transform);
    void addObject(const shapes::Shape* shape, const Eigen::Affine3d& transform);

    void visualize(ros::Publisher* publisher) const;

private:

    // shapes
    std::vector<const shapes::Shape*> shapes_;
    std::vector<std::string> mesh_filenames_;
    std::vector<Eigen::Affine3d> transforms_;
    std::vector<Type> types_;
};

}


#endif // ITOMP_EXEC_PLANNING_SCENE_H
