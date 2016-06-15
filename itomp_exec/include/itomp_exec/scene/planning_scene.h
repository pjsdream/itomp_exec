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

    PlanningScene(const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ~PlanningScene();

    inline void enableFutureDynamicObstacles()
    {
        use_future_dynamic_obstacles_ = true;
    }

    inline void disableFutureDynamicObstacles()
    {
        use_future_dynamic_obstacles_ = false;
    }

    void addStaticObstacle(const std::string& mesh_filename, const Eigen::Affine3d& transformation);
    void addStaticObstacles(const std::vector<std::string>& mesh_filenames, const std::vector<Eigen::Affine3d>& transformations);
    void addStaticSphereObstacle(const Eigen::Vector3d& position, double radius);
    
    Spheres getStaticSphereObstacles() const;
    Spheres getDynamicSphereObstacles() const;

    void setVisualizationTopic(const std::string& topic);
    void visualizeScene();

private:

    // static obstacles
    std::vector<shapes::Shape*> shapes_;
    std::vector<std::string> mesh_filenames_;
    std::vector<Eigen::Affine3d> transformations_;

    // dynamic obstacles
    bool use_future_dynamic_obstacles_;
    mutable pcml::FutureObstacleListener future_obstacle_listener_;

    // transform listener for obtaining camera link frame
    tf::TransformListener transform_listener_;

    // for visualization
    ros::NodeHandle node_handle_;
    ros::Publisher static_scene_publisher_;
};

typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
typedef std::shared_ptr<const PlanningScene> PlanningSceneConstPtr;

}


#endif // ITOMP_EXEC_PLANNING_SCENE_H
