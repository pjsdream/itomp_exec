#ifndef ITOMP_EXEC_BOUNDING_SPHERE_ROBOT_MODEL_H
#define ITOMP_EXEC_BOUNDING_SPHERE_ROBOT_MODEL_H


#include <itomp_exec/robot/robot_model.h>
#include <itomp_exec/shape/sphere.h>
#include <geometric_shapes/shapes.h>


namespace itomp_exec
{

class BoundingSphereRobotModel : public RobotModel
{
private:

    struct AABB
    {
        double x[2];
        double y[2];
        double z[2];
    };

    static AABB getAABB(const shapes::Mesh* mesh);
    static AABB getAABB(const shapes::Box* mesh);

public:

    BoundingSphereRobotModel();

    virtual void initFromMoveitRobotModel(robot_model::RobotModelConstPtr robot_model);

    void attachSphere(int link_index, const Eigen::Vector3d& position, double radius);
    void detachSpheres(int link_index);

    /// returns all spheres
    void getCollisionSpheres(const std::vector<Eigen::Affine3d>& link_transforms, std::vector<Spheres>& spheres) const;

    // visualization
    virtual void pushVisualLinkVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const;
    void pushCollisionSpheresVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const;
    void pushAttachedSpheresVisualizationMarkers(const std::vector<Eigen::Affine3d>& link_transforms, const std::string& ns, visualization_msgs::MarkerArray& msg) const;

private:

    void addCollisionSpheresFromAABB(const AABB& aabb, const Eigen::Affine3d& transform, Spheres& spheres);
    std::vector<Spheres> link_collision_spheres_; //!< collision spheres in link local coordinates

    // attached bodies
    std::vector<std::vector<Sphere> > attached_spheres_;
};

typedef std::shared_ptr<BoundingSphereRobotModel> BoundingSphereRobotModelPtr;
typedef std::shared_ptr<const BoundingSphereRobotModel> BoundingSphereRobotModelConstPtr;

}


#endif // ITOMP_EXEC_BOUNDING_SPHERE_ROBOT_MODEL_H
