#ifndef ITOMP_EXEC_SPHERE_H
#define ITOMP_EXEC_SPHERE_H


#include <Eigen/Dense>


namespace itomp_exec
{

class Sphere
{
public:

    Sphere(double radius = 1., const Eigen::Vector3d& position = Eigen::Vector3d(0., 0., 0.))
        : radius(radius)
        , position(position)
    {
    }

    double radius;
    Eigen::Vector3d position;
};

typedef std::vector<Sphere> Spheres;

}


#endif // ITOMP_EXEC_SPHERE_H
