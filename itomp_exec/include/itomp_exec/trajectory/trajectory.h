#ifndef ITOMP_EXEC_TRAJECTORY_H
#define ITOMP_EXEC_TRAJECTORY_H


#include <boost/shared_ptr.hpp>


namespace itomp_exec
{

class Trajectory
{
public:
    
    Trajectory();
    
private:
};

typedef boost::shared_ptr<Trajectory> TrajectoryPtr;

}


#endif // ITOMP_EXEC_TRAJECTORY_H
