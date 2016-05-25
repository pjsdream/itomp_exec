#include <itomp_exec/planner/itomp_planner_node.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_itomp");
    
    ros::NodeHandle nh("~");
    itomp_exec::ITOMPPlannerNode planner(nh);
    
    planner.loadParams();
    planner.printParams();
    
    return 0;
}
