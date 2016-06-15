#ifndef ITOMP_EXEC_JOINT_STATE_LISTENER_H
#define ITOMP_EXEC_JOINT_STATE_LISTENER_H


#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map>

#include <boost/thread/mutex.hpp>


namespace itomp_exec
{

// listen joint states from topic "/joint_state"
class JointStateListener
{
public:

    JointStateListener();
    ~JointStateListener();

    sensor_msgs::JointState getJointStates();

private:

    void jointStateCallbackFunction(const sensor_msgs::JointStateConstPtr& msg);

    boost::mutex joint_state_mutex_;
    sensor_msgs::JointState current_state_;
    std::map<std::string, int> joint_name_index_map_;

    ros::Subscriber joint_state_subscriber_;
    ros::CallbackQueue callback_queue_;

    ros::AsyncSpinner spinner_;
};

}


#endif // ITOMP_EXEC_JOINT_STATE_LISTENER_H
