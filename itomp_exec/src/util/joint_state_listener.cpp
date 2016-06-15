#include <itomp_exec/util/joint_state_listener.h>



namespace itomp_exec
{

void JointStateListener::jointStateCallbackFunction(const sensor_msgs::JointStateConstPtr& msg)
{
    boost::mutex::scoped_lock lock(joint_state_mutex_);

    for (int i=0; i<msg->name.size(); i++)
    {
        std::map<std::string, int>::iterator it = joint_name_index_map_.find(msg->name[i]);

        if (it == joint_name_index_map_.end())
        {
            const int idx = joint_name_index_map_.size();
            joint_name_index_map_[ msg->name[i] ] = idx;
            current_state_.name.push_back(msg->name[i]);

            if (msg->position.empty())
                current_state_.position.clear();
            else
                current_state_.position.push_back(msg->position[i]);

            if (msg->velocity.empty())
                current_state_.velocity.clear();
            else
                current_state_.velocity.push_back(msg->velocity[i]);

            if (msg->effort.empty())
                current_state_.effort.clear();
            else
                current_state_.effort.push_back(msg->effort[i]);
        }
        else
        {
            const int idx = it->second;

            if (!msg->position.empty())
                current_state_.position[idx] = msg->position[i];

            if (!msg->velocity.empty())
                current_state_.velocity[idx] = msg->velocity[i];

            if (!msg->effort.empty())
                current_state_.effort[idx] = msg->effort[i];
        }
    }
}

JointStateListener::JointStateListener()
    : spinner_(1, &callback_queue_)
{
    ros::NodeHandle node_handle;

    ros::SubscribeOptions options;
    options.template init<sensor_msgs::JointState>("/joint_states", 1, std::bind(&JointStateListener::jointStateCallbackFunction, this, std::placeholders::_1));
    options.allow_concurrent_callbacks = true;
    options.callback_queue = &callback_queue_;

    joint_state_subscriber_ = node_handle.subscribe(options);

    spinner_.start();
}

JointStateListener::~JointStateListener()
{
}

sensor_msgs::JointState JointStateListener::getJointStates()
{
    boost::mutex::scoped_lock lock(joint_state_mutex_);
    return current_state_;
}

}
