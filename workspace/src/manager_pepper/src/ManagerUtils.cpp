#include <std_msgs/String.h>
#include <ros/ros.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 

#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{
    // Shamelessly stolen from https://github.com/ros/ros_comm/issues/286
    bool RoboBreizhManagerUtils::isPublisherReady(const ros::Publisher& pub) {
        XmlRpc::XmlRpcValue req, res, payload;
        req[0] = ros::this_node::getName();
        // For more information, check http://wiki.ros.org/ROS/Master_API
        if (!ros::master::execute("getSystemState", req, res, payload, false)) {
            return false;
        }
        const size_t kSubIndex = 1;
        const auto& subscribers = payload[kSubIndex];
        for (size_t j = 0; j < subscribers.size(); j++) {
            const auto& subscriber = subscribers[j];
            // Finds the topic which this node publishes to
            std::ostringstream topic;
            const size_t kTopicNameIndex = 0;
            const size_t kSubscribersIndex = 1;
            subscriber[kTopicNameIndex].write(topic);
            if (topic.str() == pub.getTopic()) {
            auto real_num_subscribers = subscriber[kSubscribersIndex].size();
            if (pub.getNumSubscribers() == real_num_subscribers) {
                return true;
            } else {
                return false;
            }
        }
    }

    // The publisher is ready to publish since the topic does not exist yet
    return true;
    }


    string RoboBreizhManagerUtils::getPNPConditionStatus()
    {
        // TODO: Add try catch if failure
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::String const> shared_msg;
        std_msgs::String msg;

        shared_msg = ros::topic::waitForMessage<std_msgs::String>(TOPIC_PNPCONDITION, nh);

        if (shared_msg != NULL)
        {
            return msg.data;
        }
        return "";
    }

    bool RoboBreizhManagerUtils::setPNPConditionStatus(const string &status)
    {
        ROS_INFO("Update PNP condition status to %s", status.c_str());
        std_msgs::String statusMsg;
        statusMsg.data = status;
        RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
        RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(TOPIC_PNPCONDITION, statusMsg);
        return true;
    }

    void pubPlanState(const string &state){
        string robobreizhStateTopic = "/robobreizh/manager/state";
        std_msgs::String stateMsg;
        stateMsg.data = state;
        RoboBreizhManagerUtils::sendMessageToTopic<std_msgs::String>(robobreizhStateTopic, stateMsg);
    }
}// namespace robobreizh
