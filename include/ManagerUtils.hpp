#ifndef _PNP_ROBOBREIZH_MANAGER_UTILS_
#define _PNP_ROBOBREIZH_MANAGER_UTILS_

#include <std_msgs/String.h>
#include <vizbox/Story.h>
#include <string>

namespace robobreizh
{
    void pubPlanState(std::string state);

    class RoboBreizhManagerUtils
    {
    public:
        RoboBreizhManagerUtils() = default;
        ~RoboBreizhManagerUtils() = default;

        static bool isPublisherReady(const ros::Publisher& pub);

        static std::string convertCamelCaseToSpacedText(std::string params);
        template <typename T>
        static bool sendMessageToTopic(const std::string &topicPath, const T &obj)
        {
            // TODO: Add Try catch if failure
            // TODO: Add timeout
            ros::NodeHandle handle;
            ros::Publisher topicPublisher = handle.advertise<T>(topicPath, 1000);

            while (ros::ok() && !RoboBreizhManagerUtils::isPublisherReady(topicPublisher))
            {
                ROS_INFO("Waiting for publisher to be ready");
                ros::Duration(0.1).sleep();
            }

            if (ros::ok())
            {
                topicPublisher.publish(obj);
                ROS_INFO("Sending information to topic %s.", topicPath.c_str());
                topicPublisher.shutdown();
                return true;
            }
            
            return false;
        }

        template <typename T>
        static bool waitForMessageFromTopic(const std::string &topicPath, T &returnedMessage)
        {
            // TODO: Add try catch if failure
            ros::NodeHandle nh;
            boost::shared_ptr<T const> shared_msg;

            shared_msg = ros::topic::waitForMessage<std_msgs::String>(topicPath, nh);

            if (shared_msg != NULL)
            {
                returnedMessage = shared_msg;
                return true;
            }

            else
                return false;
        }


        static std::string getPNPConditionStatus();
        static bool setPNPConditionStatus(const std::string &status);
        static void pubVizBoxRobotText(const std::string &text);
        static void pubVizBoxOperatorText(const std::string &text);
        static void pubVizBoxChallengeStep(const uint &challengeStep);
        static void pubVizBoxStory(const vizbox::Story story);
    };
}// namespace robobreizh

#endif // _PNP_ROBOBREIZH_MANAGER_UTILS_
