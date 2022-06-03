#ifndef _PNP_ROBOBREIZH_MONGODB_UTILS_
#define _PNP_ROBOBREIZH_MONGODB_UTILS_

/*#include <ros/ros.h>

#include <std_msgs/String.h>

#include <mongodb_store/message_store.h>
*/
namespace robobreizh
{/*
    class SQLiteUtils
    {
    public:
        SQLiteUtils() = default;
        ~SQLiteUtils() = default;

        template <typename T>
        static bool storeNewParameter(const std::string &objectName, const T &obj)
        {
            ros::NodeHandle nh;
            mongodb_store::MessageStoreProxy messageStore(nh);
            std::string id = messageStore.insertNamed(objectName, obj);

            ROS_INFO("MongoDbUtils::storeNewParameter, parameter stored with ID %s", id.c_str());

            return true;
        }

        template <typename T>
        static bool getParameterValue(const std::string &objectName, T &value_returned)
        {
            // TODO What if multiple variables are using the same name ?
            ros::NodeHandle nh;
            mongodb_store::MessageStoreProxy messageStore(nh);
            std::vector<boost::shared_ptr<T>> results;
            messageStore.queryNamed<T>(objectName, results);
            if (results.size() > 0)
            {
                value_returned = *(results.at(0));
                return true;
            }
            else
            {
                ROS_INFO("MongoDbUtils::getParameterValue - no data found named %s", objectName.c_str());
                return false;
            }
        }

        /*template <typename T>
        static std::vector<boost::shared_ptr<T>> getAllItemsFromACertainType()
        {
            ros::NodeHandle nh;
		    mongodb_store::MessageStoreProxy messageStore(nh);

		    std::vector<boost::shared_ptr<T>> itemsFound;
		    messageStore.query<T>(itemsFound);
            return itemsFound;
        }*/
    //};
} // namespace robobreizh

#endif // _PNP_ROBOBREIZH_MONGODB_UTILS_