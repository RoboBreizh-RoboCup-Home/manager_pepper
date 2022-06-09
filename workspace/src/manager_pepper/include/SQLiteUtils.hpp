#ifndef _PNP_ROBOBREIZH_SQLITE_UTILS_
#define _PNP_ROBOBREIZH_SQLITE_UTILS_

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>


namespace robobreizh
{
    class SQLiteUtils
    {
    public:
        SQLiteUtils() = default;
        ~SQLiteUtils() = default;

        static warehouse_ros_sqlite::DatabaseConnection* conn_;

        template <typename T>
        static bool storeNewParameter(const std::string &objectName, const T &obj)
        {  
            if (conn_ != nullptr)
            {
                if (conn_->isConnected())
                {
                    warehouse_ros_sqlite::DatabaseConnection* dbConn = conn_;
                    auto coll = dbConn->openCollection<T>("main", ros::message_traits::DataType<T>::value());
                    auto objMeta = coll.createMetadata();
                    objMeta->append("name", objectName);
                    coll.insert(obj, objMeta);
                    ROS_INFO("SQLiteUtils::storeNewParameter - Parameter %s stored", objectName.c_str());
                    return true;
                }
                
                else
                {
                    ROS_INFO("SQLiteUtils::storeNewParameter - SQLite database is unavailable");
                    return false;
                }
                    
            }

            ROS_INFO("SQLiteUtils::storeNewParameter - SQLite database is unavailable");
            return false;
        }

        template <typename T>
        static bool getParameterValue(const std::string &objectName, T &value_returned)
        {
            // TODO: What happens if database is not correctly initialised or if nullptr?

            if (conn_ != nullptr)
            {
                if (conn_->isConnected())
                {
                    warehouse_ros_sqlite::DatabaseConnection* dbConn = conn_;
                    auto coll = dbConn->openCollection<T>("main", ros::message_traits::DataType<T>::value());
                    auto query = coll.createQuery();
                    query->append("name", objectName);
                    const auto results = coll.queryList(query);
                    if (results.size() > 0)
                    {
                        value_returned = *(results.at(0));
                        return true;
                    }

                    else
                    {
                        ROS_INFO("SQLiteUtils::getParameterValue - no data found named %s", objectName.c_str());
                        return false;
                    }
                }

                else
                {
                    ROS_INFO("SQLiteUtils::getParameterValue - SQLite database is unavailable");
                    return false;
                }
                    
            }

            ROS_INFO("SQLiteUtils::getParameterValue - SQLite database is unavailable");
            return false;
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
    };
} // namespace robobreizh

#endif // _PNP_ROBOBREIZH_SQLITE_UTILS_