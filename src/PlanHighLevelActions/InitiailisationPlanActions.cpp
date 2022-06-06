#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>

#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "ManagerUtils.hpp"

using namespace std;

namespace robobreizh
{
namespace initialisation
{
namespace plan
{

void aInitCarryMyLuggage (string params, bool* run)
{
    ROS_INFO("1.1 Carry My Luggage - initialisation done");
    *run = 1;
}

void aInitGPSR(string params, bool* run)
{
    // TODO: Add global variables initiailisation here
    ROS_INFO("1.5 General Purpose Service Robot - initialisation");

    // Test SQLite
    std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> conn_;
    ROS_INFO("1");
    conn_.reset(new warehouse_ros_sqlite::DatabaseConnection());
    ROS_INFO("2");
    conn_->setParams(":memory:", 0);
    ROS_INFO("3");
    bool ret = conn_->connect();
    ROS_INFO("4");

    if (ret)
        ROS_INFO("Connection done");
    else
        ROS_INFO("Connection failed");

    using V = geometry_msgs::Vector3;
    auto coll = conn_->openCollection<V>("main", "coll");
    auto meta1 = coll.createMetadata();
    meta1->append("x", 3);

    V v1, v2;
    v1.x = 3.0;
    v1.y = 1.0;

    coll.insert(v1, meta1);

    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 1

    v2.x = 5.0;
    v2.y = 7.0;

    auto meta2 = coll.createMetadata();
    meta2->append("x", 5);

    coll.insert(v2, meta2);

    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 2

    auto query = coll.createQuery();
    query->append("x", 3);

    const auto list = coll.queryList(query);

    ROS_INFO("List size for query x = 3 = %lu", list.size()); // Expected result = 1
    ROS_INFO("List size for query list[0] y value = %f", list[0]->y); // Expected result = 1.0

    coll.removeMessages(query);
    ROS_INFO("coll.count() = %u", coll.count()); // Expected result = 1

    RoboBreizhManagerUtils::setPNPConditionStatus("GPSRInitDone");
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
