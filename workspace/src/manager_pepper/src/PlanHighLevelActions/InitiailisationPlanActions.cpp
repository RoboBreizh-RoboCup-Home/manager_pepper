#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"
#include "MongoDbUtils.hpp"

#include <sstream>
#include <cassert>

using namespace geometry_msgs;
using namespace mongodb_store;
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
    /*std_msgs::Int32 param_current_order;
    param_current_order.data = 2;
    MongoDbUtils::storeNewParameter<std_msgs::Int32>("param_gpsr_i_current_order", param_current_order);
    ROS_INFO("Okay, all good now :)");

    bool is_value_available;
    std_msgs::Int32 value = MongoDbUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_current_order", is_value_available);
    ROS_INFO("aInitGPSR - Value acquired");

    if (is_value_available)
        ROS_INFO("aInitGPSR - Value of %s : %d", "param_gpsr_i_current_order", value.data);
    else
        ROS_INFO("aInitGPSR - No value found for %s", "param_gpsr_i_current_order");*/
    ros::NodeHandle nh;

    //Create object which does the work for us.
    MessageStoreProxy messageStore(nh);

    //This is the message we want to store
    Pose p;

    string name("my pose");
    //Insert something with a name, storing id too
    string id(messageStore.insertNamed(name, p));
    cout<<"Pose \""<<name<<"\" inserted with id "<<id<<endl;

    p.position.z = 666;
    messageStore.updateID(id, p);

    // now test it worked
    assert(messageStore.queryID<Pose>(id).first->position.z == 666);

    vector< boost::shared_ptr<Pose> > results;

    //Get it back, by default get one
    if(messageStore.queryNamed<Pose>(name, results)) {

            BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
            {
                    ROS_INFO_STREAM("Got by name: " << *p);
            }
    }

    results.clear();
    if(messageStore.queryID<Pose>(id, results)) {

            BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
            {
                    ROS_INFO_STREAM("Got by ID: " << *p);
            }
    }

    p.position.x = 999;
    messageStore.updateNamed(name, p);

    results.clear();
    // try to get it back with an incorrect name, so get None instead
    messageStore.queryNamed<Pose>("my favourite position", results);
    BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
    {
            ROS_INFO_STREAM("Got: " << *p);
    }

    results.clear();
    // get all poses, should show updated named position
    messageStore.query<Pose>(results);
    BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
    {
            ROS_INFO_STREAM("Got: " << *p);
    }

    messageStore.deleteID(id);

    results.clear();
    if(messageStore.queryID<Pose>(id, results)) {

            BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
            {
                    ROS_INFO_STREAM("Got by ID: " << *p);
            }

    }
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
