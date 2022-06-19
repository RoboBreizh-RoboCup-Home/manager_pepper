#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <pnp_ros/names.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 

#include "PlanHighLevelActions/OtherPlanActions.hpp"
#include "ManagerUtils.hpp"
#include "SQLiteUtils.hpp"


using namespace std;

namespace robobreizh
{
namespace other
{
namespace plan
{
    void aGPSRProcessOrders(string params, bool* run)
    {
        RoboBreizhManagerUtils::setPNPConditionStatus("nextOrderGiveMeCup");
        *run = 1;
    }

    void aCheckForMoreGuests(string params, bool* run)
    {
        std_msgs::Int32 number_guests_to_welcome, number_guests_welcomed;
        // NOT SUPPOSED TO BE HERE - Increment number_guests_welcomed
        SQLiteUtils::getParameterValue<std_msgs::Int32>("param_receptionist_number_of_guests_welcomed", number_guests_welcomed);
        number_guests_welcomed.data++;
        bool ret = SQLiteUtils::modifyParameterParameter<std_msgs::Int32>("param_receptionist_number_of_guests_welcomed", number_guests_welcomed);
        // This is a temporary function, will be replaced for a more dynamic one
        bool is_value_available = false;
        
        SQLiteUtils::getParameterValue<std_msgs::Int32>("param_receptionist_number_of_guests_to_welcome", number_guests_to_welcome);
        SQLiteUtils::getParameterValue<std_msgs::Int32>("param_receptionist_number_of_guests_welcomed", number_guests_welcomed);

        ROS_INFO("aCheckForMoreGuests - Number of guests to welcome = %d", number_guests_to_welcome.data);
        ROS_INFO("aCheckForMoreGuests - Number of guests welcomed = %d", number_guests_welcomed.data);

        if (number_guests_to_welcome.data > number_guests_welcomed.data)
            RoboBreizhManagerUtils::setPNPConditionStatus("MoreGuestToWelcome");
        else
            RoboBreizhManagerUtils::setPNPConditionStatus("NoMoreGuestToWelcome");
        *run = 1;
    }
} // namespace plan
} // namespace other
}// namespace robobreizh