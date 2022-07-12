#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include "GenericActions/OtherGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace other
{
namespace generic
{
    bool waitForGoSignal()
    {
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::String const> shared_msg;
        std_msgs::String msg;
        ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/manager/go");

        shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);

        if (shared_msg != NULL)
        {
            msg = *shared_msg;
            ROS_INFO("waitForGoSignal - Let's go! %s", msg.data.c_str());
            return true;
        }
        else
        {
            ROS_INFO("waitForGoSignal - ERROR");
            return false;
        }
    }
    

     bool isValidObject(string objName){

        std::vector<string> objects;
        objects.push_back("Water");
        objects.push_back("Milk");
        objects.push_back("Coke");
        objects.push_back("Tonic");
        objects.push_back("Bubble Tea");
        objects.push_back("Ice tea");
        objects.push_back("Cloth");
        objects.push_back("Sponge");
        objects.push_back("Cleaner");
        objects.push_back("Corn Flakes");
        objects.push_back("Tuna Can");
        objects.push_back("Sugger");
        objects.push_back("Mustard");
        objects.push_back("Apple");
        objects.push_back("Peach");
        objects.push_back("Orange");
        objects.push_back("Banana");
        objects.push_back("Strawberry");
        objects.push_back("Pockys");
        objects.push_back("Pringles");
        objects.push_back("Spoon");
        objects.push_back("Fork");
        objects.push_back("Plate");
        objects.push_back("Bowl");
        objects.push_back("Mug");
        objects.push_back("Knife");

         for (auto obj: objects) {
             std::string lowerObj = boost::to_upper_copy(obj);
             std::string lowerobjName = boost::to_upper_copy(objName);
             bool found = boost::algorithm::contains(lowerObj, lowerobjName);
            if(found ){
                return true;
            }
        }
        return false;

     }


    bool isValidPlaces(string placeName)
    {

        std::vector<string> Places;
        Places.push_back("House Plant");        
        Places.push_back("Coat Rack");
        Places.push_back("Sofa");
        Places.push_back("Couch Table");
        Places.push_back("TV");
        Places.push_back("Side Table");
        Places.push_back("Book Shelf");
        Places.push_back("Pantry");
        Places.push_back("Dinner Table");
        Places.push_back("Kitchen Bin");
        Places.push_back("Fridge");
        Places.push_back("Washing Machine");
        Places.push_back("Sink");
        Places.push_back("Small Shelf");
        Places.push_back("Cupboard");
        Places.push_back("Big Shelf");
        Places.push_back("Bed");
        Places.push_back("Desk");
        Places.push_back("Show Rack");
        Places.push_back("Bin");
        Places.push_back("Office Shelf");

         for (auto place: Places) {
             std::string lowerPlace = boost::to_upper_copy(place);
             std::string lowerPlaceName = boost::to_upper_copy(placeName);
             bool found = boost::algorithm::contains(lowerPlace, lowerPlaceName);
            if(found ){
                return true;
            }
        }
        return false;
     }




} // namespace generic
} // namespace other
} // namespace robobreizh