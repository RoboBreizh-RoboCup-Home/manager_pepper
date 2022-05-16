#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

// Boost
#include <boost/foreach.hpp>

// MongoDb
#include <mongodb_store/message_store.h>

// ROS
#include <perception_pepper/ObjectsList.h>

// NAOQI --> Service
#include <perception_pepper/object_detection_service.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/VisionGenericActions.hpp"
#include "MongoDbUtils.hpp"

using namespace std;

bool USE_NAOQI_NO_ROS = true;

namespace robobreizh
{
namespace vision
{
namespace generic
{
	
	bool getObjectIfAlreadyBeenFound(const string &objName, perception_pepper::Object &obj)
	{
		//std::vector<boost::shared_ptr<perception_pepper::Object>> objectsFound = MongoDbUtils::getAllItemsFromACertainType<perception_pepper::Object>();
		ros::NodeHandle nh;
		mongodb_store::MessageStoreProxy messageStore(nh);

		vector<boost::shared_ptr<perception_pepper::Object>> itemsFound;
		messageStore.query<perception_pepper::Object>(itemsFound);
		BOOST_FOREACH(boost::shared_ptr<perception_pepper::Object> item,  itemsFound)
		{
				if (item->label.data == objName)
				{
					obj = *item;
					return true;
				}
		}
		return false;
	}

    bool waitForHuman()
    {
        ros::NodeHandle nh;

        if(USE_NAOQI_NO_ROS == false) {
        
        	// ------- Send order --------
        	ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/robobreizh/manager/give_order/detect_object", 1000);
		ros::Rate loop_rate(10);
        
        	ros::Time start_time = ros::Time::now();
		ros::Duration timeout(2.0); // Timeout of 2 seconds
	
		while(ros::Time::now() - start_time < timeout) {
  	        	std_msgs::StringPtr str(new std_msgs::String);
        		str->data = "Human";
        		ROS_INFO("Sending request to object detector : %s", str->data.c_str());
			chatter_pub.publish(str);
			ros::spinOnce();
			loop_rate.sleep();
    		}	
    		
        	// ------- Wait for information --------
        	boost::shared_ptr<perception_pepper::ObjectsList const> shared_msg;
        	perception_pepper::ObjectsList msg;
        	ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/perception_pepper/object_detection");

        	shared_msg = ros::topic::waitForMessage<perception_pepper::ObjectsList>("/robobreizh/perception_pepper/object_detection", nh);

        	if (shared_msg != NULL)
        	{
        		msg = *shared_msg;     
        		ROS_INFO("WaitForHuman OK");
        		return true;
        	}
        	else
        	{
            		ROS_INFO("WaitForHuman OK  - ERROR");
            		return false;
        	}
    		
    		
    	}
    	
    	else if (USE_NAOQI_NO_ROS == true) {
    
		ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");
    	   	
 		perception_pepper::object_detection_service srv;
        	
        std_msgs::String msg;
 		std::stringstream ss;
 		ss << "Human" ;
 		msg.data = ss.str();
 	
        std_msgs::String msg2;
 		std::stringstream ss2;
 		ss2 << "Chair" ;
 		msg2.data = ss.str();
 		
 		vector<std_msgs::String> tabMsg;
 		tabMsg.push_back(msg);			// "Human"
 		tabMsg.push_back(msg2);		// "Chair"
 		
 		srv.request.entries_list = tabMsg;
 		
 		if (client.call(srv))
 		{
 			perception_pepper::ObjectsList objList = srv.response.outputs_list;
 			vector<perception_pepper::Object> objects = objList.objects_list;
 			int nbObjects = objects.size();
			ROS_INFO("WaitForHuman OK %d", nbObjects);
			
			for(int i=0; i < nbObjects; i++){
				perception_pepper::Object obj = objects[i] ;
				std_msgs::String msg3 = obj.label;
				geometry_msgs::Point32 coord = obj.coord;
				double distance = obj.distance;
				double score = obj.score;
				ROS_INFO("...got object : %s", msg3.data.c_str());
				ROS_INFO("            x : %f", coord.x);
				ROS_INFO("            y : %f", coord.y);
				ROS_INFO("            z : %f", coord.z);
				ROS_INFO("            distance : %f", distance);
				ROS_INFO("            score : %f", score);
			}
        		
			return true;
 		}
 		else
 		{
            		ROS_INFO("WaitForHuman OK  - ERROR");
            		//return false;
					return true;
 		}
    		
    	}
	return false;
    }

    bool findObject(std::string objectName)
    {
        // bool is probably not the right output type, a position seems more relevant
        return true;
    }

    bool isDoorOpened() // TODO: What if door not found => Use Enum instead (Open, closed, NotFound)
    {
        ros::NodeHandle nh;
        boost::shared_ptr<std_msgs::Float32 const> shared_msg;
        std_msgs::Float32 msg;
        ROS_INFO("wait_for_go_signal - Waiting for go signal from /robobreizh/perception_pepper/door_detection/open");

        shared_msg = ros::topic::waitForMessage<std_msgs::Float32>("/robobreizh/perception_pepper/door_detection/open", nh);

        if (shared_msg != NULL)
        {
            msg = *shared_msg;     
            ROS_INFO("Door opened at distance  %f", msg.data);

            system("rosnode kill /door_detection_node");
            return true;
        }
        else
        {
            ROS_INFO("waitForDoorSignal - ERROR");
            return false;
        }
    }

} // namespace generic
} // namespace vision
} // namespace robobreizh