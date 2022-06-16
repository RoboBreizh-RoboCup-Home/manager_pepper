#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

// ROS
#include <perception_pepper/ObjectsList.h>
#include <perception_pepper/Person.h>

// NAOQI --> Service
#include <perception_pepper/object_detection_service.h>
#include <perception_pepper/person_features_detection_service.h>
#include <perception_pepper/PersonList.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/VisionGenericActions.hpp"
#include "DatabaseModel/VisionModel.hpp"

using namespace std;

bool USE_NAOQI_NO_ROS = true;

namespace robobreizh
{
	namespace vision
	{
		namespace generic
		{
			bool waitForHuman()
			{
				ros::NodeHandle nh;

				if (USE_NAOQI_NO_ROS == false)
				{

					// ------- Send order --------
					ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/robobreizh/manager/give_order/detect_object", 1000);
					ros::Rate loop_rate(10);

					ros::Time start_time = ros::Time::now();
					ros::Duration timeout(2.0); // Timeout of 2 seconds

					while (ros::Time::now() - start_time < timeout)
					{
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

				else if (USE_NAOQI_NO_ROS == true)
				{

					ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");

					perception_pepper::object_detection_service srv;

					vector<string> detections;
					detections.push_back("Human face");
					detections.push_back("Human body");
					detections.push_back("Woman");
					detections.push_back("Person");
					detections.push_back("Boy");
					detections.push_back("Girl");
					detections.push_back("Human head");

					vector<std_msgs::String> tabMsg;

					for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++)
					{
						std_msgs::String msg;
						std::stringstream ss;
						ss << *t;
						msg.data = ss.str();
						tabMsg.push_back(msg);
					}

					srv.request.entries_list = tabMsg;

					if (client.call(srv))
					{
						perception_pepper::ObjectsList objList = srv.response.outputs_list;
						vector<perception_pepper::Object> objects = objList.objects_list;
						int nbObjects = objects.size();
						ROS_INFO("WaitForHuman OK %d", nbObjects);

						for (int i = 0; i < nbObjects; i++)
						{
							perception_pepper::Object obj = objects[i];
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
						if (nbObjects == 0)
							return false;
						else
							return true;
					}
					else
					{
						ROS_INFO("WaitForHuman OK  - ERROR");
						return false;
					}
				}
				return false;
			}


			bool findObject(std::string objectName)
			{
			
				ros::NodeHandle nh;
					ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");

					perception_pepper::object_detection_service srv;

					vector<string> detections;
					detections.push_back(objectName);

					vector<std_msgs::String> tabMsg;

					for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++)
					{
						std_msgs::String msg;
						std::stringstream ss;
						ss << *t;
						msg.data = ss.str();
						tabMsg.push_back(msg);
					}

					srv.request.entries_list = tabMsg;

					if (client.call(srv))
					{
						perception_pepper::ObjectsList objList = srv.response.outputs_list;
						vector<perception_pepper::Object> objects = objList.objects_list;
						int nbObjects = objects.size();
						ROS_INFO("findObject OK %d", nbObjects);

						for (int i = 0; i < nbObjects; i++)
						{
							perception_pepper::Object obj = objects[i];
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
						if (nbObjects == 0)
							return false;
						else
							return true;
					}
					else
					{
						ROS_INFO("findObject OK  - ERROR");
						return false;
					}
			
			
			
				// bool is probably not the right output type, a position seems more relevant
				return true;
			}


			bool FindEmptySeat()
			{
			
			
				ros::NodeHandle nh;
			
					ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/seat_detection_service");

					perception_pepper::object_detection_service srv;

					vector<string> detections;
					detections.push_back("SEAT_INFORMATION");

					vector<std_msgs::String> tabMsg;

					for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++)
					{
						std_msgs::String msg;
						std::stringstream ss;
						ss << *t;
						msg.data = ss.str();
						tabMsg.push_back(msg);
					}

					srv.request.entries_list = tabMsg;

					if (client.call(srv))
					{
						perception_pepper::ObjectsList objList = srv.response.outputs_list;
						vector<perception_pepper::Object> objects = objList.objects_list;
						int nbObjects = objects.size();
						ROS_INFO("FindEmptySeat OK %d", nbObjects);

						for (int i = 0; i < nbObjects; i++)
						{
							perception_pepper::Object obj = objects[i];
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
						if (nbObjects == 0)
							return false;
						else
							return true;
					}
					else
					{
						ROS_INFO("FindEmptySeat OK  - ERROR");
						return false;
					}
			
			
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

			bool findHumanAndStoreFeatures()
			{

				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_service>("/robobreizh/perception_pepper/person_features_detection_service");

                // std_msgs/String[] entries_list
				perception_pepper::person_features_detection_service srv;

				vector<std::string> detections;
				detections.push_back("Human face");
				detections.push_back("Human body");
				detections.push_back("Woman");
				detections.push_back("Person");
				detections.push_back("Boy");
				detections.push_back("Girl");
				detections.push_back("Human head");

				vector<std_msgs::String> tabMsg;

				for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++)
				{
					std_msgs::String msg;
					std::stringstream ss;
					ss << *t;
					msg.data = ss.str();
					tabMsg.push_back(msg);
				}

				srv.request.entries_list = tabMsg;

				if (client.call(srv))
				{
                    // perception_pepper/PersonList outputs_list
					perception_pepper::PersonList persList = srv.response.outputs_list;

                    /* std_msgs/String name */
                    /* std_msgs/String clothes_color */
                    /* std_msgs/String age */
                    /* std_msgs/String gender */
                    /* std_msgs/String skin_color */
                    /* std_msgs/Float32 distance */
					vector<perception_pepper::Person> persons = persList.person_list;
					int nbPersons = persons.size();
					ROS_INFO("findHumanAndStoreFeatures OK, with nbPerson ==  %d", nbPersons);

					for (int i = 0; i < nbPersons; i++)
					{
						perception_pepper::Person pers = persons[i];
						std_msgs::String name = pers.name;
						std_msgs::String gender = pers.gender;
						std_msgs::String age = pers.age;
						std_msgs::String skin_color = pers.skin_color;
						double distance = pers.distance;
						std_msgs::String age = pers.age;
						std_msgs::String clothes_color = pers.clothes_color;
						double distance = pers.distance;
						geometry_msgs::Point32 coord = pers.coord;

                        if (clothes_color.data != "" || gender.data != "" || skin_color.data != "" || age.data != ""){
                            ROS_INFO("...got personne : %s", name.data.c_str());
                            ROS_INFO("            clothes_color : %s", clothes_color.data.c_str());
                            ROS_INFO("            age : %s", age.data.c_str());
                            ROS_INFO("            gender : %s", gender.data.c_str());
                            ROS_INFO("            skin_color : %s", skin_color.data.c_str());
                            ROS_INFO("            distance : %f", distance);
                            ROS_INFO("            x : %f", coord.x);
                            ROS_INFO("            y : %f", coord.y);
                            ROS_INFO("            z : %f", coord.z);
                            robobreizh::database::VisionModel vm;
                            vm.createPersonFromFeatures(gender.data, age.data, clothes_color.data, skin_color.data);
                        }

					}
					if (nbPersons == 0)
						return false;
					else
						return true;
				}
				else
				{
					ROS_INFO("findHumanAndStoreFeatures OK  - ERROR");
					return false;
				}
				return false;
			}

		} // namespace generic
	}	  // namespace vision
} // namespace robobreizh
