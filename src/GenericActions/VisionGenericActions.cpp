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
#include <perception_pepper/person_features_detection_service2.h>
#include <perception_pepper/person_features_detection_posture.h>
#include <perception_pepper/PersonList.h>
#include <manipulation_pepper/EmptySrv.h>

#include <boost/thread/thread.hpp>

#include "GenericActions/VisionGenericActions.hpp"
#include "DatabaseModel/VisionModel.hpp"

using namespace std;



namespace robobreizh
{
	namespace vision
	{
		namespace generic
		{
		

			bool waitForHuman()
			{
				ros::NodeHandle nh;

				ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");

				perception_pepper::object_detection_service srv;

				vector<string> detections;
				// coco
				detections.push_back("person");
				//OID
				detections.push_back("Human face");
				detections.push_back("Human body");
				detections.push_back("Human head");
				detections.push_back("Human arm");
				detections.push_back("Human hand");
				detections.push_back("Human nose");
				detections.push_back("Person");
				detections.push_back("Man");
				detections.push_back("Woman");
				detections.push_back("Boy");
				detections.push_back("Girl");

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

			bool findHumanAndStoreFeatures(robobreizh::Person* person)
			{

				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_service>("/robobreizh/perception_pepper/person_features_detection_service");

                // std_msgs/String[] entries_list
				perception_pepper::person_features_detection_service srv;

				vector<std::string> detections;
					detections.push_back("Human face");
					detections.push_back("Human body");
					detections.push_back("Human head");
					detections.push_back("Human arm");
					detections.push_back("Human hand");
					detections.push_back("Human nose");
					detections.push_back("Person");
					detections.push_back("Man");
					detections.push_back("Woman");
					detections.push_back("Boy");
					detections.push_back("Girl");


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
                    bool isAdded = false;
					ROS_INFO("findHumanAndStoreFeatures OK, with nbPerson ==  %d", nbPersons);

					for (int i = 0; i < nbPersons; i++)
					{
						perception_pepper::Person pers = persons[i];
						std_msgs::String name = pers.name;
						std_msgs::String gender = pers.gender;
						std_msgs::String age = pers.age;
						std_msgs::String skin_color = pers.skin_color;
						double distance = pers.distance;
						std_msgs::String clothes_color = pers.clothes_color;
						geometry_msgs::Point32 coord = pers.coord;

                        ROS_INFO("...got personne : %s", name.data.c_str());
                        ROS_INFO("            clothes_color : %s", clothes_color.data.c_str());
                        ROS_INFO("            age : %s", age.data.c_str());
                        ROS_INFO("            gender : %s", gender.data.c_str());
                        ROS_INFO("            skin_color : %s", skin_color.data.c_str());
                        ROS_INFO("            distance : %f", distance);
                        ROS_INFO("            x : %f", coord.x);
                        ROS_INFO("            y : %f", coord.y);
                        ROS_INFO("            z : %f", coord.z);

                        if (clothes_color.data != ""){
                            person->cloth_color = clothes_color.data;
                        }
                        if (age.data != ""){
                            person->age = age.data;
                        }
                        if (gender.data != ""){
                            person->gender = gender.data;
                        }
                        if (skin_color.data != ""){
                            person->skin_color = skin_color.data;
                        }
                        std::cout << person->age << ", " << person->cloth_color << ", " << person->gender << ", " << person->skin_color << std::endl;

                        if (person->cloth_color!= "" && person->skin_color!= "" && person->age!= "" && person->gender != ""){
                            ROS_INFO("...adding person to db");
                            robobreizh::database::VisionModel vm;
                            vm.createPersonFromFeatures(person->gender, person->age, person->cloth_color, person->skin_color);
                            ros::ServiceClient moveHead = nh.serviceClient<manipulation_pepper::EmptySrv>("robobreizh/manipulation/look_down");
                            manipulation_pepper::EmptySrv emptySrv;
                            moveHead.call(emptySrv);
                            isAdded = true;
                        }

					}
					if (!isAdded)
						return false;
					else
						return true;
				}
				else
				{
					ROS_INFO("findHumanAndStoreFeatures - ERROR");
					return false;
				}
				return false;
			}

			bool isInRadius(float x1,float y1,float z1,float x2,float y2,float z2,float epsilon){
				float distance = std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2,2) + std::pow(z1-z2,2));
				std::cout << "	Calculated distance : " << std::to_string(distance) << std::endl;
				if (distance < epsilon ){
					return true;
					std::cout << "	distance is smaller than " << std::to_string(epsilon) << std::endl;
				}
				return false;
			}

			bool addObjectToDatabase(robobreizh::Object obj){
				robobreizh::database::VisionModel vm;
				// get all objects with label
				std::vector<robobreizh::Object> dbObjects = vm.getObjectsByLabel(obj.label);

				// loop over dbObjects
				bool alreadyExist = false;
				for (auto dbObj : dbObjects){
					if (isInRadius(dbObj.pos_x,dbObj.pos_y,dbObj.pos_z,obj.pos_x,obj.pos_y,obj.pos_z,0.2)){
						alreadyExist = true;
					}
				}

				if(!alreadyExist){
					vm.createObject(obj);
					return true;
				}
				return false;
			}
			
			bool findStoreAllObjects(){
				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");
				perception_pepper::object_detection_service srv;
				vector<std::string> detections; 
				detections.push_back("ALL");

				vector<std_msgs::String> tabMsg;

				for (auto t = detections.begin(); t != detections.end(); t++)
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
					perception_pepper::ObjectsList objectList = srv.response.outputs_list;

					vector<perception_pepper::Object> objects= objectList.objects_list;
					int nbObjects = objects.size();
					ROS_INFO("findStoreObjects OK, with objects ==  %d", nbObjects);

					if (nbObjects == 0) {
						return false;
					}

					for (auto obj : objects)
					{
						robobreizh::Object objStruct;
						objStruct.label = obj.label.data;
						objStruct.color = obj.color.data; 
						objStruct.distance = obj.distance;
						// TODO : convertion into the frame map
						geometry_msgs::Point32 coord = obj.coord;
						objStruct.pos_x = coord.x;
						objStruct.pos_y = coord.y;
						objStruct.pos_z = coord.z;
						ROS_INFO("...got obj: %s", objStruct.label.c_str());
						ROS_INFO("            color : %s", objStruct.color.c_str());
						ROS_INFO("            distance: %f", objStruct.distance);
						ROS_INFO("            x : %f", coord.x);
						ROS_INFO("            y : %f", coord.y);
						ROS_INFO("            z : %f", coord.z);

						if (addObjectToDatabase(objStruct)){
							ROS_INFO("...added object to db");
						}
					}
					return true;
				}
				else
				{
					ROS_INFO("findHumanAndStoreFeaturesWihDistanceFilter - ERROR");
					return false;
				}
				return true;
			}

			bool addPersonToDatabase(robobreizh::Person person){
				robobreizh::database::VisionModel vm;
				auto allPerson = vm.getAllPerson();
				// loop over allPerson 
				bool alreadyExist = false;
				for (auto dbPerson: allPerson){
					if (isInRadius(dbPerson.pos_x,dbPerson.pos_y,dbPerson.pos_z,person.pos_x,person.pos_y,person.pos_z,0.2)){
						alreadyExist = true;
					}
				}

				if(!alreadyExist){
					vm.createPerson(person);
					return true;
				}

				return false;
			}

			bool findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax)
			{

				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>("/robobreizh/perception_pepper/person_features_detection_posture");

				perception_pepper::person_features_detection_posture srv;

				vector<std::string> detections;
				detections.push_back("Human face");
				detections.push_back("Human body");
				detections.push_back("Human head");
				detections.push_back("Human arm");
				detections.push_back("Human hand");
				detections.push_back("Human nose");
				detections.push_back("Person");
				detections.push_back("Man");
				detections.push_back("Woman");
				detections.push_back("Boy");
				detections.push_back("Girl");
				

				vector<std_msgs::String> tabMsg;

				for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++)
				{
					std_msgs::String msg;
					std::stringstream ss;
					ss << *t;
					msg.data = ss.str();
					tabMsg.push_back(msg);
				}


				double distanceMax_mess = distanceMax;

				srv.request.entries_list.obj = tabMsg;
				srv.request.entries_list.distanceMaximum = distanceMax_mess;

				if (client.call(srv))
				{
					perception_pepper::PersonList persList = srv.response.outputs_list;
					perception_pepper::Person_poseList persPoseList = srv.response.outputs_pose_list;

					vector<perception_pepper::Person> persons = persList.person_list;
					vector<perception_pepper::Person_pose> personPoses = persPoseList.person_pose_list;
					int nbPersons = persons.size();
					bool isAdded = false;
					ROS_INFO("findHumanAndStoreFeaturesWithDistanceFilter OK, with nbPerson ==  %d", nbPersons);

					for (int i = 0; i < nbPersons; i++)
					{
                        robobreizh::Person person;

                        //message perception_pepper::Person 
						perception_pepper::Person pers = persons[i];
						person.gender = pers.gender.data;
                        person.age = pers.age.data;
                        person.skin_color = pers.skin_color.data;
						person.distance = (float)pers.distance;
                        person.cloth_color = pers.clothes_color.data;

                        //message perception_pepper::Person_pose
						perception_pepper::Person_pose persPose = personPoses[i];
                        person.posture = persPose.posture.data;
                        person.height = persPose.height;

						geometry_msgs::Point32 coord = pers.coord;
                        person.pos_x = coord.x; 
                        person.pos_y = coord.y; 
                        person.pos_z = coord.z; 

                        ROS_INFO("...got personne %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position (%f,%f,%f)", i,person.cloth_color.c_str(),person.age.c_str(),person.gender.c_str(),person.skin_color.c_str(),person.posture.c_str(),person.height,person.distance,person.pos_x,person.pos_y,person.pos_z);

						if (addPersonToDatabase(person)){
							ROS_INFO("...adding person to db");
							isAdded = true;
						}
					}
					if (!isAdded)
						return false;
					else
						return true;
				}
				else
				{
					ROS_INFO("findHumanAndStoreFeaturesWihDistanceFilter - ERROR");
					return false;
				}
				return true;
			}
			
			

		} // namespace generic
	}	  // namespace vision
} // namespace robobreizh
