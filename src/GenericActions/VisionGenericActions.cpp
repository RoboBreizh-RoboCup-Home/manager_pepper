// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

// ROBOBREIZH
#include <perception_pepper/ObjectsList.h>
#include <perception_pepper/Person.h>

// NAOQI --> Service
#include <perception_pepper/object_detection_service.h>
#include <perception_pepper/person_features_detection_service.h>
#include <perception_pepper/person_features_detection_posture.h>
#include <perception_pepper/wave_hand_detection.h>
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

			/*******************************************************************/
			bool waitForHuman()
			{
				ros::NodeHandle nh;

				ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>("/robobreizh/perception_pepper/object_detection_service");

				perception_pepper::object_detection_service srv;

				vector<string> detections;
				// coco
				detections.push_back("person");
				// OID
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
						geometry_msgs::Point coord = convertOdomToMap(obj.coord.x, obj.coord.y, obj.coord.z);
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

			/*******************************************************************/
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
						geometry_msgs::Point32 coordObj = obj.coord;

						float x = coordObj.x;
						float y = coordObj.y;
						float z = coordObj.z;

						/* geometry_msgs::Point convertOdomToMap(float odomx, float odomy,float odomz) */
						geometry_msgs::Point coord = vision::generic::convertOdomToMap(x, y, z);
						double distance = obj.distance;
						double score = obj.score;
						ROS_INFO("APRES ...got object : %s", msg3.data.c_str());
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

			/*******************************************************************/
			bool WaitForHumanWaivingHand()
			{
				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::wave_hand_detection>("/robobreizh/perception_pepper/wave_hand_detection");

				perception_pepper::wave_hand_detection srv;
				srv.request.distance_max = 10;

				if (client.call(srv))
				{
					geometry_msgs::PoseArray poseArray = srv.response.poses_list;

					int nbPose = poseArray.poses.size();
					ROS_INFO("WaitForHumanWaivingHand OK %d", nbPose);
					if (nbPose == 0)
					{
						return false;
					}

					for (geometry_msgs::Pose pose : poseArray)
					{

						robobreizh::Person person;
						person.posture = "waving";
						person.cloth_color = "";
						person.age = "";
						person.gender = "";
						person.skin_color = "";
						person.height = 0.0;
						person.distance = 0.0;
						person.= 0.0;

						geometry_msgs::Point coord = convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);
						person.pos_x = coord.x;
						person.pos_y = coord.y;
						person.pos_z = coord.z;

						ROS_INFO("...got personne %s position (%f,%f,%f)", posture.c_str(), person.pos_x, person.pos_y, person.pos_z);

						if (addPersonToDatabase(person))
						{
							ROS_INFO("...adding person to db");
						}
					}
					return true;
				}
				else
				{
					ROS_INFO("WaitForHumanWaivingHand OK  - ERROR");
					return false;
				}

				// bool is probably not the right output type, a position seems more relevant
				return true;
			}

			/*******************************************************************/
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
						geometry_msgs::Point coord = convertOdomToMap((float)obj.coord.x, (float)obj.coord.y, (float)obj.coord.z);
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

			/*******************************************************************/
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

			/*******************************************************************/
			bool findHumanAndStoreFeatures(robobreizh::Person *person)
			{

				double distanceMax = 10;
				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>("/robobreizh/perception_pepper/person_features_detection_posture");

				perception_pepper::person_features_detection_posture srv;

				vector<std::string> detections;

				vector<std_msgs::String> tabMsg;

				for (auto t = detections.begin(); t != detections.end(); t++)
				{
					std_msgs::String msg;
					std::stringstream ss;
					ss << *t;
					msg.data = ss.str();
					tabMsg.push_back(msg);
				}

				srv.request.entries_list.obj = tabMsg;
				srv.request.entries_list.distanceMaximum = distanceMax;

				if (client.call(srv))
				{
					perception_pepper::PersonList persList = srv.response.outputs_list;
					perception_pepper::Person_poseList persPoseList = srv.response.outputs_pose_list;

					vector<perception_pepper::Person> persons = persList.person_list;
					vector<perception_pepper::Person_pose> personPoses = persPoseList.person_pose_list;
					int nbPersons = persons.size();
					bool isAdded = false;
					ROS_INFO("findHumanAndStoreFeatures OK, with nbPerson ==  %d", nbPersons);

					for (int i = 0; i < nbPersons; i++)
					{

						// message perception_pepper::Person
						perception_pepper::Person pers = persons[i];
						person->gender = pers.gender.data;
						person->age = pers.age.data;
						person->skin_color = "white";
						person->distance = (float)pers.distance;
						person->cloth_color = pers.clothes_color.data;

						// message perception_pepper::Person_pose
						perception_pepper::Person_pose persPose = personPoses[i];
						person->posture = persPose.posture.data;
						person->height = persPose.height;

						geometry_msgs::Point coord = convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);
						person->pos_x = coord.x;
						person->pos_y = coord.y;
						person->pos_z = coord.z;

						ROS_INFO("...got personne %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position (%f,%f,%f)",
								 i,
								 person->cloth_color.c_str(),
								 person->age.c_str(),
								 person->gender.c_str(),
								 person->skin_color.c_str(),
								 person->posture.c_str(),
								 person->height,
								 person->distance,
								 person->pos_x,
								 person->pos_y,
								 person->pos_z);

						/*
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
				  std::cout << person->age << ", " << person->cloth_color << ", " << person->gender << ", " << person->skin_color << std::endl;*/

						if (person->cloth_color != "" && person->skin_color != "" && person->age != "" && person->gender != "")
						{
							ROS_INFO("...adding person to db");
							robobreizh::database::VisionModel vm;
							vm.createPersonFromFeatures(person->gender, person->age, person->cloth_color, person->skin_color);
							ros::ServiceClient moveHead = nh.serviceClient<manipulation_pepper::EmptySrv>("robobreizh/manipulation/look_down");
							manipulation_pepper::EmptySrv emptySrv;
							moveHead.call(emptySrv);
							isAdded = true;
						}
					}
					if (isAdded)
						return true;
				}
				else
				{
					ROS_INFO("findHumanAndStoreFeatures - ERROR Service, no response");
					return false;
				}
				return false;
			}

			/*******************************************************************/
			bool isInRadius(float x1, float y1, float z1, float x2, float y2, float z2, float epsilon)
			{
				float distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
				std::cout << "	Calculated distance : " << std::to_string(distance) << std::endl;
				if (distance < epsilon)
				{
					return true;
					std::cout << "	distance is smaller than " << std::to_string(epsilon) << std::endl;
				}
				return false;
			}

			/*******************************************************************/
			bool addObjectToDatabase(robobreizh::Object obj)
			{
				robobreizh::database::VisionModel vm;
				// get all objects with label
				std::vector<robobreizh::Object> dbObjects = vm.getObjectsByLabel(obj.label);

				// loop over dbObjects
				bool alreadyExist = false;
				for (auto dbObj : dbObjects)
				{
					if (isInRadius(dbObj.pos_x, dbObj.pos_y, dbObj.pos_z, obj.pos_x, obj.pos_y, obj.pos_z, 0.2))
					{
						alreadyExist = true;
					}
				}

				if (!alreadyExist)
				{
					vm.createObject(obj);
					return true;
				}
				return false;
			}

			geometry_msgs::Point convertOdomToMap(float odomx, float odomy, float odomz)
			{
				geometry_msgs::Point odomPoint;
				odomPoint.x = odomx;
				odomPoint.y = odomy;
				odomPoint.z = odomz;

				tf2_ros::Buffer tfBuffer;
				tf2_ros::TransformListener tfListener(tfBuffer);
				geometry_msgs::TransformStamped transformStamped;

				try
				{
					std::cout << tfBuffer.canTransform("map", "odom", ros::Time(0.0), ros::Duration(3.0)) << std::endl;
					transformStamped = tfBuffer.lookupTransform("map", "odom", ros::Time(0.0), ros::Duration(3.0));
				}
				catch (tf2::TransformException &ex)
				{
					ROS_WARN("%s", ex.what());
					ros::Duration(1.0).sleep();
				}

				geometry_msgs::Point mapPoint;
				tf2::doTransform(odomPoint, mapPoint, transformStamped);
				ROS_INFO("      transformStamped odom -> map: ");
				ROS_INFO("      odomPoint * transformStamped = mapPoint: ");
				ROS_INFO("            x : %f     x : %f     %f", odomPoint.x, transformStamped.transform.translation.x, mapPoint.x);
				ROS_INFO("            y : %f  X  y : %f  =  %f", odomPoint.y, transformStamped.transform.translation.y, mapPoint.y);
				ROS_INFO("            z : %f     z : %f     %f", odomPoint.z, transformStamped.transform.translation.z, mapPoint.z);

				return mapPoint;
			}

			/*******************************************************************/
			bool findStoreAllObjects()
			{
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

					vector<perception_pepper::Object> objects = objectList.objects_list;
					int nbObjects = objects.size();
					ROS_INFO("findStoreObjects OK, with objects ==  %d", nbObjects);

					if (nbObjects == 0)
					{
						return false;
					}

					std::vector<std::string> vPersonObj;
					// coco
					vPersonObj.push_back("person");
					vPersonObj.push_back("clothing");
					vPersonObj.push_back("kite");
					// OID
					vPersonObj.push_back("Clothing");
					vPersonObj.push_back("Office building");
					vPersonObj.push_back("Human face");
					vPersonObj.push_back("Human body");
					vPersonObj.push_back("Human head");
					vPersonObj.push_back("Human arm");
					vPersonObj.push_back("Human hand");
					vPersonObj.push_back("Human nose");
					vPersonObj.push_back("Person");
					vPersonObj.push_back("Man");
					vPersonObj.push_back("Woman");
					vPersonObj.push_back("Boy");
					vPersonObj.push_back("Girl");

					for (auto obj : objects)
					{
						// skips if person objects
						if (std::find(vPersonObj.begin(), vPersonObj.end(), obj.label.data) != vPersonObj.end())
						{
							continue;
						}
						robobreizh::Object objStruct;
						objStruct.label = obj.label.data;
						objStruct.color = obj.color.data;
						objStruct.distance = obj.distance;
						// TODO : convertion into the frame map
						geometry_msgs::Point coord = convertOdomToMap((float)obj.coord.x, (float)obj.coord.y, (float)obj.coord.z);
						objStruct.pos_x = coord.x;
						objStruct.pos_y = coord.y;
						objStruct.pos_z = coord.z;
						ROS_INFO("...got %s %s", objStruct.color.c_str(), objStruct.label.c_str());
						ROS_INFO("     distance: %f, position (%f,%f,%f)", objStruct.distance, coord.x, coord.y, coord.z);

						if (addObjectToDatabase(objStruct))
						{
							ROS_INFO("...added object to db");
						}
					}
					return true;
				}
				else
				{
					ROS_INFO("findStoreAllObject - ERROR");
					return false;
				}
				return true;
			}

			vector<perception_pepper::Object> findAllObjects()
			{
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

					vector<perception_pepper::Object> objects = objectList.objects_list;
					int nbObjects = objects.size();
					ROS_INFO("findStoreObjects OK, with objects ==  %d", nbObjects);

					return objects;
				}
				else
				{
					ROS_INFO("findStoreAllObject - ERROR");
					vector<perception_pepper::Object> result;
					return result;
				}
			}
			/*******************************************************************/
			bool addPersonToDatabase(robobreizh::Person person)
			{
				robobreizh::database::VisionModel vm;
				auto allPerson = vm.getAllPerson();
				// loop over allPerson
				bool alreadyExist = false;
				for (auto dbPerson : allPerson)
				{
					if (isInRadius(dbPerson.pos_x, dbPerson.pos_y, dbPerson.pos_z, person.pos_x, person.pos_y, person.pos_z, 0.2))
					{
						alreadyExist = true;
					}
				}

				if (!alreadyExist)
				{
					vm.createPerson(person);
					return true;
				}

				return false;
			}

			std::string findObjectCategory(std::string object)
			{
				vector<std::string> fruits{"Apple", "Fruit", "Grape", "Tomato", "Lemon", "Banana", "Orange", "Coconut", "Mango", "Pineapple", "Grapefruit",
										   "Pomegranate", "Watermelon", "Strawberry", "Peach", "Cantaloupe", "apple", "banana", "orange"};
				vector<std::string> vegetables{"carrot", "broccoli"};
				vector<std::string> otherFood{"Food", "Croissant", "Doughnut", "Hot dog", "Fast food", "Popcorn", "Cheese", "Muffin", "Cookie", "Dessert", "French fries",
											  "Baked goods", "Pasta", "Pizza", "Sushi", "Bread", "Ice cream", "Salad", "Sandwich", "Pastry", "Waffle", "Pancake", "Burrito", "Snack", "Taco", "Hamburger",
											  "Cake", "Honeycomb", "Pretzel", "Bagel", "Guacamole", "Submarine sandwich", "sandwich", "hot dog", "pizza", "donut", "cake", "Candy"};

				for (auto obj : fruits)
				{
					if (obj == object)
					{
						return "fruit";
					}
				}
				for (auto obj : vegetables)
				{
					if (obj == object)
					{
						return "vegetable";
					}
				}
				for (auto obj : otherFood)
				{
					if (obj == object)
					{
						return "other";
					}
				}

				return "none";
			}

			std::string findObjectRange(std::string object, geometry_msgs::Point coord)
			{
				if ((coord.y > 1.4) && (coord.y < 1.6))
				{
					return "Shelf 1";
				}
				if ((coord.y > 1.6) && (coord.y < 1.8))
				{
					return "Shelf 2";
				}
				if ((coord.y > 1.8) && (coord.y < 1.8))
				{
					return "Shelf 3";
				}
			}

			std::string findAndLocateLastObjectPose()
			{
				vector<perception_pepper::Object> objList;
				objList = vision::generic::findAllObjects();
				map<std::string, std::string> relativeposes;

				for (auto obj : objList)
				{
					std::string category;
					std::string position;
					category = vision::generic::findObjectCategory(obj.label.data);
					position = findObjectRange(obj.label.data, obj.coord);
					relativeposes[category] = position;
				}
				robobreizh::database::VisionModel bdd;
				robobreizh::Object obj;
				obj = bdd::getLastObject();
				for (auto elem : relativeposes)
				{
					if (elem.first == vision::generic::findObjectCategory(obj.label.data))
					{
						return elem.second;
					}
				}
				return "";
			}

			/*******************************************************************/
			int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax)
			{

				ros::NodeHandle nh;
				ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>("/robobreizh/perception_pepper/person_features_detection_posture");

				perception_pepper::person_features_detection_posture srv;

				vector<std::string> detections;

				vector<std_msgs::String> tabMsg;

				for (auto t = detections.begin(); t != detections.end(); t++)
				{
					std_msgs::String msg;
					std::stringstream ss;
					ss << *t;
					msg.data = ss.str();
					tabMsg.push_back(msg);
				}

				srv.request.entries_list.obj = tabMsg;
				srv.request.entries_list.distanceMaximum = distanceMax;

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

						// message perception_pepper::Person
						perception_pepper::Person pers = persons[i];
						person.gender = pers.gender.data;
						person.age = pers.age.data;
						person.skin_color = "white";
						person.distance = (float)pers.distance;
						person.cloth_color = pers.clothes_color.data;

						// message perception_pepper::Person_pose
						perception_pepper::Person_pose persPose = personPoses[i];
						person.posture = persPose.posture.data;
						person.height = persPose.height;

						ROS_INFO("            x : %f", pers.coord.x);
						ROS_INFO("            y : %f", pers.coord.y);
						ROS_INFO("            z : %f", pers.coord.z);

						if (!person.gender.empty())
						{
							geometry_msgs::Point coord = convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);
							person.pos_x = coord.x;
							person.pos_y = coord.y;
							person.pos_z = coord.z;

							ROS_INFO("...got personne %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position (%f,%f,%f)", i, person.cloth_color.c_str(), person.age.c_str(), person.gender.c_str(), person.skin_color.c_str(), person.posture.c_str(), person.height, person.distance, person.pos_x, person.pos_y, person.pos_z);

							if (addPersonToDatabase(person))
							{
								ROS_INFO("...adding person to db");
							}
						}
					}
					return nbPersons;
				}
				else
				{
					ROS_ERROR("findHumanAndStoreFeaturesWihDistanceFilter - ERROR");
					return 0;
				}
				return 0;
			}

		} // namespace generic
	}	  // namespace vision
} // namespace robobreizh
