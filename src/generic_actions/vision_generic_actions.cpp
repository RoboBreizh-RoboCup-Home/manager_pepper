// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

// ROBOBREIZH
#include <perception_pepper/ObjectsList.h>
#include <perception_pepper/Person.h>

// NAOQI --> Service
#include <perception_pepper/object_detection_service.h>
#include <perception_pepper/person_features_detection_service.h>
#include <perception_pepper/person_features_detection_posture.h>
#include <perception_pepper/shoes_and_drink_detection_service.h>
#include <perception_pepper/wave_hand_detection.h>
#include <perception_pepper/PersonList.h>
#include <std_msgs/Empty.h>

#include <boost/thread/thread.hpp>

#include "database_model/object_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/person_model.hpp"
#include "generic_actions/vision_generic_actions.hpp"

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "manager_utils.hpp"
#include "vision_utils.hpp"

using namespace std;

namespace robobreizh
{
namespace vision
{
namespace generic
{
bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>(
      "/robobreizh/perception_pepper/person_features_detection_posture");

  perception_pepper::person_features_detection_posture srv;

  vector<std::string> detections;
  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = distanceMax;

  if (client.call(srv))
  {
    perception_pepper::PersonList persList = srv.response.outputs_list;
    perception_pepper::Person_poseList persPoseList = srv.response.outputs_pose_list;

    vector<perception_pepper::Person> persons = persList.person_list;
    vector<perception_pepper::Person_pose> personPoses = persPoseList.person_pose_list;
    robobreizh::database::Person person;

    int nbPersons = persons.size();
    float distMax = 5;
    bool isAdded = false;
    for (int i = 0; i < nbPersons; i++)
    {
        perception_pepper::Person pers = persons[i];
      // message perception_pepper::Person
      if ((float)pers.distance < distMax)
      {
        perception_pepper::Person_pose persPose = personPoses[i];
        geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);

        personMsgToPersonStruct(&person, pers, persPose, coord);
        ROS_INFO(
            "...closest person %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, "
            "position (%f,%f,%f)",
            i, person.cloth_color.label.c_str(), person.age.c_str(), person.gender.c_str(),
            person.skin_color.label.c_str(), person.posture.c_str(), person.height, person.distance, person.position.x, person.position.y,
            person.position.z);
      }
    }
    robobreizh::database::PersonModel pm;
    int id = pm.getFirstPersonId();
    pm.updatePerson(id, person);
    ROS_INFO("...adding person to db");
    return true;
  }
  else
  {
    ROS_ERROR("findHostAndStoreFeaturesWihDistanceFilter - ERROR");
    return false;
  }

  return false;
}

/*******************************************************************/
bool waitForHuman()
{
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");

  perception_pepper::object_detection_service srv;

  vector<string> detections{ // coco
                             "person",
                             // OID
                             "Human face", "Human body", "Human head", "Human arm", "Human hand", "Human nose",
                             "Person", "Man", "Woman", "Boy", "Girl"
  };

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

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
      double distance = obj.distance;
      double score = obj.score;
      ROS_INFO("...got object : %s", obj.label.data.c_str());
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
  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");

  perception_pepper::object_detection_service srv;

  vector<string> detections;
  detections.push_back(objectName);

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

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
      geometry_msgs::Point32 coordObj = obj.coord;

      /* geometry_msgs::Point robobreizh::convertOdomToMap(float odomx, float odomy,float odomz) */
      double distance = obj.distance;
      double score = obj.score;
      ROS_INFO("APRES ...got object : %s", obj.label.data.c_str());
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
bool WaitForHumanWavingHand()
{
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<perception_pepper::wave_hand_detection>("/robobreizh/perception_pepper/wave_hand_detection");

  perception_pepper::wave_hand_detection srv;
  srv.request.distance_max = 10;

  if (client.call(srv))
  {
    geometry_msgs::PoseArray poseArray = srv.response.poses_list;

    int nbPose = poseArray.poses.size();
    ROS_INFO("WaitForHumanWavingHand OK %d", nbPose);
    if (nbPose == 0)
    {
      return false;
    }

    for (auto pose : poseArray.poses)
    {
      robobreizh::database::Person person;

      geometry_msgs::Point coord =
          robobreizh::convertOdomToMap((float)pose.position.x, (float)pose.position.y, (float)pose.position.z);
      person.position = coord;
      person.posture = "waving";

      ROS_INFO("...got personne %s position (%f,%f,%f)", person.posture.c_str(), person.position.x, person.position.y, person.position.z);

      if (addPersonToDatabase(person))
      {
        ROS_INFO("...adding person to db");
      }
    }
    return true;
  }
  else
  {
    ROS_INFO("WaitForHumanWavingHand OK  - ERROR");
    return false;
  }

  // bool is probably not the right output type, a position seems more relevant
  return true;
}

/*******************************************************************/
bool FindEmptySeat()
{
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/seat_detection_service");

  perception_pepper::object_detection_service srv;

  vector<string> detections;
  detections.push_back("SEAT_INFORMATION");

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list = tabMsg;

  if (client.call(srv))
  {
    perception_pepper::ObjectsList objList = srv.response.outputs_list;
    vector<perception_pepper::Object> objects = objList.objects_list;
    int nbObjects = objects.size();
    if (nbObjects == 0)
    {
      return false;
    }
    geometry_msgs::Point coord;
    for (int i = 0; i < nbObjects; i++)
    {
      perception_pepper::Object obj = objects[i];
      coord = robobreizh::convertOdomToMap((float)obj.coord.x, (float)obj.coord.y, (float)obj.coord.z);

      double distance = obj.distance;
      double score = obj.score;
      ROS_INFO("...got object : %s",obj.label.data.c_str());
      ROS_INFO("            x : %f", coord.x);
      ROS_INFO("            y : %f", coord.y);
      ROS_INFO("            z : %f", coord.z);
      ROS_INFO("            distance : %f", distance);
      ROS_INFO("            score : %f", score);
    }
    float yaw_angle = robobreizh::convertOdomToBaseFootprint(coord.x, coord.y, coord.z);
    navigation::generic::rotateOnPoint(yaw_angle);

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
bool isDoorOpened()  // TODO: What if door not found => Use Enum instead (Open, closed, NotFound)
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
bool findHumanAndStoreFeatures(robobreizh::database::Person* person)
{
  double distanceMax = 10;
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>(
      "/robobreizh/perception_pepper/person_features_detection_posture");

  perception_pepper::person_features_detection_posture srv;
  vector<std::string> detections;
  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = distanceMax;

  if (client.call(srv))
  {
    vector<perception_pepper::Person> persons = srv.response.outputs_list.person_list;
    vector<perception_pepper::Person_pose> personPoses = srv.response.outputs_pose_list.person_pose_list;
    int nbPersons = persons.size();
    bool isAdded = false;
    ROS_INFO("findHumanAndStoreFeatures OK, with nbPerson ==  %d", nbPersons);

    for (int i = 0; i < nbPersons; i++)
    {
      perception_pepper::Person pers = persons[i];
      perception_pepper::Person_pose persPose = personPoses[i];
      geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);

      personMsgToPersonStruct(person, pers, persPose, coord);

      ROS_INFO(
          "...got personne %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position "
          "(%f,%f,%f)",
          i, person->cloth_color.label.c_str(), person->age.c_str(), person->gender.c_str(),
          person->skin_color.label.c_str(), person->posture.c_str(), person->height, person->distance,
          person->position.x, person->position.y, person->position.z);

      if (person->cloth_color.label != "" && person->skin_color.label != "" && person->age != "" &&
          person->gender != "")
      {
        ROS_INFO("...adding person to db");
        robobreizh::database::PersonModel pm;
        pm.insertPerson(*person);
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

bool findStoreObjectAtLocation(std::string objectName, std::string objectLocation)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");

  perception_pepper::object_detection_service srv;

  std::vector<std::string> detections{ objectName };

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list = tabMsg;

  if (client.call(srv))
  {
    perception_pepper::ObjectsList objList = srv.response.outputs_list;
    vector<perception_pepper::Object> objects = objList.objects_list;
    int nbObjects = objects.size();
    if (nbObjects == 0)
    {
      return false;
    }

    ROS_INFO("findStoreObjectAtLocation OK, with objects ==  %d", nbObjects);

    std::vector<std::string> vPersonObj{ // coco
                                         "person", "clothing", "kite",
                                         // OID
                                         "Clothing", "Office building", "Human face", "Human body", "Human head",
                                         "Human arm", "Human hand", "Human nose", "Person", "Man", "Woman", "Boy",
                                         "Girl"
    };
    for (auto obj : objects)
    {
      // skips if person objects
      if (std::find(vPersonObj.begin(), vPersonObj.end(), obj.label.data) != vPersonObj.end())
      {
        continue;
      }

      robobreizh::database::LocationModel lm;
      auto location = lm.getLocationFromName(objectLocation);
      geometry_msgs::Point coord = location.pose.position;

      robobreizh::database::Object objStruct;
      objectMsgToObjectStruct(&objStruct, obj, coord);

      ROS_INFO("...got %s %s", objStruct.color.label.c_str(), objStruct.label.c_str());
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
    ROS_INFO("findObject OK  - ERROR");
    return false;
  }

  // bool is probably not the right output type, a position seems more relevant
  return true;
}

/*******************************************************************/
bool findStoreAllObjects()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");
  perception_pepper::object_detection_service srv;
  vector<std::string> detections{ "ALL" };

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

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

    std::vector<std::string> vPersonObj
    {
      // coco
      "person", "clothing", "kite",
          // OID
          "Clothing", "Office building", "Human face", "Human body", "Human head", "Human arm", "Human hand",
          "Human nose", "Person", "Man", "Woman", "Boy", "Girl"
    };

    for (auto obj : objects)
    {
      // skips if person objects
      if (std::find(vPersonObj.begin(), vPersonObj.end(), obj.label.data) != vPersonObj.end())
      {
        continue;
      }
      robobreizh::database::Object objStruct;
      geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)obj.coord.x, (float)obj.coord.y, (float)obj.coord.z);
      objectMsgToObjectStruct(&objStruct, obj, coord);
      ROS_INFO("...got %s %s", objStruct.color.label.c_str(), objStruct.label.c_str());
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
  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");
  perception_pepper::object_detection_service srv;
  vector<std::string> detections;
  detections.push_back("ALL");

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list = tabMsg;

  if (client.call(srv))
  {
    vector<perception_pepper::Object> objects = srv.response.outputs_list.objects_list;
    int nbObjects = objects.size();
    ROS_INFO("findAllObjects OK, with objects ==  %d", nbObjects);

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

bool findAndLocateBag()
{
  vector<std::string> bags{ "handbag",          "backpack", "Plastic bag", "Handbag",
                            "Luggage and bags", "Backpack", "Suitcase",    "Briefcase" };
  vector<perception_pepper::Object> objList;
  objList = vision::generic::findAllObjects();
  for (auto elem : objList)
  {
    for (auto elem2 : bags)
    {
      if (elem.label.data == elem2)
      {
        robobreizh::database::Object objStruct;
        geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)elem.coord.x, (float)elem.coord.y, (float)elem.coord.z);
        objectMsgToObjectStruct(&objStruct, elem, coord);

        ROS_INFO("...got %s %s", objStruct.color.label.c_str(), objStruct.label.c_str());
        ROS_INFO("    position (%f,%f,%f)", coord.x, coord.y, coord.z);

        if (addObjectToDatabase(objStruct))
        {
          ROS_INFO("...added object to db");
          return true;
        }
        else
        {
          return false;
        }
      }
    }
  }
  return false;
}

bool findAndLocateCabDriver()
{
  /* Option 1 : umbrella */
  vector<std::string> umbrellas{ "umbrella", "Umbrella" };
  vector<perception_pepper::Object> objList;
  objList = vision::generic::findAllObjects();
  for (auto elem : objList)
  {
    for (auto elem2 : umbrellas)
    {
      if (elem.label.data == elem2)
      {
        robobreizh::database::Person person;
        person.name = "cabDriver";

        geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)elem.coord.x, (float)elem.coord.y, (float)elem.coord.z);
        person.position = coord;

        ROS_INFO("...got cab driver at position (%f,%f,%f)", person.position.x, person.position.y, person.position.z);

        if (addPersonToDatabase(person))
        {
          ROS_INFO("...adding cab driver to db");
        }
        else
        {
          return false;
        }
        return true;
      }
    }
  }

  /* Option 2 : person with yellow jersey == PAS BLACK*/
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>(
      "/robobreizh/perception_pepper/person_features_detection_posture");

  perception_pepper::person_features_detection_posture srv;

  vector<std::string> detections;

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = 100.0;

  if (client.call(srv))
  {
    perception_pepper::PersonList persList = srv.response.outputs_list;
    perception_pepper::Person_poseList persPoseList = srv.response.outputs_pose_list;

    vector<perception_pepper::Person> persons = persList.person_list;
    vector<perception_pepper::Person_pose> personPoses = persPoseList.person_pose_list;
    int nbPersons = persons.size();

    ROS_INFO("findAndLocateCabDriver OK, with nbPerson ==  %d", nbPersons);

    for (int i = 0; i < nbPersons; i++)
    {
      robobreizh::database::Person person;
      perception_pepper::Person pers = persons[i];
      perception_pepper::Person_pose persPose = personPoses[i];
      geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);

      personMsgToPersonStruct(&person, pers, persPose, coord);

      if (person.cloth_color.label != "Black")
      {
        ROS_INFO("...got cab driver ");

        if (addPersonToDatabase(person))
        {
          ROS_INFO("...adding cab driver to db");
        }
      }
    }
    return nbPersons;
  }
  else
  {
    ROS_ERROR("findAndLocateCabDriver - ERROR");
    return 0;
  }

  return false;
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

    category = robobreizh::findObjectCategory(obj.label.data);
    position = robobreizh::findObjectRange(obj.label.data, obj.coord);

    relativeposes[category] = position;
  }
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj;
  obj = om.getLastObject();
  for (auto elem : relativeposes)
  {
    auto categoryTmp = robobreizh::findObjectCategory(obj.label);
    if (elem.first == categoryTmp)
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
  ros::ServiceClient client = nh.serviceClient<perception_pepper::person_features_detection_posture>(
      "/robobreizh/perception_pepper/person_features_detection_posture");

  perception_pepper::person_features_detection_posture srv;
  vector<std::string> detections;
  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

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
      robobreizh::database::Person person;

      // message perception_pepper::Person
      perception_pepper::Person pers = persons[i];
      perception_pepper::Person_pose persPose = personPoses[i];
      geometry_msgs::Point coord = robobreizh::convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);
      personMsgToPersonStruct(&person, pers, persPose, coord);

      ROS_INFO("            x : %f", pers.coord.x);
      ROS_INFO("            y : %f", pers.coord.y);
      ROS_INFO("            z : %f", pers.coord.z);
      ROS_INFO("            height : %f", pers.height);

      if (person.gender.empty())
      {
        person.gender = "M";
      }

      ROS_INFO(
          "...got personne %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position "
          "(%f,%f,%f)",
          i, person.cloth_color.label.c_str(), person.age.c_str(), person.gender.c_str(),
          person.skin_color.label.c_str(), person.posture.c_str(), person.height, person.distance, person.position.x,
          person.position.y, person.position.z);

      if (addPersonToDatabase(person))
      {
        ROS_INFO("...adding person to db");
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

int breakTheRules(double distanceMax)
{
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<perception_pepper::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");

  perception_pepper::object_detection_service srv;

  std::vector<std::string> detections{ // coco
                                       "person",
                                       // OID
                                       "Human face", "Human body", "Human head", "Human arm", "Human hand",
                                       "Human nose", "Person", "Man", "Woman", "Boy", "Girl"
  };

  vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

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
      geometry_msgs::Point coord = robobreizh::convertOdomToMap(obj.coord.x, obj.coord.y, obj.coord.z);
      double distance = obj.distance;
      double score = obj.score;
      ROS_INFO("...got object : %s", obj.label.data.c_str());
      ROS_INFO("            x : %f", coord.x);
      ROS_INFO("            y : %f", coord.y);
      ROS_INFO("            z : %f", coord.z);
      ROS_INFO("            distance : %f", distance);
      ROS_INFO("            score : %f", score);

      if (robobreizh::isInForbiddenRoom(coord.x, coord.y))
      {
        return 3;
      }
    }
  }

  else
  {
    ROS_ERROR("Shoes and drinks service couldn t be called");
    return 0;
  }

  return 0;
}

}  // namespace generic
}  // namespace vision
}  // namespace robobreizh
