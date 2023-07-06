// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <actionlib/client/terminal_state.h>

//#include <robobreizh_demo_components/PepperSpeech.h>
//#include <robobreizh_demo_components/Person.h>

// ROBOBREIZH
#include <robobreizh_msgs/Object.h>
#include <robobreizh_msgs/Person.h>
#include <robobreizh_msgs/shoes_detection.h>

// NAOQI --> Service
#include <robobreizh_msgs/pointing_hand_detection.h>
#include <robobreizh_msgs/object_detection_service.h>
#include <robobreizh_msgs/person_features_detection_service.h>
#include <robobreizh_msgs/shoes_and_drink_detection_service.h>
#include <robobreizh_msgs/gpsr_gesture_detection.h>
#include <robobreizh_msgs/hand_waving_detection.h>
#include <robobreizh_msgs/category_detection_service.h>
#include <robobreizh_msgs/Person.h>
#include <robobreizh_msgs/SonarAction.h>

#include <boost/thread/thread.hpp>

#include "database_model/object_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/person_model.hpp"
#include "generic_actions/vision_generic_actions.hpp"

#include "plan_high_level_actions/navigation_plan_actions.hpp"
#include "generic_actions/navigation_generic_actions.hpp"
#include "manager_utils.hpp"
#include "vision_utils.hpp"

namespace robobreizh {
namespace vision {
namespace generic {

std::unordered_map<std::string, int> countPose(std::unordered_map<std::string, std::string> WhatVariations) {
  // /robobreizh/perception_pepper/gpsr_gesture_detection
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::gpsr_gesture_detection>(
      "/robobreizh/perception_pepper/gpsr_gesture_detection");

  robobreizh_msgs::gpsr_gesture_detection srv;

  srv.request.distance_max = 5;
  srv.request.request.data = "";

  int waving_count = 0;
  int raising_left_count = 0;
  int raising_right_count = 0;
  int pointing_left_count = 0;
  int pointing_right_count = 0;

  std::unordered_map<std::string, int> countPoseMap;

  countPoseMap.insert(std::make_pair("waving_count",0));
  countPoseMap.insert(std::make_pair("raising_left_count",0));
  countPoseMap.insert(std::make_pair("raising_right_count",0));
  countPoseMap.insert(std::make_pair("pointing_left_count",0));
  countPoseMap.insert(std::make_pair("pointing_right_count",0));

  if (client.call(srv)) {
    auto person_pose_list = srv.response.outputs_list.person_pose_list;

    if (person_pose_list.size() == 0) {
      ROS_INFO("matchPose OK  - No person found");
      return countPoseMap;
    }

    for (auto person: person_pose_list){

      if (person.waving == true) {
        waving_count++;
      }

      if (person.raising_left == true) {
        raising_left_count++;
      }      

      if (person.raising_right == true) {
        raising_right_count++;
      }   

      if (person.pointing_left == true) {
        pointing_left_count++;
      }      

      if (person.pointing_right == true) {
        pointing_right_count++;
      }      
    }

    countPoseMap["waving_count"] = waving_count;
    countPoseMap["raising_left_count"] = raising_left_count;
    countPoseMap["raising_right_count"] = raising_right_count;
    countPoseMap["pointing_left_count"] = pointing_left_count;
    countPoseMap["pointing_right_count"] = pointing_right_count;

    return countPoseMap;
  } else {
    ROS_INFO("matchPose OK  - ERROR");
    return countPoseMap;
  }
}


std::string matchPose(std::unordered_map<std::string, std::string> WhatVariations) {
  // /robobreizh/perception_pepper/gpsr_gesture_detection
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::gpsr_gesture_detection>(
      "/robobreizh/perception_pepper/gpsr_gesture_detection");

  robobreizh_msgs::gpsr_gesture_detection srv;

  srv.request.distance_max = 5;
  srv.request.request.data = "";

  if (client.call(srv)) {
    auto person_pose_list = srv.response.outputs_list.person_pose_list;

    if (person_pose_list.size() == 0) {
      ROS_INFO("matchPose OK  - No person found");
      return "";
    }

    if (person_pose_list[0].waving == true) {
      ROS_INFO("matchPose OK  - Waving");
      return "waving hand";
    }
    if (person_pose_list[0].raising_left == true) {
      ROS_INFO("matchPose OK  - Raising left");
      return "raising left";
    }
    if (person_pose_list[0].raising_right == true) {
      ROS_INFO("matchPose OK  - Raising right");
      return "raising right";
    }
    if (person_pose_list[0].pointing_left == true) {
      ROS_INFO("matchPose OK  - Pointing left");
      return "pointing left";
    }
    if (person_pose_list[0].pointing_right == true) {
      ROS_INFO("matchPose OK  - Pointing right");
      return "pointing right";
    }
    ROS_INFO("matchPose OK  - No person with matching post found");
    return "";
  } else {
    ROS_INFO("matchPose OK  - ERROR");
    return "";
  }
  return "";
}

bool findHostAndStoreFeaturesWithDistanceFilter(double distanceMax) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/person_feature_detection");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections;
  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = distanceMax;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    robobreizh::database::Person person;

    int nbPersons = persons.size();
    if (nbPersons == 0) {
      ROS_WARN("Nb detected person = 0 [findHostAndStoreFeaturesWithDistanceFilter]");
      return false;
    }
    float distMax = 5;
    bool isAdded = false;
    for (int i = 0; i < nbPersons; i++) {
      robobreizh_msgs::Person pers = persons[i];
      // message robobreizh_msgs::Person
      if ((float)pers.distance < distMax) {
        geometry_msgs::PointStamped ps;
        ps.header.frame_id = "odom";
        ps.point.x = (float)pers.coord.x;
        ps.point.y = (float)pers.coord.y;
        ps.point.z = (float)pers.coord.z;
        auto coord = convert_point_stamped_to_frame(ps, "map");

        personMsgToPersonStruct(&person, pers, coord.point);
        ROS_INFO(
            "...closest person %d : %s clothes, %s style,%s years old, %s, %s skin, %f m distance, "
            "position (%f,%f,%f)",
            i, person.cloth_color.label.c_str(), person.clothes_style.c_str(), person.age.c_str(),
            person.gender.c_str(), person.skin_color.label.c_str(), person.distance, person.position.x,
            person.position.y, person.position.z);
      }
    }
    robobreizh::database::PersonModel pm;
    int id = pm.getFirstPersonId();
    robobreizh::database::Person first_person = pm.getPerson(id);
    person.name = first_person.name;
    person.favorite_drink = first_person.favorite_drink;
    pm.updatePerson(id, person);
    ROS_INFO("...adding person to db");
    return true;
  }
  ROS_ERROR("findHostAndStoreFeaturesWihDistanceFilter - ERROR");
  return false;
}

/*******************************************************************/
bool waitForHuman() {
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/person_detection");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections{ "person" };

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;

  srv.request.entries_list.distanceMaximum = 3;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> people = srv.response.outputs_list.person_list;
    int nbPeople = people.size();
    ROS_INFO("WaitForHuman OK %d", nbPeople);

    if (nbPeople > 0) {
      ROS_INFO("Human Found");
      return true;
    }
  }
  ROS_INFO("WaitForHuman OK  - ERROR");
  return false;
}

std::vector<robobreizh::database::Person> findPersonPosition(float distance_max) {
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/person_detection");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections{ "person" };

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;

  srv.request.entries_list.distanceMaximum = distance_max;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> people = srv.response.outputs_list.person_list;
    int nbPeople = people.size();
    ROS_INFO("WaitForHuman OK %d", nbPeople);

    if (nbPeople > 0) {
      ROS_INFO("Human Found");
      return std::vector<robobreizh::database::Person>();
    }

    for (auto person_pose : people) {
      robobreizh::database::Person person;

      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)person_pose.coord.x;
      ps.point.y = (float)person_pose.coord.y;
      ps.point.z = (float)person_pose.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");
      person.position = coord.point;

      ROS_INFO("...got person %s position (%f,%f,%f)", person.posture.c_str(), person.position.x, person.position.y,
               person.position.z);
    }
  }
  ROS_INFO("WaitForHuman OK  - ERROR");
  return std::vector<robobreizh::database::Person>();
}

// #ifdef LEGACY
geometry_msgs::Pose getTrackerPersonPose() {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::hand_waving_detection>("/robobreizh/perception_pepper/gpsr_gesture_detection");
      // nh.serviceClient<robobreizh_msgs::hand_waving_detection>("/robobreizh/perception_pepper/hand_waving");
  robobreizh_msgs::hand_waving_detection srv;

  srv.request.entries_list.distanceMaximum = 2;
  geometry_msgs::Pose tracked_person;
  if (client.call(srv)) {
    std::vector<robobreizh_msgs::PersonPose> person_list = srv.response.outputs_list.person_pose_list;
    robobreizh_msgs::PersonPose closest_person;
    closest_person.distance = 2.1;
    for (robobreizh_msgs::PersonPose person : person_list) {
      if (person.distance < closest_person.distance) {
        closest_person = person;
      }
    }                           //2.1
    if (closest_person.distance == 3) {
      ROS_ERROR("[getTrackerPersonPose] service didn't return any valid person to follow");
      return tracked_person;
    }
    // geometry_msgs::Point coord = robobreizh::convertOdomToMap(
    //     (float)closest_person.coord.x, (float)closest_person.coord.y, (float)closest_person.coord.z);
    
    auto coord = convert_point_stamped_to_frame(closest_person.coord, "map");

    tracked_person.position = coord;
    ROS_INFO("A person to track has been found and will be followed");
    return tracked_person;
  }
  ROS_ERROR("[getTrackerPersonPose] service call went wrong");
  return tracked_person;
}
// #endif

// Finds a specific object and return it's position
bool findObject(std::string objectName, database::Object* last_object) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::object_detection_service>("/robobreizh/perception_pepper/object_detection");

  robobreizh_msgs::object_detection_service srv;
  std::vector<std::string> detections;
  detections.push_back(objectName);

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = 3;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    ROS_INFO("findObject OK %d", nbObjects);

    for (int i = 0; i < nbObjects; i++) {
      robobreizh_msgs::Object obj = objects[i];

      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)obj.coord.x;
      ps.point.y = (float)obj.coord.y;
      ps.point.z = (float)obj.coord.z;

      (*last_object).position = convert_point_stamped_to_frame(ps, "map").point;
      (*last_object).distance = obj.distance;
      (*last_object).label = obj.label.data;
      (*last_object).color = { obj.color.data };
      ROS_INFO("[findObject]...got object : %s", (*last_object).label.c_str());
      ROS_INFO("            distance : %f", (*last_object).distance);
    }
    if (nbObjects == 0) {
      ROS_ERROR("[findObject] No object with the name %s were found", objectName.c_str());
      return false;
    }
  } else {
    ROS_ERROR("[findObject] service call /robobreizh/perception_pepper/object_detection Failed");
    return false;
  }
  return true;
}

/*******************************************************************/
bool WaitForHumanWavingHand() {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::hand_waving_detection>("/robobreizh/perception_pepper/hand_waving");

  robobreizh_msgs::hand_waving_detection srv;
  srv.request.entries_list.distanceMaximum = 10;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::PersonPose> person_pose_list = srv.response.outputs_list.person_pose_list;

    int nb_person = person_pose_list.size();
    ROS_INFO("WaitForHumanWavingHand OK %d", nb_person);
    if (nb_person == 0) {
      return false;
    }

    for (auto person_pose : person_pose_list) {
      robobreizh::database::Person person;

      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)person_pose.coord.x;
      ps.point.y = (float)person_pose.coord.y;
      ps.point.z = (float)person_pose.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");

      person.position = coord.point;
      person.posture = "waving";

      ROS_INFO("...got person %s position (%f,%f,%f)", person.posture.c_str(), person.position.x, person.position.y,
               person.position.z);

      if (addPersonToDatabase(person)) {
        ROS_INFO("...adding person to db");
      }
    }
    return true;
  } else {
    ROS_INFO("WaitForHumanWavingHand OK  - ERROR");
    return false;
  }

  // bool is probably not the right output type, a position seems more relevant
  return true;
}

Direction findDirectionPointedAt() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::pointing_hand_detection>(
      "/robobreizh/perception_pepper/pointing_hand_detection");
  robobreizh_msgs::pointing_hand_detection srv;
  srv.request.distance_max = 3;

  if (client.call(srv)) {
    std::vector<uint8_t> right_list = srv.response.right_list;
    std::vector<uint8_t> top_list = srv.response.top_list;
    if (right_list[0]) {
      return Direction::RIGHT;
    }

    return Direction::LEFT;
  }
  return Direction::NONE;
}

/*******************************************************************/
bool FindEmptySeat() {
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::object_detection_service>(
      "/robobreizh/perception_pepper/seat_detection_service");

  robobreizh_msgs::object_detection_service srv;

  std::vector<std::string> detections = { "chair", "person", "couch" };
  detections.push_back("SEAT_INFORMATION");
  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = 3;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    if (nbObjects == 0) {
      return false;
    }
    geometry_msgs::Point coord;
    for (int i = 0; i < nbObjects; i++) {
      robobreizh_msgs::Object obj = objects[i];

      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)obj.coord.x;
      ps.point.y = (float)obj.coord.y;
      ps.point.z = (float)obj.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");

      robobreizh::database::Object objStruct;
      objectMsgToObjectStruct(&objStruct, obj, coord.point);

      if (addObjectToDatabase(objStruct)) {
        ROS_INFO("...added empty chair to db");
      } else {
        ROS_INFO("...Failed adding empty chair to db");
      }

      double distance = obj.distance;
      double score = obj.score;
      ROS_INFO("...got object : %s", obj.label.data.c_str());
      ROS_INFO("            x : %f", coord.point.x);
      ROS_INFO("            y : %f", coord.point.y);
      ROS_INFO("            z : %f", coord.point.z);
      ROS_INFO("            distance : %f", distance);
      ROS_INFO("            score : %f", score);
    }

    return true;
  } else {
    ROS_INFO("FindEmptySeat OK  - ERROR");
    return false;
  }

  // bool is probably not the right output type, a position seems more relevant
  return true;
}

std::string FindObjectStoringGroceries(){
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection");

  robobreizh_msgs::object_detection_service srv;
  srv.request.entries_list.distanceMaximum = 3;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    if (nbObjects == 0) {
      return "";
    }
    geometry_msgs::Point coord;
    robobreizh_msgs::Object obj = objects.back();
    ROS_INFO("...got object : %s", obj.label.data.c_str());
    return obj.label.data.c_str();

  } else {
    ROS_INFO("Find Object  - SERVICE ERROR");

    return "";
  }
  return "";
}

/*******************************************************************/
bool isDoorOpened()  // TODO: What if door not found => Use Enum instead (Open, closed, NotFound)
{
  ros::NodeHandle nh;
  boost::shared_ptr<std_msgs::Float32 const> shared_msg;
  std_msgs::Float32 msg;
  ROS_INFO("[isDoorOpened]  Waiting for go signal from /robobreizh/door_detection_action");

  actionlib::SimpleActionClient<robobreizh_msgs::SonarAction> ac("/robobreizh/door_detection_action", true);
  ac.waitForServer();

  // prepare goal message with the timeout
  ROS_INFO("Sending door detection goal");
  robobreizh_msgs::SonarGoal sonar_action_goal;
  // timeout after 60 seconds
  sonar_action_goal.timeout = 60;
  ac.sendGoal(sonar_action_goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Door detection action finished: %s", state.toString().c_str());
    return true;
  } else {
    ROS_ERROR("Action did not finish before the time out.");
    ac.cancelGoal();
    return false;
  }
  ROS_ERROR("[is door open] returned false but is not supposed to");
  return false;
}

/*******************************************************************/
bool findHumanAndStoreFeatures(robobreizh::database::Person* person) {
  // use hashtable for values occurence
  double distanceMax = 10;
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/person_feature_detection");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections;
  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = distanceMax;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    int nbPersons = persons.size();
    bool isAdded = false;
    ROS_INFO("findHumanAndStoreFeatures OK, with nbPerson ==  %d", nbPersons);

    for (int i = 0; i < nbPersons; i++) {
      robobreizh_msgs::Person pers = persons[i];
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)pers.coord.x;
      ps.point.y = (float)pers.coord.y;
      ps.point.z = (float)pers.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");

      personMsgToPersonStruct(person, pers, coord.point);

      ROS_INFO(
          "...got person %d : %s clothes, %s years old, %s, %s skin, %f m distance, position "
          "(%f,%f,%f)",
          i, person->cloth_color.label.c_str(), person->age.c_str(), person->gender.c_str(),
          person->skin_color.label.c_str(), person->distance, person->position.x, person->position.y,
          person->position.z);

      if (person->cloth_color.label != "" && person->skin_color.label != "" && person->age != "" &&
          person->gender != "" && person->clothes_style != "") {
        ROS_INFO("...adding person to db");
        robobreizh::database::PersonModel pm;
        pm.insertPerson(*person);
        isAdded = true;
      }
    }
    if (isAdded)
      return true;
  } else {
    ROS_INFO("findHumanAndStoreFeatures - ERROR Service, no response");
    return false;
  }
  return false;
}

bool findStoreObjectAtLocation(std::string objectName, std::string objectLocation) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");

  robobreizh_msgs::object_detection_service srv;

  std::vector<std::string> detections{ objectName };

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = 3;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    if (nbObjects == 0) {
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
    for (auto obj : objects) {
      // skips if person objects
      if (std::find(vPersonObj.begin(), vPersonObj.end(), obj.label.data) != vPersonObj.end()) {
        continue;
      }

      robobreizh::database::LocationModel lm;
      auto location = lm.getLocationFromName(objectLocation);
      geometry_msgs::Point coord = location.pose.position;

      robobreizh::database::Object objStruct;
      objectMsgToObjectStruct(&objStruct, obj, coord);

      ROS_INFO("...got %s %s", objStruct.color.label.c_str(), objStruct.label.c_str());
      ROS_INFO("     distance: %f, position (%f,%f,%f)", objStruct.distance, coord.x, coord.y, coord.z);

      if (addObjectToDatabase(objStruct)) {
        ROS_INFO("...added object to db");
      }
    }
    return true;
  } else {
    ROS_INFO("findObject OK  - ERROR");
    return false;
  }

  // bool is probably not the right output type, a position seems more relevant
  return true;
}

/***
 * This function call a service using one of the following parameter
 * ALL
 * SHOES_DRINK_INFORMATION
 * BAG_INFORMATION
 * SEAT_INFORMATION
 */
bool findStoreSpecificObjectType(ObjectServiceType type) {
  std::string type_str;
  switch (type) {
    case ALL:
      type_str = "ALL";
      break;
    case SHOES_DRINK_INFORMATION:
      type_str = "SHOES_DRINK_INFORMATION";
      break;
    case BAG_INFORMATION:
      type_str = "BAG_INFORMATION";
      break;
    case SEAT_INFORMATION:
      type_str = "SEAT_INFORMATION";
      break;
    default:
      ROS_ERROR("[findStoreSpecificObjectType] - ERROR, unknown enum type");
      return false;
      break;
  }
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");
  robobreizh_msgs::object_detection_service srv;
  std::vector<std::string> detections{ type_str };
  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.distanceMaximum = 3;
  srv.request.entries_list.obj = tabMsg;
  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    ROS_INFO(" with objects ==  %d", nbObjects);

    if (nbObjects == 0) {
      return false;
    }

    std::vector<std::string> vPersonObj{ // coco
                                         "person", "clothing", "kite",
                                         // OID
                                         "Clothing", "Office building", "Human face", "Human body", "Human head",
                                         "Human arm", "Human hand", "Human nose", "Person", "Man", "Woman", "Boy",
                                         "Girl"
    };

    for (auto obj : objects) {
      // skips if person objects and weird objects
      if (std::find(vPersonObj.begin(), vPersonObj.end(), obj.label.data) != vPersonObj.end()) {
        continue;
      }
      if (obj.color.data.empty()) {
        ROS_WARN("[findStoreSpecificObjectType] No color received ");
      }

      robobreizh::database::Object objStruct;
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "map";
      ps.point.x = (float)obj.coord.x;
      ps.point.y = (float)obj.coord.y;
      ps.point.z = (float)obj.coord.z;

      auto coord = convert_point_stamped_to_frame(ps, "map");

      objectMsgToObjectStruct(&objStruct, obj, coord.point);
      ROS_INFO("[findStoreSpecificObjectType] ...received %s %s", objStruct.color.label.c_str(),
               objStruct.label.c_str());
      ROS_INFO("     distance: %f, position (%f,%f,%f)", objStruct.distance, coord.point.x, coord.point.y,
               coord.point.z);

      if (addObjectToDatabase(objStruct)) {
        ROS_INFO("[findStoreSpecificObjectType] added object to db");
      }
    }
    return true;
  } else {
    ROS_ERROR("findStoreSpecificObjectType - Service call failed");
    return false;
  }
  return true;
}

std::vector<robobreizh_msgs::Object> findAllObjects() {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::object_detection_service>(
      "/robobreizh/perception_pepper/object_detection_service");
  robobreizh_msgs::object_detection_service srv;
  std::vector<std::string> detections;
  detections.push_back("ALL");

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.distanceMaximum = 3;
  srv.request.entries_list.obj = tabMsg;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    ROS_INFO("findAllObjects OK, with objects ==  %d", nbObjects);

    return objects;
  } else {
    ROS_ERROR("[findStoreAllObject] - Service call failed");
    std::vector<robobreizh_msgs::Object> result;
    return result;
  }
}

std::vector<std::string> findObjectsCategories() {

  std::vector<std::string> categories;

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::category_detection_service>(
      "/robobreizh/perception_pepper/category_detection_service");
  robobreizh_msgs::category_detection_service srv;
  std::vector<std::string> detections;
  detections.push_back("ALL");

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.distanceMaximum = 3;
  srv.request.entries_list.obj = tabMsg;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Object> objects = srv.response.outputs_list.object_list;
    int nbObjects = objects.size();
    if (nbObjects < 2) {
      ROS_INFO("Less than 2 objects detected, one category will be empty", nbObjects);
      return categories;
    }
    if (nbObjects > 2) {
      ROS_INFO("More than 2 objects returned, this behavior shouldn't happen!");
      return categories;
    } else{
      /* Get the two objects of the objects list */
      robobreizh_msgs::Object obj1 = objects[0];
      robobreizh_msgs::Object obj2 = objects[1];

      /* Get the category of the two objects */
      std::string category1 = robobreizh::findObjectCategory(obj1.label.data);
      std::string category2 = robobreizh::findObjectCategory(obj2.label.data);

      /*return the two strings category1 and category2 */
      categories.push_back(category1);
      categories.push_back(category2);

      return categories;
    }

  } else {
    ROS_ERROR("[findStoreAllObject] - Service call failed");
    return categories;
  }
}

/*******************************************************************/

#ifdef LEGACY
bool findAndLocateCabDriver() {
  /* Option 1 : umbrella */
  std::vector<std::string> umbrellas{ "umbrella", "Umbrella" };
  std::vector<robobreizh_msgs::Object> objList;
  objList = vision::generic::findAllObjects();
  for (auto elem : objList) {
    for (auto elem2 : umbrellas) {
      if (elem.label.data == elem2) {
        robobreizh::database::Person person;
        person.name = "cabDriver";

        geometry_msgs::Point coord =
            robobreizh::convertOdomToMap((float)elem.coord.x, (float)elem.coord.y, (float)elem.coord.z);
        person.position = coord;

        ROS_INFO("...got cab driver at position (%f,%f,%f)", person.position.x, person.position.y, person.position.z);

        if (addPersonToDatabase(person)) {
          ROS_INFO("...adding cab driver to db");
        } else {
          return false;
        }
        return true;
      }
    }
  }

  /* Option 2 : person with yellow jersey == PAS BLACK*/
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_posture>(
      "/robobreizh/perception_pepper/person_features_detection_posture");

  robobreizh_msgs::person_features_detection_posture srv;

  std::vector<std::string> detections;

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = 100.0;

  if (client.call(srv)) {
    robobreizh_msgs::PersonList persList = srv.response.outputs_list;
    robobreizh_msgs::Person_poseList persPoseList = srv.response.outputs_pose_list;

    std::vector<robobreizh_msgs::Person> persons = persList.person_list;
    std::vector<robobreizh_msgs::Person_pose> personPoses = persPoseList.person_pose_list;
    int nbPersons = persons.size();

    ROS_INFO("findAndLocateCabDriver OK, with nbPerson ==  %d", nbPersons);

    for (int i = 0; i < nbPersons; i++) {
      robobreizh::database::Person person;
      robobreizh_msgs::Person pers = persons[i];
      robobreizh_msgs::Person_pose persPose = personPoses[i];
      geometry_msgs::Point coord =
          robobreizh::convertOdomToMap((float)pers.coord.x, (float)pers.coord.y, (float)pers.coord.z);

      personMsgToPersonStruct(&person, pers, persPose, coord);

      if (person.cloth_color.label != "Black") {
        ROS_INFO("...got cab driver ");

        if (addPersonToDatabase(person)) {
          ROS_INFO("...adding cab driver to db");
        }
      }
    }
    return nbPersons;
  } else {
    ROS_ERROR("findAndLocateCabDriver - ERROR");
    return 0;
  }

  return false;
}
#endif

std::string analyseShelfCategories() {
  std::vector<robobreizh_msgs::Object> objList;
  objList = vision::generic::findAllObjects();
  std::map<std::string, std::string> relativeposes;
  for (auto obj : objList) {
    std::string category;
    std::string position;

    category = robobreizh::findObjectCategory(obj.label.data);
    position = robobreizh::findObjectRange(obj.label.data, obj.coord);

    relativeposes[category] = position;
  }
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj;
  obj = om.getLastObject();
  for (auto elem : relativeposes) {
    auto categoryTmp = robobreizh::findObjectCategory(obj.label);
    if (elem.first == categoryTmp) {
      return elem.second;
    }
  }
  return "";
}

std::string findAndLocateLastObjectPose() {
  std::vector<robobreizh_msgs::Object> objList;
  objList = vision::generic::findAllObjects();
  std::map<std::string, std::string> relativeposes;
  for (auto obj : objList) {
    std::string category;
    std::string position;

    category = robobreizh::findObjectCategory(obj.label.data);
    position = robobreizh::findObjectRange(obj.label.data, obj.coord);

    relativeposes[category] = position;
  }
  robobreizh::database::ObjectModel om;
  robobreizh::database::Object obj;
  obj = om.getLastObject();
  for (auto elem : relativeposes) {
    auto categoryTmp = robobreizh::findObjectCategory(obj.label);
    if (elem.first == categoryTmp) {
      return elem.second;
    }
  }
  return "";
}

/*******************************************************************/
int findHumanAndStoreFeaturesWithDistanceFilter(double distanceMax) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/person_feature_detection");

  robobreizh_msgs::person_features_detection_service srv;
  std::vector<std::string> detections;
  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.obj = tabMsg;
  srv.request.entries_list.distanceMaximum = distanceMax;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    int nbPersons = persons.size();
    bool isAdded = false;
    ROS_INFO("findHumanAndStoreFeaturesWithDistanceFilter OK, with nbPerson ==  %d", nbPersons);

    for (int i = 0; i < nbPersons; i++) {
      robobreizh::database::Person person;

      // message robobreizh_msgs::Person
      robobreizh_msgs::Person pers = persons[i];
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = (float)pers.coord.x;
      ps.point.y = (float)pers.coord.y;
      ps.point.z = (float)pers.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");
      personMsgToPersonStruct(&person, pers, coord.point);

      ROS_INFO("            x : %f", pers.coord.x);
      ROS_INFO("            y : %f", pers.coord.y);
      ROS_INFO("            z : %f", pers.coord.z);
      ROS_INFO("            height : %f", pers.height);

      if (person.gender.empty()) {
        person.gender = "M";
      }

      ROS_INFO(
          "...got person %d : %s clothes, %s years old, %s, %s skin, %s posture, %f height, %f m distance, position "
          "(%f,%f,%f)",
          i, person.cloth_color.label.c_str(), person.age.c_str(), person.gender.c_str(),
          person.skin_color.label.c_str(), person.posture.c_str(), person.height, person.distance, person.position.x,
          person.position.y, person.position.z);

      if (addPersonToDatabase(person)) {
        ROS_INFO("...adding person to db");
      }
    }
    return nbPersons;
  } else {
    ROS_ERROR("findHumanAndStoreFeaturesWihDistanceFilter - ERROR");
    return 0;
  }
  return 0;
}

bool breakTheRules(double distanceMax) {
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/stickler_service");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections{};

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.distanceMaximum = distanceMax;
  srv.request.entries_list.obj = tabMsg;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    int nbObjects = persons.size();
    ROS_INFO("breakTheRules OK - nb person : %d", nbObjects);

    for (int i = 0; i < nbObjects; i++) {
      robobreizh_msgs::Person person = persons[i];
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "odom";
      ps.point.x = person.coord.x;
      ps.point.y = person.coord.y;
      ps.point.z = person.coord.z;
      auto coord = convert_point_stamped_to_frame(ps, "map");

      float distance = person.distance;
      ROS_INFO_STREAM("...Found person: ");
      ROS_INFO_STREAM("   (x,y,z) : (" << coord.point.x << ", " << coord.point.y << ", " << coord.point.z << ")");
      ROS_INFO_STREAM("   distance : " << distance);
      ROS_INFO_STREAM("   Drink: " << (int)person.is_drink << ", Shoes: " << (int)person.is_shoes);
      robobreizh::database::Person person_struct;
      personMsgToPersonStruct(&person_struct, person, coord.point);

      addPersonToDatabase(person_struct);
    }
  } else {
    ROS_ERROR("Shoes and drinks service couldn t be called");
    return false;
  }
  return true;
}

/**
 * @brief  Call the shoes and drink service and return a true if the person is holding a drink
 * @param distance_max: maximum distance to detect a person
 */
bool findHumanWithDrink(float distance_max) {
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<robobreizh_msgs::person_features_detection_service>(
      "/robobreizh/perception_pepper/stickler_service");

  robobreizh_msgs::person_features_detection_service srv;

  std::vector<std::string> detections{};

  std::vector<std_msgs::String> tabMsg = robobreizh::fillTabMsg(detections);

  srv.request.entries_list.distanceMaximum = distance_max;
  srv.request.entries_list.obj = tabMsg;
  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    int nbObjects = persons.size();
    ROS_INFO("findHumanWithDrink OK - nb person : %d", nbObjects);

    for (int i = 0; i < nbObjects; i++) {
      robobreizh_msgs::Person person = persons[i];
      ROS_INFO_STREAM("...Found person: ");
      ROS_INFO_STREAM("   Drink: " << (int)person.is_drink << ", Shoes: " << (int)person.is_shoes);
      if (person.is_drink) {
        return true;
      }
    }
  } else {
    ROS_ERROR("/robobreizh/perception_pepper/stickler_service service couldn t be called");
    return false;
  }
  return false;
}

/**
 * @brief Call the shoes and drink service and return a true if the person is wearing shoes
 * @param distance_max : max distance to detect the person
 */
bool findHumanWithShoes(float distance_max) {
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<robobreizh_msgs::shoes_detection>("/robobreizh/perception_pepper/shoes_detection");

  robobreizh_msgs::shoes_detection srv;

  srv.request.distance_max = distance_max;

  if (client.call(srv)) {
    std::vector<robobreizh_msgs::Person> persons = srv.response.outputs_list.person_list;
    int nbObjects = persons.size();
    ROS_INFO("findHumanWithDrink OK - nb person : %d", nbObjects);

    for (int i = 0; i < nbObjects; i++) {
      robobreizh_msgs::Person person = persons[i];
      ROS_INFO_STREAM("...Found person: ");
      ROS_INFO_STREAM("   Shoes: " << (int)person.is_shoes);
      if (person.is_shoes) {
        return true;
      }
    }
  } else {
    ROS_ERROR("/robobreizh/perception_pepper/shoes_detection service couldn t be called");
    return false;
  }
  return false;
}

}  // namespace generic
}  // namespace vision
}  // namespace robobreizh
