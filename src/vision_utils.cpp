#include "robobreizh_msgs/Person.h"
#include "robobreizh_msgs/Object.h"
#include "robobreizh_msgs/PersonPose.h"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include "vision_utils.hpp"
#include "manager_utils.hpp"

namespace robobreizh {
void personMsgToPersonStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                             geometry_msgs::Point coord) {
  person->gender = pers.gender.data;
  person->age = pers.age.data;
  person->skin_color = { pers.skin_color.data };
  person->distance = (float)pers.distance;
  person->cloth_color = { pers.clothes_color.data };
  person->clothes_style = pers.clothes_style.data;
  person->position = coord;
}

void personMsgToPersonPoseStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                                 robobreizh_msgs::PersonPose persPose, geometry_msgs::Point coord) {
  person->gender = pers.gender.data;
  person->age = pers.age.data;
  person->skin_color = { pers.skin_color.data };
  person->distance = (float)pers.distance;
  person->cloth_color = { pers.clothes_color.data };
  person->clothes_style = pers.clothes_style.data;

  // message robobreizh_msgs::Person_pose
  person->posture = persPose.posture.data;
  person->height = persPose.height;

  person->position = coord;
}

void objectMsgToObjectStruct(robobreizh::database::Object* object, robobreizh_msgs::Object objectMsg,
                             geometry_msgs::Point coord) {
  object->label = objectMsg.label.data;
  object->color = { objectMsg.color.data };
  object->position = coord;
  float distance = objectMsg.distance;
  object->room = { "living room" };
}

std::vector<std_msgs::String> fillTabMsg(std::vector<std::string> detections) {
  std::vector<std_msgs::String> tabMsg;
  for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << *t;
    msg.data = ss.str();
    tabMsg.push_back(msg);
  }
  return tabMsg;
}

int isInForbiddenRoom(float x, float y) {
  geometry_msgs::Point coord1;
  geometry_msgs::Point coord2;
  coord1.x = -2.785;
  coord1.y = 8.762;
  coord2.x = 0.928;
  coord2.y = 13.203;

  if (x > coord1.x and x < coord2.x and y > coord1.y and y < coord2.y) {
    return true;
  }
  return false;
}

bool addPersonToDatabase(robobreizh::database::Person person) {
  robobreizh::database::PersonModel pm;
  auto allPerson = pm.getPersons();
  // loop over allPerson
  bool alreadyExist = false;
  for (auto dbPerson : allPerson) {
    if (isInRadius(dbPerson.position.x, dbPerson.position.y, dbPerson.position.z, person.position.x, person.position.y,
                   person.position.z, 0.2)) {
      alreadyExist = true;
    }
  }

  if (!alreadyExist) {
    pm.insertPerson(person);
    return true;
  }

  return false;
}

std::string findObjectCategory(std::string object) {
  try {
    switch (robobreizh::object_category[object]) {
      case robobreizh::ObjectCategory::Fruit:
        return "fruit";
        break;
      case robobreizh::ObjectCategory::Vegetable:
        return "vegetable";
        break;
      case robobreizh::ObjectCategory::OtherFood:
        return "other";
        break;
      default:
        return "none";
        break;
    }
  } catch (std::out_of_range e) {
    return "none";
  }
}

std::string findObjectRange(std::string object, geometry_msgs::Point32 coord) {
  if ((coord.y > 1.4) && (coord.y < 1.6)) {
    return "Shelf 1";
  }
  if ((coord.y > 1.6) && (coord.y < 1.8)) {
    return "Shelf 2";
  }
  if ((coord.y > 1.8) && (coord.y < 1.8)) {
    return "Shelf 3";
  }
  return "";
}

geometry_msgs::PointStamped convert_point_stamped_to_frame(geometry_msgs::PointStamped point,
                                                           std::string frame_destination) {
  geometry_msgs::PointStamped destination_point;
  try {
    ROS_INFO_STREAM("Converting between" << point.header.frame_id << " and " << frame_destination);
    ROS_INFO("point in odom frame: (%.2f, %.2f. %.2f)", point.point.x, point.point.y, point.point.z);
    destination_point = tfBuffer.transform(point, frame_destination);
    ROS_INFO("point in map frame: (%.2f, %.2f. %.2f)", destination_point.point.x, destination_point.point.y,
             destination_point.point.z);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    // TODO: handle case where no transform is found
  }

  return destination_point;
}

bool isInRadius(float x1, float y1, float z1, float x2, float y2, float z2, float epsilon) {
  float distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
  std::cout << "	Calculated distance : " << std::to_string(distance) << std::endl;
  if (distance < epsilon) {
    return true;
    std::cout << "	distance is smaller than " << std::to_string(epsilon) << std::endl;
  }
  return false;
}

bool addObjectToDatabase(robobreizh::database::Object obj) {
  robobreizh::database::ObjectModel om;
  // get all objects with label
  std::vector<robobreizh::database::Object> dbObjects = om.getObjectByLabel(obj.label);

  // loop over dbObjects
  bool alreadyExist = false;
  for (auto dbObj : dbObjects) {
    if (isInRadius(dbObj.position.x, dbObj.position.y, dbObj.position.z, obj.position.x, obj.position.y, obj.position.z,
                   0.2)) {
      alreadyExist = true;
    }
  }

  if (!alreadyExist) {
    om.insertObject(obj);
    return true;
  }
  return false;
}

}  // namespace robobreizh
