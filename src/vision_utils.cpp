#include "robobreizh_msgs/Person.h"
#include "robobreizh_msgs/Object.h"
#include "robobreizh_msgs/PersonPose.h"
#include "database_model/person_model.hpp"
#include "database_model/object_model.hpp"

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include "vision_utils.hpp"
#include "manager_utils.hpp"

namespace robobreizh {
void personMsgToPersonPoseStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                             robobreizh_msgs::PersonPose persPose, geometry_msgs::Point coord) {
  person->gender = pers.gender.data;
  person->age = pers.age.data;
  person->skin_color = { pers.skin_color.data };
  person->distance = (float)pers.distance;
  person->cloth_color = { pers.clothes_color.data };

  // message robobreizh_msgs::Person_pose
  person->posture = persPose.posture.data;
  person->height = persPose.height;

  person->position = coord;
}

void personMsgToPersonStruct(robobreizh::database::Person* person, robobreizh_msgs::Person pers,
                             geometry_msgs::Point coord) {
  person->gender = pers.gender.data;
  person->age = pers.age.data;
  person->skin_color = { pers.skin_color.data };
  person->distance = (float)pers.distance;
  person->cloth_color = { pers.clothes_color.data };

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
  // print detections
  std::cout << "		[";
  for (std::vector<std::string>::iterator t = detections.begin(); t != detections.end(); t++) {
    std::cout << *t << ", ";
  }
  std::cout << "]" << std::endl;
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

float convertOdomToBaseFootprint(float odomx, float odomy, float odomz) {
  geometry_msgs::Point odomPoint;
  odomPoint.x = odomx;
  odomPoint.y = odomy;
  odomPoint.z = odomz;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  try {
    std::cout << tfBuffer.canTransform("base_footprint", "odom", ros::Time(0.0), ros::Duration(3.0)) << std::endl;
    transformStamped = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time(0.0), ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Point mapPoint;
  tf2::doTransform(odomPoint, mapPoint, transformStamped);
  // ROS_INFO("      transformStamped odom -> base_footprint: ");
  // ROS_INFO("      odomPoint * transformStamped = base_footprintPoint: ");
  // ROS_INFO("            x : %f     x : %f     %f", odomPoint.x, transformStamped.transform.translation.x,
  // mapPoint.x); ROS_INFO("            y : %f  X  y : %f  =  %f", odomPoint.y,
  // transformStamped.transform.translation.y, mapPoint.y); ROS_INFO("            z : %f     z : %f     %f",
  // odomPoint.z, transformStamped.transform.translation.z, mapPoint.z);
  double yaw_angle = tf::getYaw(transformStamped.transform.rotation);

  return float(yaw_angle);
}

geometry_msgs::Point convertOdomToMap(float odomx, float odomy, float odomz) {
  geometry_msgs::Point odomPoint;
  odomPoint.x = odomx;
  odomPoint.y = odomy;
  odomPoint.z = odomz;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  try {
    std::cout << tfBuffer.canTransform("map", "odom", ros::Time(0.0), ros::Duration(3.0)) << std::endl;
    transformStamped = tfBuffer.lookupTransform("map", "odom", ros::Time(0.0), ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Point mapPoint;
  tf2::doTransform(odomPoint, mapPoint, transformStamped);
  // ROS_INFO("      transformStamped odom -> map: ");
  // ROS_INFO("      odomPoint * transformStamped = mapPoint: ");
  // ROS_INFO("            x : %f     x : %f     %f", odomPoint.x, transformStamped.transform.translation.x,
  // mapPoint.x); ROS_INFO("            y : %f  X  y : %f  =  %f", odomPoint.y,
  // transformStamped.transform.translation.y, mapPoint.y); ROS_INFO("            z : %f     z : %f     %f",
  // odomPoint.z, transformStamped.transform.translation.z, mapPoint.z);

  return mapPoint;
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
