#include "database_model/database_utils.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace robobreizh {
namespace database {

/**
 * @brief operator overload to print Color
 */
std::ostream& operator<<(std::ostream& os, const Color& value) {
  os << "label: " << value.label;
  return os;
}

/**
 * @brief operator overload to print Room
 */
std::ostream& operator<<(std::ostream& os, const Room& value) {
  os << "label: " << value.label;
  return os;
}

/**
 * @brief operator overload to print geometry_msgs/Point struct
 */
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point& value) {
  os << "Position (" << value.x << ", " << value.y << ", " << value.z << ")";
  return os;
}

/**
 * @brief operator overload to print geometry_msgs/Quaternion struct
 */
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Quaternion& value) {
  os << "Orientation (" << value.x << ", " << value.y << ", " << value.z << ", " << value.w << ")";
  return os;
}

/**
 * @brief operator overload to print person struct
 */
std::ostream& operator<<(std::ostream& os, const Person& value) {
  os << "name: " << value.name << ", drink: " << value.favorite_drink << ", gender: " << value.gender
     << ", age: " << value.age << ", cloth color: " << value.cloth_color << ", skin color: " << value.skin_color
     << ", posture: " << value.posture << ", height: " << value.height << ", " << value.position
     << ", distance: " << value.distance;
  return os;
}

/**
 * @brief operator overload to print object
 */
std::ostream& operator<<(std::ostream& os, const Object& value) {
  os << "label: " << value.label << ", color: " << value.color.label << ", " << value.position
     << ", distance: " << value.distance << ", room: " << value.room;
  return os;
}

/**
 * @brief operator overload to print location
 */
std::ostream& operator<<(std::ostream& os, const Location& value) {
  os << "name: " << value.name << ", frame id: " << value.frame << ", Position(" << value.pose.position << ", "
     << value.pose.orientation << "), angle: " << value.angle << ", room: " << value.room;
  return os;
}

std::ostream& operator<<(std::ostream& os, const GPSRVariation& value) {
  if (value.item_context != "") os << "item_content: " << value.item_context;
  if (value.descr_verb != "") os << ", descr_verb: " << value.descr_verb;
  if (value.descr_adj != "") os << ", descr_adj: " << value.descr_adj;
  if (value.descr_key != "") os << ", descr_key: " << value.descr_key;
  if (value.descr != "") os << ", descr: " << value.descr;
  if (value.pos != "") os << ", pos: " << value.pos;
  if (value.pos_adj != "") os << ", pos_adj: " << value.pos_adj;
  if (value.dest_per != "") os << ", dest_per: " << value.dest_per;
  return os;
}

/**
 * @brief operator overload to print GPSRAction
 */
std::ostream& operator<<(std::ostream& os, const GPSRAction& value) {
  os << "intent: " << value.intent << ", object_item: " << value.object_item << ", person: " << value.person
     << ", destination: " << value.destination << ", source: " << value.source;
  return os;
}

/**
 * @brief operator overload to print Stickler
 */
std::ostream& operator<<(std::ostream& os, const Stickler& value) {
  os << "Shoes: " << value.Shoes << ", drink: " << value.drink << ", ForbiddenRoom: " << value.ForbiddenRoom
     << ", Littering: " << value.Littering;
  return os;
}
}  // namespace database
}  // namespace robobreizh
