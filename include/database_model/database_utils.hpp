#ifndef _PNP_ROBOBREIZH_UTILS_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_UTILS_DATABASE_MODEL_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

/**
 * This file contains structures and structures operators.
 * Each structure representes a table in the database.
 * Some don't have id because they are autoincrement.
 */
namespace robobreizh
{
namespace database
{

/**
 * @brief operator overload to print geometry_msgs/Point struct
 */
std::ostream& operator<<(std::ostream& os, const Point& value)
{
  os << "Position (" << value.x << ", " << value.y << ", " << value.z << ")";
  os << return os;
}

/**
 * @brief operator overload to print geometry_msgs/Quaternion struct
 */
std::ostream& operator<<(std::ostream& os, const Quaternion& value)
{
  os << "Orientation (" << value.x << ", " << value.y << ", " << value.z << ", " << value.w << ")";
  os << return os;
}

/**
 * @brief struct representation of person table
 */
typedef struct Person
{
  std::string name;
  std::string favorite_drink;
  std::string gender;
  std::string age;
  Color cloth_color;
  Color skin_color;
  std::string posture;
  float height;
  Point position;
  float distance;
} Person;
/**
 * @brief operator overload to print person struct
 */
std::ostream& operator<<(std::ostream& os, const Person& value)
{
  os << "name: " << value.name << ", drink: " << value.favorite_drink << ", gender: " << value.gender
     << ", age: " << value.age << ", cloth color: " << value.cloth_color << ", skin color: " << value.skin_color
     << ", posture: " << value.posture << ", height: " << value.height << ", " << value.position
     << ", distance: " << value.distance;
  return os;
}

/**
 * @brief struct representation of object table
 */
typedef struct Object
{
  std::string label;
  Color color;  // color label retrieve
  Point position;
  float distance;
  Room room;
  bool operator<(const Object& rhs) const
  {
    return distance < rhs.distance;
  };
  bool operator>(const Object& rhs) const
  {
    return distance > rhs.distance;
  };
} Object;

/**
 * @brief operator overload to print object
 */
std::ostream& operator<<(std::ostream& os, const Object& value)
{
  os << "label: " << value.label << ", color: " << value.color.label << ", " << value.position
     << ", distance: " << value.distance << ", room: " << value.room;
  return os;
}

/**
 * @brief struct representation of Location table
 */
typedef struct Location
{
  std::string name;
  std::string frame;
  geometry_msgs::Pose pose;
  float angle;
  Room room;
} Location;

/**
 * @brief operator overload to print location
 */
std::ostream& operator<<(std::ostream& os, const Location& value)
{
  os << "name: " << value.name << ", frame id: " << value.frame << ", Position(" << value.pose.position << ", "
     << value.pose.orientation << "), angle: " << value.angle << ", room: " << value.room;
  return os;
}

typedef struct Color
{
  std::string label;
} Color;

/**
 * @brief operator overload to print Room
 */
std::ostream& operator<<(std::ostream& os, const Color& value)
{
  os << "label: " << value.label;
  return os;
}

typedef struct Room
{
  std::string label;
} Room;

/**
 * @brief operator overload to print Room
 */
std::ostream& operator<<(std::ostream& os, const Room& value)
{
  os << "label: " << value.label;
  return os;
}

/**
 * @brief struct representation of GPSRAction table
 */
typedef struct GPSRAction
{
  std::string intent;
  std::string object_item;
  std::string person;
  std::string destination;
  std::string who;
  std::string what;
} GPSRAction;

enum GPSRActionItemName
{
  intent,
  object_item,
  person,
  destination,
  who,
  what
};

/**
 * @brief operator overload to print GPSRAction
 */
std::ostream& operator<<(std::ostream& os, const GPSRAction& value)
{
  os << "intent: " << value.intent << ", object_item: " << value.object_item << ", person: " << value.person
     << ", destination: " << value.destination << ", who: " << value.who << ", what: " << value.what;
  return os;
}

/**
 * @brief struct representation of Stickler table
 */
typedef struct Stickler
{
  bool Shoes;
  bool drink;
  bool ForbiddenRoom;
  bool Littering;
} Stickler;

/**
 * @brief operator overload to print Stickler
 */
std::ostream& operator<<(std::ostream& os, const Stickler& value)
{
  os << "Shoes: " << value.Shoes << ", drink: " << value.drink << ", ForbiddenRoom: " << value.ForbiddenRoom
     << ", Littering: " << value.Littering;
  return os;
}

}  // namespace database
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_UTILS_DATABASE_MODEL_