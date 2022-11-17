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
typedef struct Color
{
  std::string label = "";
} Color;

typedef struct Room
{
  std::string label = "";
} Room;

typedef struct Speech
{
  std::string transcript = "";
} Speech;

/**
 * @brief struct representation of person table
 */
typedef struct Person
{
  std::string name = "";
  std::string favorite_drink = "";
  std::string gender = "";
  std::string age = "";
  Color cloth_color = { "" };
  Color skin_color = { "" };
  std::string posture = "";
  float height = 0.0;
  geometry_msgs::Point position;
  float distance = 0.0;
} Person;

/**
 * @brief struct representation of object table
 */
typedef struct Object
{
  std::string label = "";
  Color color = { "" };
  geometry_msgs::Point position;
  float distance = 0.0;
  Room room = { "" };
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
 * @brief struct representation of Location table
 */
typedef struct Location
{
  std::string name = "";
  std::string frame = "";
  geometry_msgs::Pose pose;
  float angle = 0.0;
  Room room = { "" };
} Location;

/**
 * @brief struct representation of GPSRAction table
 */
typedef struct GPSRAction
{
  std::string intent = "";
  std::string object_item = "";
  std::string person = "";
  std::string destination = "";
  std::string who = "";
  std::string what = "";
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
 * @brief struct representation of Stickler table
 */
typedef struct Stickler
{
  bool Shoes;
  bool drink;
  bool ForbiddenRoom;
  bool Littering;
} Stickler;

std::ostream& operator<<(std::ostream& os, const Color& value);
std::ostream& operator<<(std::ostream& os, const Room& value);
std::ostream& operator<<(std::ostream& os, const Speech& value);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point& value);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Quaternion& value);
std::ostream& operator<<(std::ostream& os, const Person& value);
std::ostream& operator<<(std::ostream& os, const Object& value);
std::ostream& operator<<(std::ostream& os, const Location& value);
std::ostream& operator<<(std::ostream& os, const GPSRAction& value);
std::ostream& operator<<(std::ostream& os, const Stickler& value);

}  // namespace database
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_UTILS_DATABASE_MODEL_
