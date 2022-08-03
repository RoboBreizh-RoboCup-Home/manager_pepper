#ifndef _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#include "database_model/location_model.hpp"
#include "database_model/database.hpp"
#include <string>
#include <geometry_msgs/Pose.h>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{

typedef struct
{
  std::string name;
  std::string frame;
  geometry_msgs::Pose pose;
  float angle;
  std::ostream& operator<<(std::ostream& os, const A& value)
  {
    os << "name: " << value.name << ", frame id: " << value.frame << ", Point(" << value.pose.position.x << ", "
       << value.pose.position.y << ", " << value.pose.position.z << "), orientation(" << value.pose.orientation.x
       << ", " << value.pose.orientation.y << ", " << value.pose.orientation.z << ", " << value.pose.orientation.w
       << "), angle: " << angle << std::endl;
    return os;
  }
} Location;

class LocationModel : Database
{
public:
  LocationModel();
  virtual ~LocationModel();
  void createTable();
  void insertLocation(Location location);
  void insertLocation(std::string name, std::string frame, geometry_msgs::Pose pose, float angle);
  void updateLocation(Location location);
  void deleteLocation(std::string location_name);
  void clearLocation();
  std::vector<Location> getAllLocations();
  Location getLocationFromName(std::string location_name);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
