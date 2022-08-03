#ifndef _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#include "database_model/location_model.hpp"
#include "database_model/database.hpp"
#include "database_model/sqlite_wrapper.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

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
} Location;

class LocationModel : Database
{
public:
  LocationModel();
  virtual ~LocationModel();
  void createTable();
  void insertLocation(Location location);
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
