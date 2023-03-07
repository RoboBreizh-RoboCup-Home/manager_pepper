#ifndef _PNP_ROBOBREIZH_LOCATION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_LOCATION_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <geometry_msgs/Pose.h>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh {
namespace database {
class LocationModel : Database {
public:
  LocationModel();
  virtual ~LocationModel();
  void createTable();
  void insertLocation(robobreizh::database::Location location);
  void insertLocation(std::string name, std::string frame, geometry_msgs::Pose pose, float angle);
  void updateLocation(robobreizh::database::Location location);
  void deleteLocation(std::string location_name);
  void clearLocation();
  std::vector<robobreizh::database::Location> getAllLocations();
  robobreizh::database::Location getLocationFromName(std::string location_name);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_LOCATION_DATABASE_MODEL_
