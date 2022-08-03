#ifndef _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
typedef struct NavigationPlace
{
  std::string name;
  std::string frame;
  geometry_msgs::Pose pose;
  float angle;
} NavigationPlace;

namespace database
{
class NavigationModel : Database
{
public:
  NavigationModel();
  virtual ~NavigationModel();
  NavigationPlace getLocationFromName(std::string location_name);

protected:
  std::string query;
  sqlite3_stmt* pStmt;

private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_NAVIGATION_DATABASE_MODEL_
