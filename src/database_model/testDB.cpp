#include "database_model/database_utils.hpp"
#include "database_model/location_model.hpp"
#include "database_model/color_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/gpsr_actions_model.hpp"
#include "database_model/person_model.hpp"
#include "sqlite_utils.hpp"
#include <vector>

#include <warehouse_ros_sqlite/database_connection.h>
#include <warehouse_ros_sqlite/utils.h>

warehouse_ros_sqlite::DatabaseConnection* robobreizh::SQLiteUtils::conn_ =
    new warehouse_ros_sqlite::DatabaseConnection();
int main() {
  robobreizh::database::LocationModel lm;
  lm.createTable();
  /* auto locations = lm.getAllLocations(); */
  /* for (auto value : locations) */
  /* { */
  /*     std::cout  << "name: " << value.name << ", frame id: " << value.frame << ", Point(" << value.pose.position.x <<
   * ", " */
  /*    << value.pose.position.y << ", " << value.pose.position.z << "), orientation(" << value.pose.orientation.x */
  /*    << ", " << value.pose.orientation.y << ", " << value.pose.orientation.z << ", " << value.pose.orientation.w */
  /*    << "), angle: " << value.angle << std::endl; */
  /* } */
  return 0;
}
