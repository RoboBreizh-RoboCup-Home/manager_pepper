#include "database_model/database_utils.hpp"
#include "database_model/location_model.hpp"
#include <vector>

int main()
{
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
