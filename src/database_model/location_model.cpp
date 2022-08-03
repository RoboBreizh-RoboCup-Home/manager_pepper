#include "database_model/location_model.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
namespace database
{
LocationModel::LocationModel()
{
}
LocationModel::~LocationModel()
{
}

/**
 * @brief Create location table in the database
 */
void LocationModel::createTable()
{
  static const char create_table_query[] = R"(CREATE TABLE IF NOT EXISTS location (
    name TEXT PRIMARY KEY UNIQUE NOT NULL,
    frame TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    z REAL NOT NULL, 
    qw REAL NOT NULL,
    qx REAL NOT NULL, 
    qy REAL NOT NULL, 
    qz REAL NOT NULL,
    angle REAL))";
  db::query<create_table_query>();
}

/**
 * @brief Insert a location in the database
 * @param location Location to insert
 */
void LocationModel::insertLocation(Location location)
{
  static const char insert_query[] = R"(INSERT INTO location (
    name,
    frame,
    x,
    y,
    z,
    qw,
    qx,
    qy,
    qz,
    angle)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?))";
  db::query<insert_query>(location.name, location.frame, location.pose.position.x, location.pose.position.y,
                          location.pose.position.z, location.pose.orientation.w, location.pose.orientation.x,
                          location.pose.orientation.y, location.pose.orientation.z, location.angle);
}

/**
 * @brief Update a location in the database
 * @param location Location to update
 */
void LocationModel::updateLocation(Location location)
{
  static const char update_query[] = R"(UPDATE location SET
    frame = ?,
    x = ?,
    y = ?,
    z = ?,
    qw = ?,
    qx = ?,
    qy = ?,
    qz = ?,
    angle = ?
    WHERE name = ?)";
  db::query<update_query>(location.frame, location.pose.position.x, location.pose.position.y, location.pose.position.z,
                          location.pose.orientation.w, location.pose.orientation.x, location.pose.orientation.y,
                          location.pose.orientation.z, location.angle, location.name);
}

/**
 * @brief Delete a location in the database
 * @param location Location to delete
 */
void LocationModel::deleteLocation(std::string location_name)
{
  static const char delete_query[] = R"(DELETE FROM location WHERE name = ?)";
  db::query<delete_query>(location_name);
}

/**
 * @brief Clear all locations in the database
 *
 */
void clearLocation()
{
  static const char clear_query[] = R"(DELETE FROM location)";
  db::query<clear_query>();
}

/**
 * @brief Get all locations from the database
 * @return std::vector<Location> Vector of locations
 */
std::vector<Location> LocationModel::getAllLocations()
{
  static const char select_query[] = R"(SELECT * FROM location)";
  std::vector<Location> locations;
  db::query<select_query>(locations);
  return locations;
}

/**
 * @brief Get a location from its name
 * @param location_name Name of the location to get
 * @return Location
 */
Location LocationModel::getLocationFromName(std::string location_name)
{
  static const char select_query[] = R"(SELECT * FROM location WHERE name = (?))";
  Location location;
  db::query<select_query>(location_name, location.name, location.frame, location.pose.position.x,
                          location.pose.position.y, location.pose.position.z, location.pose.orientation.w,
                          location.pose.orientation.x, location.pose.orientation.y, location.pose.orientation.z,
                          location.angle);
  return location;
}
}  // namespace database
}  // namespace robobreizh
