#include "database_model/location_model.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
namespace database
{
LocationModel::LocationModel() : Database()
{
}
LocationModel::~LocationModel() : Database()
{
}

/**
 * @brief Create location table in the database
 */
void LocationModel::createTable()
{
  db.exec("CREATE TABLE IF NOT EXISTS location (
          name TEXT PRIMARY KEY UNIQUE NOT NULL,
          frame TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL, qw REAL NOT NULL, qx REAL NOT NULL,
          qy REAL NOT NULL, qz REAL NOT NULL, angle REAL) ");
}

/**
 * @brief Insert a location in the database
 * @param location Location to insert
 */
void LocationModel::insertLocation(Location location)
{
  db.exec("INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle) VALUES (" + location.name + ", " +
          location.frame + ", " + location.pose.position.x + ", " + location.pose.position.y + ", " +
          location.pose.position.z + ", " + location.pose.orientation.qw + ", " + location.pose.orientation.qx + ", " +
          location.pose.orientation.qy + ", " + location.pose.orientation.qz + ", " + location.angle + ")");
}

/**
 * @brief Insert a location in the database
 * @param name Name of the location
 * @param frame Frame id of the location
 * @param pose Pose of the location
 * @param angle Angle of the pose
 */
void LocationModel::insertLocation(std::string name, std::string frame, geometry_msgs::Pose pose, float angle)
{
  db.exec("INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle) VALUES (" + name + ", " + frame + ", " +
          pose.position.x + ", " + pose.position.y + ", " + pose.position.z + ", " + pose.orientation.qw + ", " +
          pose.orientation.qx + ", " + pose.orientation.qy + ", " + pose.orientation.qz + ", " + angle + ")");
}

/**
 * @brief Update a location in the database
 * @param location Location to update
 */
void LocationModel::updateLocation(Location location)
{
  db.exec("UPDATE location SET name = " + location.name + ", frame = " + location.frame +
          ", x = " + location.pose.position.x + ", y = " + location.pose.position.y +
          ", z = " + location.pose.position.z + ", qw = " + location.pose.orientation.qw +
          ", qx = " + location.pose.orientation.qx + ", qy = " + location.pose.orientation.qy +
          ", qz = " + location.pose.orientation.qz + ", angle = " + location.angle + " WHERE name = " + location.name);
}

/**
 * @brief Delete a location in the database
 * @param location Location to delete
 */
void LocationModel::deleteLocation(std::string location_name)
{
  db.exec("DELETE FROM location WHERE name = \"" + location_name + "\"");
}

/**
 * @brief Clear all locations in the database
 *
 */
void clearLocation()
{
  db.exec("DELETE FROM location");
}

/**
 * @brief Get all locations from the database
 * @return std::vector<Location> Vector of locations
 */
std::vector<Location> LocationModel::getAllLocations()
{
  std::vector<Location> locations;
  SQLite::Statement query(db, "SELECT * FROM location");
  while (query.executeStep())
  {
    Location location;
    location.name = query.getColumn(0).getText();
    location.frame = query.getColumn(1).getText();
    location.pose.position.x = query.getColumn(2).getDouble();
    location.pose.position.y = query.getColumn(3).getDouble();
    location.pose.position.z = query.getColumn(4).getDouble();
    location.pose.orientation.w = query.getColumn(5).getDouble();
    location.pose.orientation.x = query.getColumn(6).getDouble();
    location.pose.orientation.y = query.getColumn(7).getDouble();
    location.pose.orientation.z = query.getColumn(8).getDouble();
    location.angle = query.getColumn(9).getDouble();
    locations.push_back(location);
  }
  return locations;
}

/**
 * @brief Get a location from its name
 * @param location_name Name of the location to get
 * @return Location
 */
Location LocationModel::getLocationFromName(std::string location_name)
{
  Location location;
  SQLite::Statement query(db, "SELECT * FROM location WHERE name = \"" + location_name + "\"");
  while (query.executeStep())
  {
    location.name = query.getColumn(0).getText();
    location.frame = query.getColumn(1).getText();
    location.pose.position.x = query.getColumn(2).getDouble();
    location.pose.position.y = query.getColumn(3).getDouble();
    location.pose.position.z = query.getColumn(4).getDouble();
    location.pose.orientation.w = query.getColumn(5).getDouble();
    location.pose.orientation.x = query.getColumn(6).getDouble();
    location.pose.orientation.y = query.getColumn(7).getDouble();
    location.pose.orientation.z = query.getColumn(8).getDouble();
    location.angle = query.getColumn(9).getDouble();
  }
  return location;
}
}  // namespace database
}  // namespace robobreizh
