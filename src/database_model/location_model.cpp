#include "database_model/location_model.hpp"
#include <string>
#include <iostream>
#include <vector>
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
  db.exec(R"(CREATE TABLE IF NOT EXISTS location (
          name TEXT PRIMARY KEY UNIQUE NOT NULL,
          frame TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL, qw REAL NOT NULL, qx REAL NOT NULL,
          qy REAL NOT NULL, qz REAL NOT NULL, angle REAL) )");
}

/**
 * @brief Insert a location in the database
 * @param location Location to insert
 */
void LocationModel::insertLocation(Location location)
{
  SQLite::Statement query(db,R"(
  INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle) VALUES (?,?,?,?,?,?,?,?,?,?)");
  query.bind(1, location.name);
  query.bind(2, location.frame);
  query.bind(3, location.pose.position.x);
  query.bind(4, location.pose.position.y);
  query.bind(5, location.pose.position.z);
  query.bind(6, location.pose.orientation.w);
  query.bind(7, location.pose.orientation.x);
  query.bind(8, location.pose.orientation.y);
  query.bind(9, location.pose.orientation.z);
  query.bind(10, location.angle);
  query.exec();
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
  SQLite::Statement query(db,R"(
  INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle) VALUES (?,?,?,?,?,?,?,?,?,?))");
  query.bind(1, name);
  query.bind(2, frame);
  query.bind(3, pose.position.x);
  query.bind(4, pose.position.y);
  query.bind(5, pose.position.z);
  query.bind(6, pose.orientation.w);
  query.bind(7, pose.orientation.x);
  query.bind(8, pose.orientation.y);
  query.bind(9, pose.orientation.z);
  query.bind(10, angle);
  query.exec();
}

/**
 * @brief Update a location in the database
 * @param location Location to update
 */
void LocationModel::updateLocation(Location location)
{
  SQLite::Statement query(db,R"(
  UPDATE location SET frame = ?, x = ?, y = ?, z = ?, qw = ?, qx = ?, qy = ?, qz = ?, angle = ? WHERE name = ?)");
  query.bind(1, location.frame);
  query.bind(2, location.pose.position.x);
  query.bind(3, location.pose.position.y);
  query.bind(4, location.pose.position.z);
  query.bind(5, location.pose.orientation.w);
  query.bind(6, location.pose.orientation.x);
  query.bind(7, location.pose.orientation.y);
  query.bind(8, location.pose.orientation.z);
  query.bind(9, location.angle);
  query.bind(10, location.name);
  query.exec();
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
void LocationModel::clearLocation()
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
