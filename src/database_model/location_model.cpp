#include "database_model/location_model.hpp"
#include "database_model/room_model.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

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
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS location (
    name TEXT PRIMARY KEY UNIQUE NOT NULL,
    frame TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    z REAL NOT NULL, 
    qw REAL NOT NULL,
    qx REAL NOT NULL, 
    qy REAL NOT NULL, 
    qz REAL NOT NULL,
    angle REAL NOT NULL,
    room_id INTEGER NOT NULL,
    FOREIGN KEY(room_id) REFERENCES room(id)
)");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert a location in the database
 * @param location Location to insert
 */
void LocationModel::insertLocation(Location location)
{
  try
  {
    RoomModel rm;
    int room_id = rm.getRoomId(location.room);
    SQLite::Statement query(db, R"(
  INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle, room_id) VALUES (?,?,?,?,?,?,?,?,?,?,?)");
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
    query.bind(11, location.room_id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
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
  try
  {
    RoomModel rm;
    int room_id = rm.getRoomId(name);
    SQLite::Statement query(db, R"(
  INSERT INTO location (name, frame, x, y, z, qw, qx, qy, qz, angle,room_id) VALUES (?,?,?,?,?,?,?,?,?,?))");
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
    query.bind(11, room_id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update a location in the database
 * @param location Location to update
 */
void LocationModel::updateLocation(Location location)
{
  try
  {
    RoomModel rm;
    int room_id = rm.getRoomId(location.room);
    SQLite::Statement query(db, R"(
  UPDATE location SET frame = ?, x = ?, y = ?, z = ?, qw = ?, qx = ?, qy = ?, qz = ?, angle = ?, room_id = ? WHERE name = ?)");
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
    query.bind(11, room_id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete a location in the database
 * @param location Location to delete
 */
void LocationModel::deleteLocation(std::string location_name)
{
  try
  {
    db.exec("DELETE FROM location WHERE name = \"" + location_name + "\"");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Clear all locations in the database
 *
 */
void LocationModel::clearLocation()
{
  try
  {
    db.exec("DELETE FROM location");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Get all locations from the database
 * @return std::vector<Location> Vector of locations
 */
std::vector<Location> LocationModel::getAllLocations()
{
  try
  {
    std::vector<Location> locations;
    SQLite::Statement query(db, "SELECT * FROM location");
    while (query.executeStep())
    {
      Location location;
      location.name = query.getColumn(0).getText();
      location.frame = query.getColumn(1).getText();
      Point point{ query.getColumn(2).getDouble(), query.getColumn(3).getDouble(), query.getColumn(4).getDouble() };
      Quaternion quaternion{ query.getColumn(5).getDouble(), query.getColumn(6).getDouble(),
                             query.getColumn(7).getDouble(), query.getColumn(8).getDouble() };
      location.pose = Pose(point, quaternion);
      location.angle = query.getColumn(9).getDouble();
      location.room = Room(query.getColumn(10).getText());
      locations.push_back(location);
    }
    return locations;
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Get a location from its name
 * @param location_name Name of the location to get
 * @return Location
 */
Location LocationModel::getLocationFromName(std::string location_name)
{
  try
  {
    Location location;
    SQLite::Statement query(db, "SELECT * FROM location WHERE name = \"" + location_name + "\"");
    while (query.executeStep())
    {
      location.name = query.getColumn(0).getText();
      location.frame = query.getColumn(1).getText();
      Point point{ query.getColumn(2).getDouble(), query.getColumn(3).getDouble(), query.getColumn(4).getDouble() };
      Quaternion quaternion{ query.getColumn(5).getDouble(), query.getColumn(6).getDouble(),
                             query.getColumn(7).getDouble(), query.getColumn(8).getDouble() };
      location.pose = Pose(point, quaternion);
      location.angle = query.getColumn(9).getDouble();
      location.room = Room(query.getColumn(10).getText());
    }
    return location;
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}
}  // namespace database
}  // namespace robobreizh
