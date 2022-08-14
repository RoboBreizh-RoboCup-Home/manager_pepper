#include "database_model/room_model.hpp"
#include "database_model/object_model.hpp"
#include "database_model/color_model.hpp"
#include "database_model/database_utils.hpp"

#include <string>
#include <geometry_msgs/Point.h>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{
/**
 * @brief constructor
 */
ObjectModel::ObjectModel()
{
}

/**
 * @brief destructor
 */
ObjectModel::~ObjectModel()
{
}

/**
 * @brief create table in database
 */
void ObjectModel::createTable()
{
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS object (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL,
	color_id INTEGER,
    x REAL,
    y REAL,
    z REAL,
    distance REAL,
    room_id INTEGER NOT NULL,
    FOREIGN KEY(room_id) REFERENCES room(id),
    FOREIGN KEY(color_id) REFERENCES color(id)
))");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief insert an object in the database
 * @param object Object
 */
void ObjectModel::insertObject(Object object)
{
  try
  {
    ColorModel cm;
    int color_id = cm.getColorId(object.color);
    RoomModel rm;
    int room_id = rm.getRoomId(object.room);
    SQLite::Statement query(db, R"(
    INSERT INTO object (label, color_id, x, y, z, distance, room_id) VALUES (?,?,?,?,?,?,?)");
    query.bind(1, object.label);
    query.bind(2, color_id);
    query.bind(3, object.position.x);
    query.bind(4, object.position.y);
    query.bind(5, object.position.z);
    query.bind(6, object.distance);
    query.bind(7, room_id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief insert an object in the database
 * @param label string Label of the object
 * @param color Color of the object
 * @param point Point that represent the position of the object
 * @param distance float distance of the object
 * @param room Room of the object
 */
void ObjectModel::insertObject(std::string label, Color color, geometry_msgs::Point point, float distance, Room room)
{
  try
  {
    ColorModel cm;
    int color_id = cm.getColorId(color);
    RoomModel rm;
    int room_id = rm.getRoomId(room);
    SQLite::Statement query(
        db, R"(INSERT INTO object (label, color_id, x, y, z, distance, room_id) VALUES (?,?,?,?,?,?,?))");
    query.bind(1, label);
    query.bind(2, color_id);
    query.bind(3, point.x);
    query.bind(4, point.y);
    query.bind(5, point.z);
    query.bind(6, distance);
    query.bind(7, room_id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief clear object table content
 */
void ObjectModel::clearObjects()
{
  try
  {
    db.exec(R"(DELETE FROM object)");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief get all objects from the database
 * @return vector of objects
 */
std::vector<Object> ObjectModel::getObjects()
{
  std::vector<Object> objects;
  try
  {
    SQLite::Statement query(
        db,
        R"(SELECT object.label, obj_color.label as color_id, object.x, object.y, object.z, object.distance, room.label as room_id 
      FROM object
      LEFT JOIN color obj_color ON object.color_id = obj_color.id
      LEFT JOIN room room ON object.room_id = room.id)");
    while (query.executeStep())
    {
      Object object;
      object.label = query.getColumn(0).getText();
      object.color = { query.getColumn(1).getText() };
      // ros structs do not provide {} initialization for struct
      geometry_msgs::Point point;
      point.x = query.getColumn(2).getDouble();
      point.y = query.getColumn(3).getDouble();
      point.z = query.getColumn(4).getDouble();
      object.position = point;
      object.distance = query.getColumn(5).getDouble();
      object.room = { query.getColumn(6).getText() };
      objects.push_back(object);
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return objects;
}

/**
 * @brief get object with a given labelfrom the database
 * @return vector of objects
 */
std::vector<Object> ObjectModel::getObjectByLabel(std::string label)
{
  std::vector<Object> objects;
  try
  {
    SQLite::Statement query(
        db,
        R"(SELECT object.label, obj_color.label as color_id, object.x, object.y, object.z, object.distance, room.label as room_id
FROM object
LEFT JOIN color obj_color ON object.color_id = obj_color.id
LEFT JOIN room room ON object.room_id = room.id
WHERE label = ?)");
    query.bind(1, label);
    while (query.executeStep())
    {
      Object object;
      object.label = query.getColumn(0).getText();
      object.color = { query.getColumn(1).getText() };
      // ros structs do not provide {} initialization for struct
      geometry_msgs::Point point;
      point.x = query.getColumn(2).getDouble();
      point.y = query.getColumn(3).getDouble();
      point.z = query.getColumn(4).getDouble();
      object.position = point;
      object.distance = query.getColumn(5).getDouble();
      object.room = { query.getColumn(6).getText() };
      objects.push_back(object);
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return objects;
}

/**
 * @brief get last from the database
 * @return object
 */
Object ObjectModel::getLastObject()
{
  Object object;
  try
  {
    SQLite::Statement query(
        db,
        R"(SELECT object.label, obj_color.label as color_id, object.x, object.y, object.z, object.distance, room.label as room_id
FROM object
LEFT JOIN color obj_color ON object.color_id = obj_color.id
LEFT JOIN room room ON object.room_id = room.id
ORDER BY object.id DESC
LIMIT 1)");
    query.executeStep();

    object.label = query.getColumn(0).getText();
    object.color = { query.getColumn(1).getText() };
    // ros structs do not provide {} initialization for struct
    geometry_msgs::Point point;
    point.x = query.getColumn(2).getDouble();
    point.y = query.getColumn(3).getDouble();
    point.z = query.getColumn(4).getDouble();
    object.position = point;
    object.distance = query.getColumn(5).getDouble();
    object.room = { query.getColumn(6).getText() };
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return object;
}

/**
 * @brief get last object id
 * @return int id of the last object
 */
int ObjectModel::getLastObjectId()
{
  try
  {
    SQLite::Statement query(db, R"(SELECT id FROM object order by id DESC limit 1)");
    query.executeStep();

    return query.getColumn(0).getInt();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return -1;
}

/**
 * @brief update an object in the database
 *
 * @param id-int id of the object to update
 * @param object-Object new values
 */
void ObjectModel::updateObject(int id, Object object)
{
  try
  {
    RoomModel rm;
    int room_id = rm.getRoomId(object.room);
    ColorModel cm;
    int color_id = cm.getColorId(object.color);
    SQLite::Statement query(db, R"(
        UPDATE object SET label = ?, color_id = ?, x = ?, y = ?, z = ?, distance = ?, sub_location_name = ? WHERE id = ?)");
    query.bind(1, object.label);
    query.bind(2, color_id);
    query.bind(3, object.position.x);
    query.bind(4, object.position.y);
    query.bind(5, object.position.z);
    query.bind(6, object.distance);
    query.bind(7, room_id);
    query.bind(8, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}
void ObjectModel::deleteObject(int id)
{
  try
  {
    SQLite::Statement query(db, R"(DELETE FROM object WHERE id = ?)");
    query.bind(1, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

}  // namespace database
}  // namespace robobreizh
