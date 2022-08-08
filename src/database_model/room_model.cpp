#include "database_model/room_model.hpp"
#include <string>
#include <iostream>
#include <vector>

namespace robobreizh
{
namespace database
{
RoomModel::RoomModel()
{
}

RoomModel::~RoomModel()
{
}

/**
 * @brief Create room table in the database
 */
void RoomModel::createTable()
{
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS room (
          id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          label TEXT NOT NULL UNIQUE) )");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert a room in the database
 * @param room Room to insert
 */
void RoomModel::insertRoom(Room room)
{
  try
  {
    SQLite::Statement query(db, R"(
  INSERT INTO room (label) VALUES (?))");
    query.bind(1, room.label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert a room in the database
 * @param label Label of the room
 */
void RoomModel::insertRoom(std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  INSERT INTO room (label) VALUES (?))");
    query.bind(1, label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update a room in the database
 * @param id Id of the room
 * @param room Room to update
 */
void RoomModel::updateRoom(int id, Room room)
{
  try
  {
    SQLite::Statement query(db, R"(
  UPDATE room SET label = ? WHERE id = ?)");
    query.bind(1, room.label);
    query.bind(2, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update a room in the database
 * @param id Id of the room
 * @param label Label of the room
 */
void RoomModel::updateRoom(int id, std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  UPDATE room SET label = ? WHERE id = ?)");
    query.bind(1, label);
    query.bind(2, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete a room in the database
 * @param id Id of the room
 */
void RoomModel::deleteRoom(int id)
{
  try
  {
    SQLite::Statement query(db, R"(
  DELETE FROM room WHERE id = ?)");
    query.bind(1, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete a room in the database
 * @param label Label of the room
 */
void RoomModel::deleteRoom(std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  DELETE FROM room WHERE label = ?)");
    query.bind(1, label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete all rooms in the database
 */
void clearRoom()
{
  try
  {
    db.exec("DELETE FROM room");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Get all rooms from the database
 * @return std::vector<Room> Vector of rooms
 */
std::vector<Room> RoomModel::getAllRooms()
{
  std::vector<Room> rooms;
  try
  {
    SQLite::Statement query(db, "SELECT label FROM room");
    while (query.executeStep())
    {
      Room room;
      room.label = query.getColumn(0).getText();
      rooms.push_back(room);
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return rooms;
}

/**
 * @brief Get a room from its id
 * @param id Id of the room
 * @return Room Room found
 */
Room RoomModel::getRoomFromId(int id)
{
  try
  {
    SQLite::Statement query(db, "SELECT label FROM room WHERE id = ?");
    query.bind(1, id);
    query.executeStep();
    Room room;
    room.label = query.getColumn(0).getText();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return room;
}

/**
 * @brief Get the id of a room
 * @param label Label of the room
 * @return int Id of the room
 */
int RoomModel::getRoomId(std::string label)
{
  try
  {
    SQLite::Statement query(db, "SELECT id FROM room WHERE label = ?");
    query.bind(1, label);
    query.executeStep();
    return query.getColumn(0).getInt();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Get the id of a room
 * @param room Room to find
 * @return int Id of the room
 */
int RoomModel::getRoomId(Room room)
{
  try
  {
    SQLite::Statement query(db, "SELECT id FROM room WHERE label = ?");
    query.bind(1, room.label);
    query.executeStep();
    return query.getColumn(0).getInt();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

}  // namespace database
}  // namespace robobreizh