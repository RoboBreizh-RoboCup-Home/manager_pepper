#ifndef _PNP_ROBOBREIZH_ROOM_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_ROOM_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh {
namespace database {
class RoomModel : Database {
public:
  RoomModel();
  virtual ~RoomModel();
  void createTable();
  void insertRoom(Room room);
  void insertRoom(std::string label);
  void updateRoom(int id, Room room);
  void updateRoom(int id, std::string label);
  void deleteRoom(int id);
  void deleteRoom(std::string label);
  void clearRoom();
  std::vector<Room> getAllRooms();
  Room getRoomFromId(int id);
  int getRoomId(std::string label);
  int getRoomId(Room room);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_ROOM_DATABASE_MODEL_
