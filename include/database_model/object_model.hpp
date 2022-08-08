#ifndef _PNP_ROBOBREIZH_OBJECT_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_OBJECT_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{
class ObjectModel : Database
{
public:
  ObjectModel();
  virtual ~ObjectModel();
  void createTable();
  void insertObject(Object object);
  void insertObject(std::string label, Color color, Point point, float distance, Location location);
  void clearObjects();
  std::vector<Object> getObjects();
  Object getLastObject();
  int getLastObjectId();
  void updateObject(int id, Object object);
  void deleteObject(int id);
}
}  // namespace database
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_OBJECT_DATABASE_MODEL_