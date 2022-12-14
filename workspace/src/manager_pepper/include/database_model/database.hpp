#ifndef _PNP_ROBOBREIZH_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DATABASE_MODEL_
#include <stdio.h>
#include <string.h>
#include <SQLiteCpp/SQLiteCpp.h>

static SQLite::Database db("/root/workspace/src/manager_pepper/manager_db/roboBreizhDb.db",
                           SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
namespace robobreizh
{
namespace database
{
class Database
{
public:
  Database();
  virtual ~Database();

protected:
private:
  void connect();
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_DATABASE_MODEL_
