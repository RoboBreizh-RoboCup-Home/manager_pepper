#ifndef _PNP_ROBOBREIZH_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DATABASE_MODEL_
#include <stdio.h>
#include <string.h>
#include <SQLiteCpp/SQLiteCpp.h>

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
  SQLite::Database db;

private:
  void connect();
  void close();
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_DATABASE_MODEL_
