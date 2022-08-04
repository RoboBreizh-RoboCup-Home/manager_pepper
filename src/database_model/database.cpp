#include "database_model/database.hpp"
#include <string>
#include <iostream>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{
Database::Database()
{
  connect();
};

Database::~Database()
{
  close();
};

void Database::connect()
{
 std::cout << "SQLite database file '" << db.getFilename().c_str() << "' opened successfully"<< std::endl;

};

void Database::close()
{
  remove("/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db");
};
};  // namespace database
};  // namespace robobreizh
