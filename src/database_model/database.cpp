#include "DatabaseModel/Database.hpp"
#include <string>
#include <ros/package.h>
#include <iostream>

namespace robobreizh
{
namespace database
{
Database::Database()
{
  Database::connect();
};

Database::~Database()
{
  Database::close();
};

void Database::connect()
{
  db("/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db");
  std::cout << "SQLite database file '" << db.getFilename().c_str() << "' opened successfully\n";
};

void Database::close()
{
  remove("test.db3");
};
};  // namespace database
};  // namespace robobreizh
