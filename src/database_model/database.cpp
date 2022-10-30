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

Database::~Database(){};

void Database::connect()
{
  /* std::cout << "SQLite database file '" << db.getFilename().c_str() << "' opened successfully" << std::endl; */
};

};  // namespace database
};  // namespace robobreizh
