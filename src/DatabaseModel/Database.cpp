#include "DatabaseModel/Database.hpp"

namespace robobreizh
{
    namespace database
    {
        class Database
        {
            Database::Database()
            {
                *zErrMsg = 0;
                Database::connect()
            }

            Database::~Database()
            {
                Database::close();
            }

            void Database::connect()
            {
                int rc;

                std::string manager_path = ros::package::getPath("manager_pepper");
                std::string db_file_path = manager_path + "/manager_db/roboBreizhDb.sql";

                rc = sqlite3_open(db_file_path, &db);
                if (rc)
                {
                    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
                    return (0);
                }
                else
                {
                    fprintf(stderr, "Opened database successfully\n");
                }
            }

            void Database::close()
            {
                sqlite3_close(db);
            }
        };
    };
};