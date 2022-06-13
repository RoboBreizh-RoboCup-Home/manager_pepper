#include "DatabaseModel/Database.hpp"
#include <string>
#include <ros/package.h>

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
            int rc;

            std::string manager_path = ros::package::getPath("manager_pepper");
            std::string db_file_path = manager_path + "/manager_db/roboBreizhDb.db";

            rc = sqlite3_open(db_file_path.c_str(), &db);
            if (rc)
            {
                fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
                return ;
            }
            else
            {
                fprintf(stderr, "Opened database successfully\n");
            }
        };

        void Database::close()
        {
            sqlite3_close(db);
        };
    };
};
