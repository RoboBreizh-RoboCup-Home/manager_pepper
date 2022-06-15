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
            int rc;

            std::string db_file_path("/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db");

            rc = sqlite3_open(db_file_path.c_str(), &db);
            if (rc)
            {
                std::cerr << "Can't open database" << sqlite3_errmsg(db) << std::endl;
                std::cerr << "SQL code error : " << sqlite3_extended_errcode(db) << std::endl;
                return ;
            }
        };

        void Database::manageSQLiteErrors(sqlite3_stmt *pStmt){
            std::cerr << "SQL error : " << sqlite3_errmsg(db) << std::endl;
            std::cerr << "SQL code error : " << sqlite3_extended_errcode(db) << std::endl;
            sqlite3_finalize(pStmt);
        }

        void Database::close()
        {
            sqlite3_close(db);
        };
    };
};
