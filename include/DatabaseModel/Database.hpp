#ifndef _PNP_ROBOBREIZH_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DATABASE_MODEL_
#include <stdio.h>
#include <sqlite3.h>
#include <string.h>

namespace robobreizh
{
    namespace database
    {
        class Database
        {
        public:
            Database();
            virtual ~Database();
            void close();

            void manageSQLiteErrors(sqlite3_stmt *pStmt);

            sqlite3 *db;
            const char * zErrMsg = 0;
        private:
            void connect();
        };
    };
};
#endif // _PNP_ROBOBREIZH_DATABASE_MODEL_
