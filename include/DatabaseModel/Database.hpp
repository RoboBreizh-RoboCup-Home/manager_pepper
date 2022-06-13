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

            sqlite3 *db;
            char * zErrMsg;
        private:
            void connect();
        };
    };
};