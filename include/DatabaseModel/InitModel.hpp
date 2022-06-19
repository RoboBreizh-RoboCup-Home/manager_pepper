#ifndef _PNP_ROBOBREIZH_INIT_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_INIT_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp" 
#include <string>

namespace robobreizh
{
    namespace database
    {
        class InitModel : Database{
        public:
            InitModel();
            virtual ~InitModel();
            void deleteAllPersonRows();
            void deleteAllSeatedPerson();
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_INIT_DATABASE_MODEL_
