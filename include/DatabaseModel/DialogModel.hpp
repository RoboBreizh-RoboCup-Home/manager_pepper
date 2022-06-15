#ifndef _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp" 
#include <string>

namespace robobreizh
{
    namespace database
    {
        class DialogModel : Database{
        public:
            DialogModel();
            virtual ~DialogModel();
            int selectLastPersonId();
            void updatePersonName(std::string personName);
            void updatePersonFavoriteDrink(std::string personFavoriteDrink);
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
