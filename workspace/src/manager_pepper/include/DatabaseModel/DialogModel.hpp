#ifndef _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp" 
#include "DatabaseModel/VisionModel.hpp" 
#include <string>
#include <vector>

namespace robobreizh
{
    namespace database
    {
        class DialogModel : Database{
        public:
            DialogModel();
            virtual ~DialogModel();
            void insertSeatedPerson();
            int getLastPersonIdWithName();
            Person getLastPersonWithName();
            std::vector<Person> getSeatedPerson();
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
