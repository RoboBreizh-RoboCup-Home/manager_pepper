#ifndef _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_

#include <string>

#include "DatabaseModel/Database.hpp"

namespace robobreizh
{
    namespace database
    {
        typedef struct GPSRAction{
        std::string intent;
        std::string object_item;
        std::string person;
        std::string destination;
        std::string who;
        std::string what;
        } GPSRAction;

        enum GPSRActionItemName {intent, object_item, person, destination, who, what};

        class GPSRActionsModel : Database {
        public:
            GPSRActionsModel();
            virtual ~GPSRActionsModel();
            bool insertAction(unsigned int id, const GPSRAction& action);
            GPSRAction getAction(unsigned int id);
            std::string getSpecificItemFromCurrentAction(GPSRActionItemName itemName);
            bool deleteAllActions();
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
