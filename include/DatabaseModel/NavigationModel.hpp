#include "DatabaseModel/Database.hpp" 
#include <string.h>
namespace robobreizh
{
    namespace database
    {
        class NavigationModel : Database
        {
        public:
            NavigationModel();
            virtual ~NavigationModel();
            geometry_msgs::Pose NavigationModel::getLocationFromName(string location_name);
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
            void getLocationFromNameCallback();
        };
    };
};