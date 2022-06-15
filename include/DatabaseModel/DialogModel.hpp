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
