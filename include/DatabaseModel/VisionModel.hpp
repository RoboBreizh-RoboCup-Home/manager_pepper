#ifndef _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp" 
#include <string>

namespace robobreizh
{
    typedef struct Person{
        std::string name;
        std::string favorite_drink;
        std::string gender;
        int age;
        std::string cloth_color;
        std::string skin_color;
    } Person;

    namespace database
    {
        class VisionModel : Database{
        public:
            VisionModel();
            virtual ~VisionModel();
            Person selectLastPerson();
            void createPersonFromFeatures(std::string cloth_color, std::string gender, int age,std::string skin_color);
            int getColorByLabel(std::string sColor);
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
