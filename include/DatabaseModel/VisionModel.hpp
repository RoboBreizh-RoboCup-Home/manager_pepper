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
        std::string age;
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
            void createPersonFromFeatures(std::string gender, std::string age, std::string cloth_color,std::string skin_color);
            void createPersonGenderAgeClothSkin(std::string gender, std::string age, int cloth_color, int skin_color);
            void createPersonGenderAgeCloth(std::string gender, std::string age, int cloth_color);
            void createPersonGenderAgeSkin(std::string gender, std::string age, int skin_color);
            int getColorByLabel(std::string sColor);
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
