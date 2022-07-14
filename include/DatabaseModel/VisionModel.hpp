#ifndef _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
#include "DatabaseModel/Database.hpp" 
#include <string>
#include <vector>

namespace robobreizh
{
    typedef struct Person{
        std::string name;
        std::string favorite_drink;
        std::string gender;
        std::string age;
        std::string cloth_color;
        std::string skin_color;
        std::string posture;
        float height;
        float pos_x;
        float pos_y;
        float pos_z;
        float distance;
    } Person;

    typedef struct Object{
        std::string label;
        std::string color;
        float pos_x;
        float pos_y;
        float pos_z;
        float distance;
        bool operator < (const Object& rhs) const {
            return distance < rhs.distance;
        };
        bool operator > (const Object& rhs) const {
            return distance > rhs.distance;
        };
    } Object;

    namespace database
    {
        class VisionModel : Database{
        public:
            VisionModel();
            virtual ~VisionModel();
            Person selectLastPerson();
            void deleteAllPerson();
            void deleteAllObjects();
            void createPersonFromFeatures(std::string gender, std::string age, std::string cloth_color,std::string skin_color);
            void createPersonGenderAgeClothSkin(std::string gender, std::string age, int cloth_color, int skin_color);
            void createPersonGenderAgeCloth(std::string gender, std::string age, int cloth_color);
            void createPersonGenderAgeSkin(std::string gender, std::string age, int skin_color);
            int getColorByLabel(std::string sColor);
            void createPerson(Person person);
            void createObject(Object object); 
            std::vector<robobreizh::Object> getObjectsByLabel(std::string label); 
            std::vector<robobreizh::Object> getAllObject(); 
            std::vector<robobreizh::Person> getAllPerson();
            int getLastObjectId(); 
            int getLastPersonId(); 
            void updateFirstPerson(Person person);
            int selectLowestPersonId();
            Person selectFirstPerson();
        protected:
            std::string query;
            sqlite3_stmt *pStmt; 
        private:
        };
    };
};
#endif // _PNP_ROBOBREIZH_VISION_DATABASE_MODEL_
