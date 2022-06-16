#include "DatabaseModel/VisionModel.hpp"
#include <string>
#include <iostream>

namespace robobreizh
{
    namespace database
    {
        VisionModel::VisionModel(): Database(){
        }
        VisionModel::~VisionModel(){
        }

        Person VisionModel::selectLastPerson(){
            // don t want to spend too much time managing strings
            query = "SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id FROM person LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id order by person.id DESC limit 1";
            pStmt = nullptr;
            int rc;
            Person person;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return person;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) {                                              
                
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    std::string strName((char*)sqlite3_column_text(pStmt, 1));
                    person.name = strName;
                } else {
                    person.name = "";
                } 

                if (sqlite3_column_type(pStmt,1) != SQLITE_NULL){
                    std::string strDrink((char*)sqlite3_column_text(pStmt, 1));
                    person.favorite_drink = strDrink;
                } else {
                    person.favorite_drink = "";
                } 

                if (sqlite3_column_type(pStmt,2) != SQLITE_NULL){
                    std::string strGender((char*)sqlite3_column_text(pStmt, 2));
                    person.gender = strGender;
                } else {
                    person.gender= "";
                } 

                if (sqlite3_column_type(pStmt,3) != SQLITE_NULL){
                    std::string strAge((char*)sqlite3_column_text(pStmt, 3));
                    person.age = strAge;
                } else {
                    person.gender= "";
                } 

                if (sqlite3_column_type(pStmt,4) != SQLITE_NULL){
                    std::string strClothColor((char*)sqlite3_column_text(pStmt, 4));
                    person.cloth_color = strClothColor;
                } else {
                    person.cloth_color = "";
                } 

                if (sqlite3_column_type(pStmt,5) != SQLITE_NULL){
                    std::string strSkinColor((char*)sqlite3_column_text(pStmt, 5));
                    person.skin_color = strSkinColor;
                } else {
                    person.skin_color = "";
                } 
            }
            sqlite3_finalize(pStmt);
            return person;
        }

        int VisionModel::getColorByLabel(std::string sColor){
            int colorId;
            query = "SELECT id FROM color WHERE label = (?)";
            pStmt = nullptr;
            int rc;
            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare getColorByLabel didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return colorId;
            }

            if (sqlite3_bind_text(pStmt, 1, sColor.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind color label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return colorId;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) {                                              
                colorId = sqlite3_column_int(pStmt, 0);
            }
            sqlite3_finalize(pStmt);
            return colorId;
        }

        void VisionModel::createPersonFromFeatures(std::string gender, std::string age,std::string cloth_color,std::string skin_color){

            // get the index for given color
            int cloth_color_index = getColorByLabel(cloth_color); 
            // get the index for given color
            int skin_color_index = getColorByLabel(skin_color); 

            query = "INSERT INTO person (gender, age,cloth_color_id, skin_color_id) VALUES (?,?,?,?)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare createPersonFromFeatures didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt, 1, gender.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person gender didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_text(pStmt,2,age.c_str(),-1,NULL) != SQLITE_OK){
                std::cout << "bind person age didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            rc = sqlite3_bind_int(pStmt, 3, cloth_color_index);
            if ( rc != SQLITE_OK){
                std::cout << "bind person cloth didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_int(pStmt, 4, skin_color_index) != SQLITE_OK){
                std::cout << "bind person color skin didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if ((rc = sqlite3_step(pStmt)) != SQLITE_DONE) {                                              /* 2 */
                std::cout << "step didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            sqlite3_finalize(pStmt);
        }
    };
};
