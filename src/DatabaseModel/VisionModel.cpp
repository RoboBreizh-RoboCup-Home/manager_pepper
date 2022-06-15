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
            query = "";
            query += "SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id ";
            query += "FROM person ";
            query += "LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id ";
            query += "LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id ";
            query += "order by id DESC limit 1";
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
                std::string strName((char*)sqlite3_column_text(pStmt, 0));
                person.name = strName;

                std::string strDrink((char*)sqlite3_column_text(pStmt, 1));
                person.favorite_drink = strDrink;
                std::string strGender((char*)sqlite3_column_text(pStmt, 2));
                person.gender = strGender;

                person.age = sqlite3_column_int(pStmt, 3);

                std::string strClothColor((char*)sqlite3_column_text(pStmt, 4));
                person.cloth_color = strClothColor;

                std::string strSkinColor((char*)sqlite3_column_text(pStmt, 5));
                person.skin_color = strSkinColor;
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

        void VisionModel::createPersonFromFeatures(std::string cloth_color, std::string gender, int age,std::string skin_color){

            query = "INSERT INTO person (gender, age,cloth_color_id, skin_color_id) VALUES ((?),(?),(?),(?))";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare createPersonFromFeatures didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            // get the index for given color
            int cloth_color_index = getColorByLabel(cloth_color); 
            if (sqlite3_bind_int(pStmt, 1, cloth_color_index) != SQLITE_OK){
                std::cout << "bind person cloth didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_text(pStmt, 2, gender.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person gender didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_int(pStmt,3,age) != SQLITE_OK){
                std::cout << "bind person age didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            // get the index for given color
            int skin_color_index = getColorByLabel(skin_color); 
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
