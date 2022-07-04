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
                    std::string strName((char*)sqlite3_column_text(pStmt, 0));
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
                    person.age= "";
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

        void VisionModel::createPersonGenderAgeClothSkin(std::string gender, std::string age,int cloth_color,int skin_color){
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

            rc = sqlite3_bind_int(pStmt, 3, cloth_color);
            if ( rc != SQLITE_OK){
                std::cout << "bind person cloth didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_int(pStmt, 4, skin_color) != SQLITE_OK){
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
        void VisionModel::createPersonGenderAgeCloth(std::string gender, std::string age,int cloth_color){
            query = "INSERT INTO person (gender, age,cloth_color_id) VALUES (?,?,?)";
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

            rc = sqlite3_bind_int(pStmt, 3, cloth_color);
            if ( rc != SQLITE_OK){
                std::cout << "bind person cloth didn t went through" << std::endl;
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
        void VisionModel::createPersonGenderAgeSkin(std::string gender, std::string age,int  skin_color){
            query = "INSERT INTO person (gender, age,skin_color_id) VALUES (?,?,?)";
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

            rc = sqlite3_bind_int(pStmt, 3, skin_color);
            if ( rc != SQLITE_OK){
                std::cout << "bind person skin didn t went through" << std::endl;
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

        void VisionModel::createPersonFromFeatures(std::string gender, std::string age,std::string cloth_color,std::string skin_color){

            // get the index for given color
            int cloth_color_index = -1;
            if (!cloth_color.empty()){
                cloth_color_index = getColorByLabel(cloth_color); 
            }
            // get the index for given color
            int skin_color_index = -1;
            if (!skin_color.empty()){
                skin_color_index = getColorByLabel(skin_color); 
            }
            if (cloth_color_index != -1 && skin_color_index != -1){
                createPersonGenderAgeClothSkin(gender, age, cloth_color_index, skin_color_index);
            } else if (cloth_color_index != -1 && skin_color_index == -1){
                createPersonGenderAgeCloth(gender, age, cloth_color_index);
            } else if (cloth_color_index == -1 && skin_color_index != -1){
                createPersonGenderAgeSkin(gender, age, skin_color_index);
            }
        }


        void VisionModel::createPerson(Person person){

            // get the index for given color
            int cloth_color_index = -1;
            if (!person.cloth_color.empty()){
                cloth_color_index = getColorByLabel(person.cloth_color); 
            }
            // get the index for given color
            int skin_color_index = -1;
            if (!person.skin_color.empty()){
                skin_color_index = getColorByLabel(person.skin_color); 
            }
            query = "INSERT INTO person (gender, age,cloth_color_id, skin_color_id,posture,height,position_x,position_y,position_z,distance) VALUES (?,?,?,?,?,?,?,?,?,?)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare createPersonFromFeatures didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt, 1, person.gender.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person gender didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_text(pStmt,2,person.age.c_str(),-1,NULL) != SQLITE_OK){
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
            if (sqlite3_bind_text(pStmt,5,person.posture.c_str(),-1,NULL) != SQLITE_OK){
                std::cout << "bind person posture didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 6, person.height) != SQLITE_OK){
                std::cout << "bind person height didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 7, person.pos_x) != SQLITE_OK){
                std::cout << "bind person pos x didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 8, person.pos_y) != SQLITE_OK){
                std::cout << "bind person pos y didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 9, person.pos_z) != SQLITE_OK){
                std::cout << "bind person pos z didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 10, person.distance) != SQLITE_OK){
                std::cout << "bind person pos distance didn t went through" << std::endl;
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
        int VisionModel::getLastPersonId(){
            query = "SELECT person.id FROM person order by person.id DESC limit 1";
            pStmt = nullptr;
            int rc;
            int id = -1;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return id;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    id = sqlite3_column_int(pStmt, 0);
                } 
            }

            sqlite3_finalize(pStmt);
            return id;
        } 

        void VisionModel::createObject(Object object){
            // get the index for given color
            int colorIndex = -1;
            if (!object.color.empty() && object.color != "0"){
                colorIndex = getColorByLabel(object.color); 
            }

            query = "INSERT INTO object (label,color_id,position_x,position_y,position_z,distance) VALUES (?,?,?,?,?,?)";
            pStmt = nullptr;
            int rc;
            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare createObject didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt, 1, object.label.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind object label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            std::cout << "type of the second column : " <<std::to_string(sqlite3_column_type(pStmt,2)) << std::endl;
            if (colorIndex != -1){
                rc = sqlite3_bind_int(pStmt, 2, colorIndex);
                if ( rc != SQLITE_OK){
                    std::cout << "bind object color index didn t went through" << std::endl;
                    std::cout << std::to_string(colorIndex) << std::endl;
                    manageSQLiteErrors(pStmt);
                    return ;
                }
            }

            if (sqlite3_bind_double(pStmt, 3, object.pos_x) != SQLITE_OK){
                std::cout << "bind object pos x didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 4, object.pos_y) != SQLITE_OK){
                std::cout << "bind person object y didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 5, object.pos_z) != SQLITE_OK){
                std::cout << "bind person object z didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_double(pStmt, 6, object.distance) != SQLITE_OK){
                std::cout << "bind person object distance didn t went through" << std::endl;
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

        int VisionModel::getLastObjectId(){
            query = "SELECT object.id FROM object order by object.id DESC limit 1";
            pStmt = nullptr;
            int rc;
            int id = -1;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return id;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    id = sqlite3_column_int(pStmt, 0);
                } 
            }

            sqlite3_finalize(pStmt);
            return id;
        }; 

        std::vector<robobreizh::Object> VisionModel::getObjectsByLabel(std::string label){
            query = "SELECT object.label, obj_color.label as color_id, object.position_x, object.position_y, object.position_z, object.distance FROM object LEFT JOIN color obj_color ON object.color_id = obj_color.id";
            pStmt = nullptr;
            int rc;
            int id = -1;
            std::vector<robobreizh::Object> objectList; 

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare getObjects by label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return objectList;
            }

            if (sqlite3_bind_text(pStmt, 1, label.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind object label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return objectList;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                robobreizh::Object object;
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    std::string strLabel((char*)sqlite3_column_text(pStmt, 0));
                    object.label = strLabel;
                } 
                if (sqlite3_column_type(pStmt,1) != SQLITE_NULL){
                    std::string strColor((char*)sqlite3_column_text(pStmt, 1));
                    object.color = strColor;
                } 
                if (sqlite3_column_type(pStmt,2) != SQLITE_NULL){
                    object.pos_x = sqlite3_column_double(pStmt, 2);
                } 
                if (sqlite3_column_type(pStmt,3) != SQLITE_NULL){
                    object.pos_y = sqlite3_column_double(pStmt, 3);
                } 
                if (sqlite3_column_type(pStmt,4) != SQLITE_NULL){
                    object.pos_z = sqlite3_column_double(pStmt, 4);
                } 
                if (sqlite3_column_type(pStmt,5) != SQLITE_NULL){
                   object.distance = sqlite3_column_double(pStmt, 5);
                } 
                objectList.push_back(object);
            }
            sqlite3_finalize(pStmt);
            return objectList;
        }; 

        std::vector<robobreizh::Object> VisionModel::getObjectsByLabel(std::string label){
            query = "SELECT object.label, obj_color.label as color_id, object.position_x, object.position_y, object.position_z, object.distance FROM object LEFT JOIN color obj_color ON object.color_id = obj_color.id where object.label = (?)";
            pStmt = nullptr;
            int rc;
            int id = -1;
            std::vector<robobreizh::Object> objectList; 

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare getObjects by label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return objectList;
            }

            if (sqlite3_bind_text(pStmt, 1, label.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind object label didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return objectList;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                robobreizh::Object object;
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    std::string strLabel((char*)sqlite3_column_text(pStmt, 0));
                    object.label = strLabel;
                } 
                if (sqlite3_column_type(pStmt,1) != SQLITE_NULL){
                    std::string strColor((char*)sqlite3_column_text(pStmt, 1));
                    object.color = strColor;
                } 
                if (sqlite3_column_type(pStmt,2) != SQLITE_NULL){
                    object.pos_x = sqlite3_column_double(pStmt, 2);
                } 
                if (sqlite3_column_type(pStmt,3) != SQLITE_NULL){
                    object.pos_y = sqlite3_column_double(pStmt, 3);
                } 
                if (sqlite3_column_type(pStmt,4) != SQLITE_NULL){
                    object.pos_z = sqlite3_column_double(pStmt, 4);
                } 
                if (sqlite3_column_type(pStmt,5) != SQLITE_NULL){
                   object.distance = sqlite3_column_double(pStmt, 5);
                } 
                objectList.push_back(object);
            }
            sqlite3_finalize(pStmt);
            return objectList;
        }; 

        std::vector<robobreizh::Person> VisionModel::getAllPerson(){
            query = "SELECT person.id, person.name, person.favorite_drink, person.gender, color_skin.label as skin_color_id, color_cloth.label as cloth_color_id, person.posture, person.height, person.position_x, person.position_y, person.position_z, person.distance FROM person LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id";
            pStmt = nullptr;
            int rc;
            int id = -1;
            std::vector<robobreizh::Person> personList; 

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectAllPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return personList;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                robobreizh::Person person;
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    std::string strName((char*)sqlite3_column_text(pStmt, 0));
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
                    person.age= "";
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
                if (sqlite3_column_type(pStmt,6) != SQLITE_NULL){
                    std::string strPosture((char*)sqlite3_column_text(pStmt, 6));
                    person.posture = strPosture;
                } else {
                    person.posture= "";
                } 
                if (sqlite3_column_type(pStmt,7) != SQLITE_NULL){
                    person.height= sqlite3_column_double(pStmt, 7);
                } 
                if (sqlite3_column_type(pStmt,8) != SQLITE_NULL){
                    person.pos_x = sqlite3_column_double(pStmt, 8);
                } 
                if (sqlite3_column_type(pStmt,9) != SQLITE_NULL){
                    person.pos_y = sqlite3_column_double(pStmt, 9);
                } 
                if (sqlite3_column_type(pStmt,10) != SQLITE_NULL){
                    person.pos_z = sqlite3_column_double(pStmt, 10);
                } 
                if (sqlite3_column_type(pStmt,11) != SQLITE_NULL){
                   person.distance = sqlite3_column_double(pStmt, 11);
                } 
                
                personList.push_back(person);
            }
            sqlite3_finalize(pStmt);
            return personList;
        };
    };
};
