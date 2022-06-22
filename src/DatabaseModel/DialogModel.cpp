#include "DatabaseModel/DialogModel.hpp"
#include "DatabaseModel/VisionModel.hpp"
#include <string>
#include <iostream>

namespace robobreizh
{
    namespace database
    {
        DialogModel::DialogModel(): Database(){
        }
        DialogModel::~DialogModel(){
        }

        bool DialogModel::isDialogRequestFalse(){
            query = "SELECT run FROM dialog WHERE id = 1";
            pStmt = nullptr;
            int rc;
            bool isFalse;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare isDialogRequestFalse didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return false;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                if (sqlite3_column_type(pStmt,0) != SQLITE_NULL){
                    isFalse = sqlite3_column_int(pStmt, 0);
                } 
            }
            return isFalse; 
        }

        void DialogModel::setDialogRequestTrue(){
            query = "update dialog set run = 1 where id = 1";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare setDialogRequestTrue didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if ((rc = sqlite3_step(pStmt)) != SQLITE_DONE) {
                std::cout << "step didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

        }

        std::vector<Person> DialogModel::getSeatedPerson(){
            query = "SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id FROM seated_person LEFT JOIN person ON seated_person.person_id = person.id LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id order by person.id";
            pStmt = nullptr;
            int rc;
            std::vector<Person> personList;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPersonWithName didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return personList;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                Person person;
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
                personList.push_back(person);
            }
            sqlite3_finalize(pStmt);
            return personList;
        }

        void DialogModel::insertSeatedPerson(){
            int personId = getLastPersonIdWithName();
            query ="INSERT INTO seated_person (person_id) VALUES (?)";
            pStmt = nullptr;
            int rc;
              
            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare insertSeatedPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_int(pStmt,1,personId) != SQLITE_OK){
                std::cout << "bind person id didn t went through" << std::endl;
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

        int DialogModel::getLastPersonIdWithName(){
            query ="SELECT person.id FROM person WHERE name IS NOT NULL order by person.id DESC limit 1";
            pStmt = nullptr;
            int rc;
            int personId;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPersonIdWithName didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return personId;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                personId = sqlite3_column_int(pStmt,0); 
            }
            sqlite3_finalize(pStmt);
            return personId;
        }

        Person DialogModel::getLastPersonWithName(){
            query ="SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id FROM person LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id WHERE name IS NOT NULL order by person.id DESC limit 1";
            pStmt = nullptr;
            int rc;
            Person person;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPersonWithName didn t went through" << std::endl;
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

        int DialogModel::selectLastPersonId(){
            query ="SELECT id from person order by id DESC limit 1";
            pStmt = nullptr;
            int rc;
            int lastPersonId;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare selectLastPersonId didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return lastPersonId;
            }

            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) {                                              /* 2 */
                lastPersonId = sqlite3_column_int(pStmt, 0);
                /* std::cout << sqlite3_column_name(pStmt,0) << ", " << lastPersonId << std::endl; */
            }
            sqlite3_finalize(pStmt);
            return lastPersonId;
        }

        void DialogModel::updatePersonName(std::string personName){

            int lastPersonId = selectLastPersonId();

            query ="UPDATE person SET name = (?) WHERE id = (?)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare updatePersonName didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt, 1, personName.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person Name didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_int(pStmt,2,lastPersonId) != SQLITE_OK){
                std::cout << "bind person id didn t went through" << std::endl;
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

        void DialogModel::updatePersonFavoriteDrink(std::string personFavoriteDrink){

            int lastPersonId = selectLastPersonId();

            query ="UPDATE person SET favorite_drink = (?) WHERE id = (?)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare updatePersonFavoriteDrink didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt, 1, personFavoriteDrink.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person drink didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
            if (sqlite3_bind_int(pStmt,2,lastPersonId) != SQLITE_OK){
                std::cout << "bind person id didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if ((rc = sqlite3_step(pStmt)) != SQLITE_DONE) {                                              /* 2 */
                std::cout << "step didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }
        }
    };
};
