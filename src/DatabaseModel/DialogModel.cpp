#include "DatabaseModel/DialogModel.hpp"
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
