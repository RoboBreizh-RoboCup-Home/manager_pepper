#include "DatabaseModel/DialogModel.hpp"
#include <string>

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
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return np;
            }

            sqlite3_bind_int(pStmt);
            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) {                                              /* 2 */
                lastPersonId = sqlite3_column_int(pStmt, 0);
                printf("%s,%d",sqlite3_column_name(pStmt,0),lastPersonId);
            }
            return lastPersonId;
        }

        void DialogModel::updatePersonName(std::string personName){

            int lastPersonId = selectLastPersonId();

            query ="UPDATE person SET name="+personName+"WHERE id="+lastPersonId;
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return ;
            }

            sqlite3_bind_text(pStmt);
            if ((rc = sqlite3_step(pStmt)) == SQLITE_Done) {                                              /* 2 */
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return ;
            }
        }

        void DialogModel::updatePersonFavoriteDrink(std::string personFavoriteDrink){

            int lastPersonId = selectLastPersonId();

            query ="UPDATE person SET drink="+personFavoriteDrink+"WHERE id="+lastPersonId;
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return ;
            }

            sqlite3_bind_text(pStmt);
            if ((rc = sqlite3_step(pStmt)) == SQLITE_Done) {                                              /* 2 */
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return ;
            }
        }
    };
};
