#include "DatabaseModel/InitModel.hpp"
#include "DatabaseModel/DialogModel.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
    namespace database
    {
        InitModel::InitModel(): Database(){
        }
        InitModel::~InitModel(){
        }


        void InitModel::deleteAllPerson(){

            query = "DELETE FROM person WHERE id IN (SELECT id FROM person)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare deleteAllPersonRows didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_step(pStmt) != SQLITE_DONE){
                std::cout << "step delteAllPersonRows didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            } 
            return ;
        }

        void InitModel::deleteAllObject(){

            query = "DELETE FROM object WHERE id IN (SELECT id FROM object)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare deleteAllObject didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_step(pStmt) != SQLITE_DONE){
                std::cout << "step delteAllObject didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            } 
            return ;
        }

        void InitModel::deleteAllSeatedPerson(){
            query = "DELETE FROM seated_person WHERE id IN (SELECT id FROM seated_person)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare deleteAllSeatedPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_step(pStmt) != SQLITE_DONE){
                std::cout << "step delteAllSeatedPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            } 
            return ;
        }

        void InitModel::addPerson(std::string name, std::string drink){
            query ="INSERT INTO person (name,favorite_drink) VALUES (?,?)";
            pStmt = nullptr;
            int rc;
            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare insertPerson didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt,1,name.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person name didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_bind_text(pStmt,2,drink.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind person drink didn t went through" << std::endl;
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

        void InitModel::addReceptionistHost(std::string name, std::string drink){
            addPerson(name, drink);
            DialogModel dm;
            dm.insertSeatedPerson();
        }
    };
};
