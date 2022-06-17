#include "DatabaseModel/InitModel.hpp"
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


        void InitModel::deleteAllPersonRows(){

            query = "DELETE FROM person WHERE id IN (SELECT id FROM person)";
            pStmt = nullptr;
            int rc;

            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare deleteAllPersonRows didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            }

            if (sqlite3_step(pStmt) != SQLITE_OK){
                std::cout << "step delteAllPersonRows didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return ;
            } 
            return ;
        }
    };
};
