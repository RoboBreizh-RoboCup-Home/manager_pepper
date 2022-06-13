#include "DatabaseModel/NavigationModel.hpp"
#include <string>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
    namespace database
    {
        NavigationModel::NavigationModel(): Database(){
        }
        NavigationModel::~NavigationModel(){
        }


        NavigationPlace NavigationModel::getLocationFromName(std::string location_name){

            query = "SELECT * FROM location WHERE name = \"" + location_name + "\"";
            /* query = "SELECT * FROM location"; */
            pStmt = nullptr;
            int rc;
            NavigationPlace np;


            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                return np;
            }

            sqlite3_bind_double(pStmt,2,6);
            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) {                                              /* 2 */
                /* printf("%s %s: Position(%f,%f,%f), Quaternion(%f,%f,%f,%f) \n", sqlite3_column_bytes(pStmt, 0),(double)sqlite3_column_bytes(pStmt, 1)) ,(float)sqlite3_column_bytes(pStmt, 2), (float)sqlite3_column_double(pStmt, 3),(float)sqlite3_column_double(pStmt, 4), (float)sqlite3_column_double(pStmt, 5),(float)sqlite3_column_double(pStmt, 6),(float)sqlite3_column_double(pStmt, 7);  /1* 3 *1/ */
                /* printf("%20s \t %d \t %20s \t %d\n", */
                /*   sqlite3_column_name(pStmt, 1), */
                /*   sqlite3_column_type(pStmt, 1), */
                /*   sqlite3_column_decltype(pStmt, 1), */
                /*   sqlite3_column_bytes(pStmt, 1)); */
                std::string strName((char*)sqlite3_column_text(pStmt, 0));
                np.name = strName;
                std::string strFrame((char*)sqlite3_column_text(pStmt, 1));
                np.frame=strFrame ;
                np.pose.position.x = sqlite3_column_double(pStmt, 2);
                np.pose.position.y = (float)sqlite3_column_double(pStmt, 3);
                np.pose.position.z = (float)sqlite3_column_double(pStmt, 4);
                np.pose.orientation.x = (float)sqlite3_column_double(pStmt, 5);
                np.pose.orientation.y = (float)sqlite3_column_double(pStmt, 6);
                np.pose.orientation.z = (float)sqlite3_column_double(pStmt, 7);
                np.pose.orientation.w = (float)sqlite3_column_double(pStmt, 8);
            }
            return np;
        }
    };
};
