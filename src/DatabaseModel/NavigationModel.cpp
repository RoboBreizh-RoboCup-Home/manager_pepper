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

            query = "SELECT * FROM location WHERE name = (?)";
            /* query = "SELECT * FROM location"; */
            pStmt = nullptr;
            int rc;
            NavigationPlace np;


            rc = sqlite3_prepare_v2(db,query.c_str(), -1, &pStmt, NULL);
            if (rc != SQLITE_OK){
                std::cout << "prepare getLocationFromName didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return np;
            }

            if (sqlite3_bind_text(pStmt, 1, location_name.c_str(), -1, NULL) != SQLITE_OK){
                std::cout << "bind location didn t went through" << std::endl;
                manageSQLiteErrors(pStmt);
                return np;
            }
            while ( (rc = sqlite3_step(pStmt)) == SQLITE_ROW) { 
                std::string strName((char*)sqlite3_column_text(pStmt, 0));
                np.name = strName;
                std::string strFrame((char*)sqlite3_column_text(pStmt, 1));
                np.frame=strFrame ;

                np.pose.position.x = sqlite3_column_double(pStmt, 2);
                np.pose.position.y = sqlite3_column_double(pStmt, 3);
                np.pose.position.z = sqlite3_column_double(pStmt, 4);

                np.pose.orientation.w = sqlite3_column_double(pStmt, 5);
                np.pose.orientation.x = sqlite3_column_double(pStmt, 6);
                np.pose.orientation.y = sqlite3_column_double(pStmt, 7);
                np.pose.orientation.z = sqlite3_column_double(pStmt, 8);
                np.angle = sqlite3_column_double(pStmt, 9);

                /* printf("%s %s: Position(%f,%f,%f), Quaternion(%f,%f,%f,%f) \n", np.name,np.frame,np.pose.position.x,np.pose.position.y,np.pose.position.z,np.pose.orientation.w,np.pose.orientation.x,np.pose.orientation.y,np.pose.orientation.z);  /1* 3 *1/ */
            }
            return np;
        }
    };
};
