#include "DatabaseModel/NavigationModel.hpp"
#include <string.h>
#include <geometry_msgs/Pose.h>

namespace robobreizh
{
    namespace database
    {
        class NavigationModel: Database
        {
            NavigationModel::NavigationModel(): Database(){
            }
            NavigationModel::~NavigationModel(){
            }

            static int NavigationModel::sqliteNavigationFromNameCallback(void *NotUsed, int argc, char **argv, char **azColName)
            {
                int i;
                for(i=0; i<argc; i++)
                {
                    cout<<azColName[i]<<" = " << (argv[i] ? argv[i] : "NULL")<<"\n";
                }
                cout<<"\n";
                return 0;
            }(){

            }
            geometry_msgs::Pose NavigationModel::getLocationFromName(string location_name){

                query = "SELECT * 
                       FROM location 
                       WHERE name =" + location_name;
                pStmt = nullptr;
                int rp;


                rp = sqlite3_prepare_v3(db,query.c_str(),getLocationCallback, query.size() + 1, &pStmt, &zErrMsg);
                if (rp != SQLITE_OK){
                    fprintf(stderr, "SQL error: %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                    return ;
                }
                printf("The statement %s has %d parameters(s).\n", str.c_str(), sqlite3_bind_parameter_count(pStmt));
                sqlite3_bind_int(sql)

                if( rc != SQLITE_OK ) {
                    fprintf(stderr, "SQL error: %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                } else {
                    fprintf(stdout, "Operation done successfully\n");
                    return ;
                }
            }
        }
    };
};