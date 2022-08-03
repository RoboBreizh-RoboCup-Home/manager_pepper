#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "DatabaseModel/GPSRActionsModel.hpp"
#include "SQLiteUtils.hpp"

using namespace std;

namespace robobreizh
{
namespace database
{
// Constructors - Destructors
GPSRActionsModel::GPSRActionsModel() : Database()
{
}
GPSRActionsModel::~GPSRActionsModel()
{
}

// Methods
bool GPSRActionsModel::insertAction(unsigned int id, const GPSRAction& action)
{
  query = "INSERT INTO gpsr_action (id, intent, object_item, person, destination, who, what) VALUES (?,?,?,?,?,?,?)";
  pStmt = nullptr;
  int rc;

  rc = sqlite3_prepare_v2(db, query.c_str(), -1, &pStmt, NULL);
  if (rc != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - SQLite database not available, abort");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_int(pStmt, 1, id) != SQLITE_OK)
  {
    std::cout << "GPSRActionsModel::insertAction - Action id binding failed" << std::endl;
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 2, action.intent.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action intent binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 3, action.object_item.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action Object binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 4, action.person.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action person binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 5, action.destination.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action destination binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 6, action.who.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action who binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_bind_text(pStmt, 7, action.what.c_str(), -1, NULL) != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::insertAction - Action what binding failed");
    manageSQLiteErrors(pStmt);
    return false;
  }

  if ((rc = sqlite3_step(pStmt)) != SQLITE_DONE)
  { /* 2 */
    ROS_INFO("step didn t went through");
    manageSQLiteErrors(pStmt);
    return false;
  }
  sqlite3_finalize(pStmt);
  return true;
}

GPSRAction GPSRActionsModel::getAction(unsigned int id)
{
  query = "SELECT * FROM gpsr_action WHERE id = ?";
  pStmt = nullptr;
  int rc;
  GPSRAction gpsrAction;

  rc = sqlite3_prepare_v2(db, query.c_str(), -1, &pStmt, NULL);
  if (rc != SQLITE_OK)
  {
    ROS_INFO("GPSRActionsModel::getAction - Error");
    manageSQLiteErrors(pStmt);
    return gpsrAction;
  }

  if (sqlite3_bind_int(pStmt, 1, id) != SQLITE_OK)
  {
    std::cout << "GPSRActionsModel::getAction - Action id binding failed" << std::endl;
    manageSQLiteErrors(pStmt);
    return gpsrAction;
  }

  while ((rc = sqlite3_step(pStmt)) == SQLITE_ROW)
  {
    if (sqlite3_column_type(pStmt, 1) != SQLITE_NULL)
    {
      string strIntent((char*)sqlite3_column_text(pStmt, 1));
      gpsrAction.intent = strIntent;
    }
    else
    {
      gpsrAction.intent = "";
    }

    if (sqlite3_column_type(pStmt, 2) != SQLITE_NULL)
    {
      string strObject((char*)sqlite3_column_text(pStmt, 2));
      gpsrAction.object_item = strObject;
    }
    else
    {
      gpsrAction.object_item = "";
    }

    if (sqlite3_column_type(pStmt, 3) != SQLITE_NULL)
    {
      string strPerson((char*)sqlite3_column_text(pStmt, 3));
      gpsrAction.person = strPerson;
    }
    else
    {
      gpsrAction.person = "";
    }

    if (sqlite3_column_type(pStmt, 4) != SQLITE_NULL)
    {
      string strDestination((char*)sqlite3_column_text(pStmt, 4));
      gpsrAction.destination = strDestination;
    }
    else
    {
      gpsrAction.destination = "";
    }

    if (sqlite3_column_type(pStmt, 5) != SQLITE_NULL)
    {
      string strWho((char*)sqlite3_column_text(pStmt, 5));
      gpsrAction.who = strWho;
    }
    else
    {
      gpsrAction.who = "";
    }

    if (sqlite3_column_type(pStmt, 6) != SQLITE_NULL)
    {
      string strWhat((char*)sqlite3_column_text(pStmt, 6));
      gpsrAction.what = strWhat;
    }
    else
    {
      gpsrAction.what = "";
    }
  }
  sqlite3_finalize(pStmt);
  return gpsrAction;
}

bool GPSRActionsModel::deleteAllActions()
{
  query = "DELETE FROM gpsr_action WHERE id IN (SELECT id FROM gpsr_action)";
  pStmt = nullptr;
  int rc;

  rc = sqlite3_prepare_v2(db, query.c_str(), -1, &pStmt, NULL);
  if (rc != SQLITE_OK)
  {
    cout << "GPSRActionsModel::deleteAllActions - Aborted" << endl;
    manageSQLiteErrors(pStmt);
    return false;
  }

  if (sqlite3_step(pStmt) != SQLITE_OK)
  {
    cout << "GPSRActionsModel::deleteAllActions - Aborted" << endl;
    manageSQLiteErrors(pStmt);
    return false;
  }
  return true;
}

string GPSRActionsModel::getSpecificItemFromCurrentAction(GPSRActionItemName itemName)
{
  string specificItem = "";
  // Get current action id
  std_msgs::Int32 current_action_id_int32;
  bool is_value_available =
      SQLiteUtils::getParameterValue<std_msgs::Int32>("param_gpsr_i_action", current_action_id_int32);

  // Get gpsrActionInformation
  database::GPSRAction gpsrAction = getAction(current_action_id_int32.data);

  switch (itemName)
  {
    case GPSRActionItemName::intent:
      specificItem = gpsrAction.intent;
      break;

    case GPSRActionItemName::object_item:
      specificItem = gpsrAction.object_item;
      break;

    case GPSRActionItemName::person:
      specificItem = gpsrAction.person;
      break;

    case GPSRActionItemName::destination:
      specificItem = gpsrAction.destination;
      break;

    case GPSRActionItemName::who:
      specificItem = gpsrAction.who;
      break;

    case GPSRActionItemName::what:
      specificItem = gpsrAction.what;
      break;

    default:
      break;
  }

  return specificItem;
}
}  // namespace database
}  // namespace robobreizh