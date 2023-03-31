#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "database_model/gpsr_actions_model.hpp"
#include "database_model/location_model.hpp"
#include "database_model/database_utils.hpp"
#include "manager_utils.hpp"
#include "sqlite_utils.hpp"

using namespace std;

namespace robobreizh {
namespace database {
// Constructors - Destructors
GPSRActionsModel::GPSRActionsModel() : Database() {
}
GPSRActionsModel::~GPSRActionsModel() {
}

/**
 * @brief Insert action in the database
 * @param
 */
void GPSRActionsModel::insertAction(unsigned int id, const GPSRAction& action) {
  std::cout << "inserting action" << std::endl;
  std::cout << "id :" << id << std::endl;
  std::cout << "intent : " << action.intent << std::endl;
  std::cout << "object : " << action.object_item << std::endl;
  std::cout << "person : " << action.person << std::endl;
  std::cout << "destination : " << action.destination << std::endl;
  std::cout << "source : " << action.source << std::endl;
  SQLite::Statement query(
      db, R"(INSERT INTO gpsr_action (id, intent, object_item, person, destination, source ) VALUES (?,?,?,?,?,?))");
  query.bind(1, id);
  query.bind(2, action.intent);
  query.bind(3, action.object_item);
  query.bind(4, action.person);
  query.bind(5, action.destination);
  query.bind(6, action.source);
  query.exec();
}

GPSRAction GPSRActionsModel::getAction(unsigned int id) {
  GPSRAction action;
  SQLite::Statement query(db,
                          R"(SELECT intent,object_item, person, destination, source FROM gpsr_action WHERE id = ?)");
  query.bind(1, id);
  if (query.executeStep()) {
    action.intent = query.getColumn(0).getText();
    action.object_item = query.getColumn(1).getText();
    action.person = query.getColumn(2).getText();
    action.destination = query.getColumn(3).getText();
    action.source = query.getColumn(4).getText();
  }
  return action;
}

void GPSRActionsModel::deleteAllActions() {
  db.exec("DELETE FROM gpsr_action");
}

std::string GPSRActionsModel::getSpecificItemFromCurrentAction(GPSRActionItemName itemName) {
  std::string specificItem = "";

  // Get gpsrActionInformation
  auto gpsrAction = getAction(g_order_index);

  switch (itemName) {
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
      std::cout << "target gpsr destination : " << specificItem << std::endl;
      break;

    case GPSRActionItemName::source:
      specificItem = gpsrAction.source;
      break;

    default:
      break;
  }

  return specificItem;
}
}  // namespace database
}  // namespace robobreizh
