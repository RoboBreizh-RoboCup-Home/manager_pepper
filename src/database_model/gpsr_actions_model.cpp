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

int GPSRActionsModel::insertActionVariation(const GPSRVariation& action) {
    std::cout << "inserting action variation" << std::endl;
    std::cout << "item_context : " << action.item_context << std::endl;
    std::cout << "descr_verb : " << action.descr_verb << std::endl;
    std::cout << "descr_adj : " << action.descr_adj << std::endl;
    std::cout << "descr_key : " << action.descr_key << std::endl;
    std::cout << "descr : " << action.descr << std::endl;
    std::cout << "pos : " << action.pos << std::endl;
    std::cout << "pos_obj : " << action.pos_obj << std::endl;
    std::cout << "dest_per : " << action.dest_per << std::endl;
    SQLite::Statement query(db, R"(INSERT INTO gpsr_variation (item_context, descr_verb, descr_adj, descr_key, descr, pos, pos_obj, dest_per) VALUES (?, ?, ?, ?, ?, ?, ?, ?))");
    query.bind(1, action.item_context);
    query.bind(2, action.descr_verb);
    query.bind(3, action.descr_adj);
    query.bind(4, action.descr_key);
    query.bind(5, action.descr);
    query.bind(6, action.pos);
    query.bind(7, action.pos_obj);
    query.bind(8, action.dest_per);
    query.exec();
    return db.getLastInsertRowid();
}

/**
 * @brief Insert action in the database
 * @param
 */
void GPSRActionsModel::insertAction(unsigned int id, const GPSRAction& action) {
  std::cout << "inserting action" << std::endl;
  std::cout << "id :" << id << std::endl;
  std::cout << "intent : " << action.intent << std::endl;
  std::cout << "object_item : " << action.object_item << std::endl;
  std::cout << "person : " << action.person << std::endl;
  std::cout << "destination : " << action.destination << std::endl;
  std::cout << "source : " << action.source << std::endl;
  
  SQLite::Statement query(db, R"(INSERT INTO gpsr_action (id, intent, object_item_id, person_id, destination_id, source_id) VALUES (?, ?, ?, ?, ?, ?))");
  query.bind(1, id);
  query.bind(2, action.intent);
  query.bind(3, GPSRActionsModel::insertActionVariation(action.object_item));
  query.bind(4, GPSRActionsModel::insertActionVariation(action.person));
  query.bind(5, GPSRActionsModel::insertActionVariation(action.destination));
  query.bind(6, GPSRActionsModel::insertActionVariation(action.source));
  query.exec();
}

GPSRAction GPSRActionsModel::getAction(unsigned int id) {
  GPSRAction action;
  SQLite::Statement query(db, R"""(SELECT gpsr_action.intent, 
                                    object_variation.item_context, object_variation.descr_verb, object_variation.descr_adj, object_variation.descr_key, object_variation.descr, object_variation.pos, object_variation.pos_obj, object_variation.dest_per, 
                                    person_variation.item_context, person_variation.descr_verb, person_variation.descr_adj, person_variation.descr_key, person_variation.descr, person_variation.pos, person_variation.pos_obj, person_variation.dest_per,
                                    destination_variation.item_context, destination_variation.descr_verb, destination_variation.descr_adj, destination_variation.descr_key, destination_variation.descr, destination_variation.pos, destination_variation.pos_obj, destination_variation.dest_per,
                                    source_variation.item_context, source_variation.descr_verb, source_variation.descr_adj, source_variation.descr_key, source_variation.descr, source_variation.pos, source_variation.pos_obj, source_variation.dest_per
                                    FROM gpsr_action
                                    LEFT JOIN gpsr_variation as object_variation ON gpsr_action.object_item_id = gpsr_variation.id
                                    LEFT JOIN gpsr_variation as person_variation ON gpsr_action.person_id = gpsr_variation.id
                                    LEFT JOIN gpsr_variation as destination_variation ON gpsr_action.destination_id = gpsr_variation.id
                                    LEFT JOIN gpsr_variation as source_variation ON gpsr_action.source_id = gpsr_variation.id
                                    WHERE gpsr_action.id = ?)""");
  query.bind(1, id);
  query.executeStep();
  action.intent = query.getColumn(0).getString();
  action.object_item.item_context = query.getColumn(1).getString();
  action.object_item.descr_verb = query.getColumn(2).getString();
  action.object_item.descr_adj = query.getColumn(3).getString();
  action.object_item.descr_key = query.getColumn(4).getString();
  action.object_item.descr = query.getColumn(5).getString();
  action.object_item.pos = query.getColumn(6).getString();
  action.object_item.pos_obj = query.getColumn(7).getString();
  action.object_item.dest_per = query.getColumn(8).getString();
  action.person.item_context = query.getColumn(9).getString();
  action.person.descr_verb = query.getColumn(10).getString();
  action.person.descr_adj = query.getColumn(11).getString();
  action.person.descr_key = query.getColumn(12).getString();
  action.person.descr = query.getColumn(13).getString();
  action.person.pos = query.getColumn(14).getString();
  action.person.pos_obj = query.getColumn(15).getString();
  action.person.dest_per = query.getColumn(16).getString();
  action.destination.item_context = query.getColumn(17).getString();
  action.destination.descr_verb = query.getColumn(18).getString();
  action.destination.descr_adj = query.getColumn(19).getString();
  action.destination.descr_key = query.getColumn(20).getString();
  action.destination.descr = query.getColumn(21).getString();
  action.destination.pos = query.getColumn(22).getString();
  action.destination.pos_obj = query.getColumn(23).getString();
  action.destination.dest_per = query.getColumn(24).getString();
  action.source.item_context = query.getColumn(25).getString();
  action.source.descr_verb = query.getColumn(26).getString();
  action.source.descr_adj = query.getColumn(27).getString();
  action.source.descr_key = query.getColumn(28).getString();
  action.source.descr = query.getColumn(29).getString();
  action.source.pos = query.getColumn(30).getString();
  action.source.pos_obj = query.getColumn(31).getString();
  action.source.dest_per = query.getColumn(32).getString();
  return action;
}

GPSRVariation GPSRActionsModel::getActionVariation(unsigned int id) {
  GPSRVariation variation;
  SQLite::Statement query(db, R"(SELECT item_context, descr_verb, descr_adj, descr_key, descr, pos, pos_obj, dest_per FROM gpsr_variation WHERE id = ?)");
  query.bind(1, id);
  query.executeStep();
  variation.item_context = query.getColumn(0).getString();
  variation.descr_verb = query.getColumn(1).getString();
  variation.descr_adj = query.getColumn(2).getString();
  variation.descr_key = query.getColumn(3).getString();
  variation.descr = query.getColumn(4).getString();
  variation.pos = query.getColumn(5).getString();
  variation.pos_obj = query.getColumn(6).getString();
  variation.dest_per = query.getColumn(7).getString();
  return variation;
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

    case GPSRActionItemName::object_item_id:
      specificItem = gpsrAction.object_item.item_context;

    case GPSRActionItemName::person_id:
      specificItem = gpsrAction.person.item_context;

    case GPSRActionItemName::source_id:
      specificItem = gpsrAction.source.item_context;

    case GPSRActionItemName::destination_id:
      specificItem = gpsrAction.destination.item_context;

    default:
      break;
  }

  return specificItem;
}
}  // namespace database
}  // namespace robobreizh
