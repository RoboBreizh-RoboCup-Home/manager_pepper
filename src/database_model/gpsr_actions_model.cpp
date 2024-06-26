#include <ros/ros.h>
#include <iostream>
#include <string>
#include <unordered_map>
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
    if (action.item_context != "") {
        std::cout << "inserting action variation" << std::endl;
        std::cout << "item_context : " << action.item_context << std::endl;
        if (action.descr_verb != "") std::cout << "descr_verb : " << action.descr_verb << std::endl;
        if (action.descr_adj != "") std::cout << "descr_adj : " << action.descr_adj << std::endl;
        if (action.descr_key != "") std::cout << "descr_key : " << action.descr_key << std::endl;
        if (action.descr != "") std::cout << "descr : " << action.descr << std::endl;
        if (action.pos != "") std::cout << "pos : " << action.pos << std::endl;
        if (action.pos_obj != "") std::cout << "pos_obj : " << action.pos_obj << std::endl;
        if (action.dest_per != "") std::cout << "dest_per : " << action.dest_per << std::endl;
    }
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
  std::cout << "inserting action" << std::endl << std::endl;
  std::cout << "id :" << id << std::endl;
  std::cout << "intent : " << action.intent << std::endl;
  std::cout << "object_item : " << action.object_item << std::endl;
  std::cout << "person : " << action.person << std::endl;
  std::cout << "destination : " << action.destination << std::endl;
  std::cout << "source : " << action.source << std::endl;
  std::cout << "what : " << action.what << std::endl;

  SQLite::Statement query(db, R"(INSERT INTO gpsr_action (id, intent, destination_id, object_item_id, person_id, source_id, what_id) VALUES (?, ?, ?, ?, ?, ?, ?))");
  query.bind(1, id);
  query.bind(2, action.intent);
  query.bind(3, GPSRActionsModel::insertActionVariation(action.destination));
  query.bind(4, GPSRActionsModel::insertActionVariation(action.object_item));
  query.bind(5, GPSRActionsModel::insertActionVariation(action.person));
  query.bind(6, GPSRActionsModel::insertActionVariation(action.source));
  query.bind(7, GPSRActionsModel::insertActionVariation(action.what));
  query.exec();
}

GPSRAction GPSRActionsModel::getAction(unsigned int id) {
  GPSRAction action;
  SQLite::Statement query(db, R"""(SELECT gpsr_action.intent, 
                                    destination_variation.item_context, destination_variation.descr_verb, destination_variation.descr_adj, destination_variation.descr_key, destination_variation.descr, destination_variation.pos, destination_variation.pos_obj, destination_variation.dest_per,
                                    object_variation.item_context, object_variation.descr_verb, object_variation.descr_adj, object_variation.descr_key, object_variation.descr, object_variation.pos, object_variation.pos_obj, object_variation.dest_per, 
                                    person_variation.item_context, person_variation.descr_verb, person_variation.descr_adj, person_variation.descr_key, person_variation.descr, person_variation.pos, person_variation.pos_obj, person_variation.dest_per,
                                    source_variation.item_context, source_variation.descr_verb, source_variation.descr_adj, source_variation.descr_key, source_variation.descr, source_variation.pos, source_variation.pos_obj, source_variation.dest_per,
                                    what_variation.item_context, what_variation.descr_verb, what_variation.descr_adj, what_variation.descr_key, what_variation.descr, what_variation.pos, what_variation.pos_obj, what_variation.dest_per
                                    FROM gpsr_action
                                    LEFT JOIN gpsr_variation as destination_variation ON gpsr_action.destination_id = destination_variation.id
                                    LEFT JOIN gpsr_variation as object_variation ON gpsr_action.object_item_id = object_variation.id
                                    LEFT JOIN gpsr_variation as person_variation ON gpsr_action.person_id = person_variation.id
                                    LEFT JOIN gpsr_variation as source_variation ON gpsr_action.source_id = source_variation.id
                                    LEFT JOIN gpsr_variation as what_variation ON gpsr_action.what_id = what_variation.id
                                    WHERE gpsr_action.id = ?)""");
  query.bind(1, id);
  query.executeStep();
  action.intent = query.getColumn(0).getString();
  action.destination.item_context = query.getColumn(1).getString();
  action.destination.descr_verb = query.getColumn(2).getString();
  action.destination.descr_adj = query.getColumn(3).getString();
  action.destination.descr_key = query.getColumn(4).getString();
  action.destination.descr = query.getColumn(5).getString();
  action.destination.pos = query.getColumn(6).getString();
  action.destination.pos_obj = query.getColumn(7).getString();
  action.destination.dest_per = query.getColumn(8).getString();
  action.object_item.item_context = query.getColumn(9).getString();
  action.object_item.descr_verb = query.getColumn(10).getString();
  action.object_item.descr_adj = query.getColumn(11).getString();
  action.object_item.descr_key = query.getColumn(12).getString();
  action.object_item.descr = query.getColumn(13).getString();
  action.object_item.pos = query.getColumn(14).getString();
  action.object_item.pos_obj = query.getColumn(15).getString();
  action.object_item.dest_per = query.getColumn(16).getString();
  action.person.item_context = query.getColumn(17).getString();
  action.person.descr_verb = query.getColumn(18).getString();
  action.person.descr_adj = query.getColumn(19).getString();
  action.person.descr_key = query.getColumn(20).getString();
  action.person.descr = query.getColumn(21).getString();
  action.person.pos = query.getColumn(22).getString();
  action.person.pos_obj = query.getColumn(23).getString();
  action.person.dest_per = query.getColumn(24).getString();
  action.source.item_context = query.getColumn(25).getString();
  action.source.descr_verb = query.getColumn(26).getString();
  action.source.descr_adj = query.getColumn(27).getString();
  action.source.descr_key = query.getColumn(28).getString();
  action.source.descr = query.getColumn(29).getString();
  action.source.pos = query.getColumn(30).getString();
  action.source.pos_obj = query.getColumn(31).getString();
  action.source.dest_per = query.getColumn(32).getString();
  action.what.item_context = query.getColumn(33).getString();
  action.what.descr_verb = query.getColumn(34).getString();
  action.what.descr_adj = query.getColumn(35).getString();
  action.what.descr_key = query.getColumn(36).getString();
  action.what.descr = query.getColumn(37).getString();
  action.what.pos = query.getColumn(38).getString();
  action.what.pos_obj = query.getColumn(39).getString();
  action.what.dest_per = query.getColumn(40).getString();
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

std::unordered_map<std::string, std::string> GPSRActionsModel::getSpecificItemVariationsFromCurrentAction(GPSRActionItemName itemName) {
  std::unordered_map<std::string, std::string> specificItemVariations;
  
  // Get gpsrActionInformation
  auto gpsrAction = getAction(g_order_index);

  std::cout << "gpsrAction : " << gpsrAction << std::endl;
  std::cout << "itemName : " << itemName << std::endl;

  switch (itemName) {
    case GPSRActionItemName::intent:
      specificItemVariations["intent"] = gpsrAction.intent;
      break;
    
    case GPSRActionItemName::destination_id:
      specificItemVariations["item_context"] = gpsrAction.destination.item_context;
      specificItemVariations["descr_verb"] = gpsrAction.destination.descr_verb;
      specificItemVariations["descr_adj"] = gpsrAction.destination.descr_adj;
      specificItemVariations["descr_key"] = gpsrAction.destination.descr_key;
      specificItemVariations["descr"] = gpsrAction.destination.descr;
      specificItemVariations["pos"] = gpsrAction.destination.pos;
      specificItemVariations["pos_obj"] = gpsrAction.destination.pos_obj;
      specificItemVariations["dest_per"] = gpsrAction.destination.dest_per;
      break;

    case GPSRActionItemName::object_item_id:
      specificItemVariations["item_context"] = gpsrAction.object_item.item_context;
      specificItemVariations["descr_verb"] = gpsrAction.object_item.descr_verb;
      specificItemVariations["descr_adj"] = gpsrAction.object_item.descr_adj;
      specificItemVariations["descr_key"] = gpsrAction.object_item.descr_key;
      specificItemVariations["descr"] = gpsrAction.object_item.descr;
      specificItemVariations["pos"] = gpsrAction.object_item.pos;
      specificItemVariations["pos_obj"] = gpsrAction.object_item.pos_obj;
      specificItemVariations["dest_per"] = gpsrAction.object_item.dest_per;
      break;

    case GPSRActionItemName::person_id:
      specificItemVariations["item_context"] = gpsrAction.person.item_context;
      specificItemVariations["descr_verb"] = gpsrAction.person.descr_verb;
      specificItemVariations["descr_adj"] = gpsrAction.person.descr_adj;
      specificItemVariations["descr_key"] = gpsrAction.person.descr_key;
      specificItemVariations["descr"] = gpsrAction.person.descr;
      specificItemVariations["pos"] = gpsrAction.person.pos;
      specificItemVariations["pos_obj"] = gpsrAction.person.pos_obj;
      specificItemVariations["dest_per"] = gpsrAction.person.dest_per;
      break;

    case GPSRActionItemName::source_id:
      specificItemVariations["item_context"] = gpsrAction.source.item_context;
      specificItemVariations["descr_verb"] = gpsrAction.source.descr_verb;
      specificItemVariations["descr_adj"] = gpsrAction.source.descr_adj;
      specificItemVariations["descr_key"] = gpsrAction.source.descr_key;
      specificItemVariations["descr"] = gpsrAction.source.descr;
      specificItemVariations["pos"] = gpsrAction.source.pos;
      specificItemVariations["pos_obj"] = gpsrAction.source.pos_obj;
      specificItemVariations["dest_per"] = gpsrAction.source.dest_per;
      break;

    case GPSRActionItemName::what_id:
      specificItemVariations["item_context"] = gpsrAction.what.item_context;
      specificItemVariations["descr_verb"] = gpsrAction.what.descr_verb;
      specificItemVariations["descr_adj"] = gpsrAction.what.descr_adj;
      specificItemVariations["descr_key"] = gpsrAction.what.descr_key;
      specificItemVariations["descr"] = gpsrAction.what.descr;
      specificItemVariations["pos"] = gpsrAction.what.pos;
      specificItemVariations["pos_obj"] = gpsrAction.what.pos_obj;
      specificItemVariations["dest_per"] = gpsrAction.what.dest_per;
      break;

    default:
      break;
  }
  for (auto& item : specificItemVariations) {
    std::cout << item.first << " : " << item.second << std::endl;
  }
  return specificItemVariations;
}

std::string GPSRActionsModel::getSpecificItemFromCurrentAction(GPSRActionItemName itemName) {
  std::string specificItem = "";

  // Get gpsrActionInformation
  auto gpsrAction = getAction(g_order_index);

  std::cout << "gpsrAction : " << gpsrAction << std::endl;
  std::cout << "itemName : " << itemName << std::endl;

  switch (itemName) {
    case GPSRActionItemName::intent:
      specificItem = gpsrAction.intent;
      break;

    case GPSRActionItemName::destination_id:
      specificItem = gpsrAction.destination.item_context;
      break;

    case GPSRActionItemName::object_item_id:
      specificItem = gpsrAction.object_item.item_context;
      break;

    case GPSRActionItemName::person_id:
      specificItem = gpsrAction.person.item_context;
      break;

    case GPSRActionItemName::source_id:
      specificItem = gpsrAction.source.item_context;
      break;

    case GPSRActionItemName::what_id:
      specificItem = gpsrAction.what.item_context;
      break;

    default:
      break;
  }

  std::cout << "specificItem : " << specificItem << std::endl;

  return specificItem;
}
}  // namespace database
}  // namespace robobreizh
