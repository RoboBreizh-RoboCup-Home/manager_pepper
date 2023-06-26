#ifndef _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_

#include <string>
#include <unordered_map>
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"

namespace robobreizh {
namespace database {
class GPSRActionsModel : Database {
public:
  GPSRActionsModel();
  virtual ~GPSRActionsModel();
  int insertActionVariation(const GPSRVariation& action);
  void insertAction(unsigned int id, const GPSRAction& action);
  GPSRAction getAction(unsigned int id);
  GPSRVariation getActionVariation(unsigned int id);
  std::unordered_map<std::string, std::string> getSpecificItemVariationsFromCurrentAction(GPSRActionItemName itemName);
  std::string getSpecificItemFromCurrentAction(GPSRActionItemName itemName);
  void deleteAllActions();

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
