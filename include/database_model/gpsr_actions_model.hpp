#ifndef _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_

#include <string>

#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"

namespace robobreizh
{
namespace database
{
class GPSRActionsModel : Database
{
public:
  GPSRActionsModel();
  virtual ~GPSRActionsModel();
  void insertAction(unsigned int id, const GPSRAction& action);
  GPSRAction getAction(unsigned int id);
  std::string getSpecificItemFromCurrentAction(GPSRActionItemName itemName);
  void deleteAllActions();

protected:

private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
