#ifndef _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_

#include <string>

#include "DatabaseModel/Database.hpp"

namespace robobreizh
{
namespace database
{
class GPSRActionsModel : Database
{
public:
  GPSRActionsModel();
  virtual ~GPSRActionsModel();
  bool insertAction(unsigned int id, const GPSRAction& action);
  GPSRAction getAction(unsigned int id);
  std::string getSpecificItemFromCurrentAction(GPSRActionItemName itemName);
  bool deleteAllActions();

protected:
  std::string query;
  sqlite3_stmt* pStmt;

private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_GPSR_ACTIONS_DATABASE_MODEL_
