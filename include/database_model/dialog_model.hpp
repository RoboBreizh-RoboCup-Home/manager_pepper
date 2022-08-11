#ifndef _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <geometry_msgs/Pose.h>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{
class DialogModel : Database
{
public:
  DialogModel();
  virtual ~DialogModel();
  void createTable();
  bool isDialogRequestFalse();
  void updateDialog(int boolean);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_