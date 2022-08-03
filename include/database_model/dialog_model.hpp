#ifndef _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/vision_model.hpp"
#include <string>
#include <vector>

namespace robobreizh
{
namespace database
{
class DialogModel : Database
{
public:
  DialogModel();
  virtual ~DialogModel();
  void insertSeatedPerson();
  int getLastPersonIdWithName();
  Person getLastPersonWithName();
  std::vector<Person> getSeatedPerson();
  int selectLastPersonId();
  void updatePersonName(std::string personName);
  void updatePersonFavoriteDrink(std::string personFavoriteDrink);
  bool isDialogRequestFalse();
  void setDialogRequestTrue();
  void setDialogRequestFalse();

protected:
  std::string query;
  sqlite3_stmt* pStmt;

private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_DIALOG_DATABASE_MODEL_
