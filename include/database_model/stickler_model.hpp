#ifndef _PNP_ROBOBREIZH_STICKLER_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_STICKLER_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh {
namespace database {
class SticklerModel : Database {
public:
  SticklerModel();
  ~SticklerModel();
  void createTable();
  void insertStickler(Stickler stickler);
  void updateStickler(Stickler stickler);

private:
};

}  // namespace database
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_STICKLER_DATABASE_MODEL_