#ifndef _PNP_ROBOBREIZH_COLOR_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_COLOR_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{

class ColorModel : Database
{
public:
  ColorModel();
  virtual ~ColorModel();
  void createTable();
  void insertColor(Color color);
  void insertColor(std::string label);
  void updateColor(int id, Color color);
  void deleteColor(int id);
  void deleteColor(std::string label);
  void clearColor();
  std::vector<Color> getAllColors();
  Color getColorFromId(int id);
  int getColorId(std::string label);
  int getColorId(Color color);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_COLOR_DATABASE_MODEL_
