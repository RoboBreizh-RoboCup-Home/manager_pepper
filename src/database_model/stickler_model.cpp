#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include "database_model/stickler_model.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>
#include <stdexcept>
/*
    typedef struct Stickler
    {
    bool Shoes;
    bool drink;
    bool ForbiddenRoom;
    bool Littering;
    } Stickler;
*/
namespace robobreizh
{
namespace database
{
/**
 * @brief constructor
 */
SticklerModel::SticklerModel()
{
}

/**
 * @brief destructor
 */
SticklerModel::~SticklerModel()
{
}

/**
 * @brief create table Stickler
 *
 */
void SticklerModel::createTable()
{
  try
  {
    db.exec(
        R"(CREATE TABLE IF NOT EXISTS stickler(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    shoes INTEGER,
    drink INTEGER,
    forbiddenRoom INTEGER,
  	littering INTEGER
    ))");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief insert Stickler, make sure only one value is true
 *
 * @param stickler
 */
void SticklerModel::insertStickler(Stickler stickler)
{
  try
  {
    if (stickler.Shoes + stickler.drink + stickler.ForbiddenRoom + stickler.Littering > 1)
    {
      throw std::runtime_error("More than one value is true");
    }
    SQLite::Statement query(db, R"(
        INSERT INTO stickler (shoes, drink, forbiddenRoom, littering) VALUES (?,?,?,?)
        )");
    query.bind(1, stickler.Shoes);
    query.bind(2, stickler.drink);
    query.bind(3, stickler.ForbiddenRoom);
    query.bind(4, stickler.Littering);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief update Stickler
 *
 */
void SticklerModel::updateStickler(Stickler stickler)
{
  try
  {
    if (stickler.Shoes + stickler.drink + stickler.ForbiddenRoom + stickler.Littering > 1)
    {
      throw std::runtime_error("More than one value is true");
    }

    SQLite::Statement query(db, R"(
            UPDATE stickler SET shoes = ?, drink = ?, forbiddenRoom = ?, littering = ? WHERE id = ?
            )");
    query.bind(1, stickler.Shoes);
    query.bind(2, stickler.drink);
    query.bind(3, stickler.ForbiddenRoom);
    query.bind(4, stickler.Littering);
    query.bind(5, 1);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

}  // namespace database
}  // namespace robobreizh
