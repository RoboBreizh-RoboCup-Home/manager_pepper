#include "database_model/dialog_model.hpp"
#include <string>
#include <iostream>
#include <vector>

/**
 * This class is used to trigger the speech to text.
 * If the first raw is true, then the speech to text is triggered.
 * If the first raw is false, then the speech to text is not disabled.
 *
 */
namespace robobreizh
{
namespace database
{
/**
 * @brief constructor
 *
 */
DialogModel::DialogModel()
{
}

/**
 * @brief Destroy the Dialog Model:: Dialog Model object
 *
 */
DialogModel::~DialogModel()
{
}

/**
 * @brief Create dialog table in the database
 */
void DialogModel::createTable()
{
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS dialog (
          id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          run INTEGER NOT NULL UNIQUE) )");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Set the first and only row in dialog table to false
 */
void DialogModel::initDialog()
{
  try
  {
    db.exec(R"(INSERT INTO dialog (run) VALUES (0))");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update the state for the speech recognition
 * @param boolean-int valute representing the state for speech recognition
 */
void DialogModel::updateDialog(int boolean)
{
  try
  {
    SQLite::Statement query(db, R"(
  UPDATE dialog SET run = ? WHERE id = 1)");
    query.bind(1, boolean);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Check if the speech to text is disabled
 * @return true if the speech to text is disabled
 * @return false if the speech to text is not disabled
 */
bool DialogModel::isListening()
{
  try
  {
    SQLite::Statement query(db, R"(
  SELECT run FROM dialog WHERE id = 1)");
    query.executeStep();
    int run = query.getColumn(0);
    if (run == 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return false;
}

}  // namespace database
}  // namespace robobreizh
