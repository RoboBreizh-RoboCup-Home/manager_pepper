#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include "database_model/speech_model.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{

SpeechModel::SpeechModel()
{
}

SpeechModel::~SpeechModel()
{
}

void SpeechModel::createTable()
{
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS speech(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    transcript TEXT NOT NULL))");
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

void SpeechModel::clearSpeech()
{
  try
  {
    db.exec(R"(DELETE FROM speech)");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

std::string SpeechModel::getLastSpeech()
{
  try
  {
    SQLite::Statement query(db, "SELECT id FROM room ORDER BY person.id DESC LIMIT 1");
    while (query.executeStep())
    {
      return query.getColumn(0).getText();
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return "";
}

};      // namespace database
};      // namespace robobreizh
