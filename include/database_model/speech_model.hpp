#ifndef _PNP_ROBOBREIZH_SPEECH_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_SPEECH_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh
{
namespace database
{
class SpeechModel : Database
{
public:
  SpeechModel();
  virtual ~SpeechModel();
  void createTable();
  // Commented methods are not necessary yet
  //   void insertSpeech(Speech speech);
  //   void insertSpeech(std::string transcript);
  //   void updateSpeech(int id, Speech speech);
  //   void updateSpeech(int id, std::string transcript);
  //   void deleteSpeech(int id);
  //   void deleteSpeech(std::string transcript);
  void clearSpeech();
  //   std::vector<Speech> getAllSpeechs();
  //   Speech getSpeechFromId(int id);
  Speech getLastSpeech();
  //   int getSpeechId(std::string transcript);
  //   int getSpeechId(Speech speech);

protected:
private:
};
};      // namespace database
};      // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_speech_DATABASE_MODEL_
