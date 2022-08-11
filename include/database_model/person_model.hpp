#ifndef _PNP_ROBOBREIZH_PERSON_DATABASE_MODEL_
#define _PNP_ROBOBREIZH_PERSON_DATABASE_MODEL_
#include "database_model/database.hpp"
#include "database_model/database_utils.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>
#include <geometry_msgs/Point.h>

/*
typedef struct Person
{
  std::string name;
  std::string favorite_drink;
  std::string gender;
  std::string age;
  Color cloth_color;
  Color skin_color;
  std::string posture;
  float height;
  Point position;
  float distance;
} Person;
*/
namespace robobreizh
{
namespace database
{
class PersonModel : Database
{
public:
  PersonModel();
  ~PersonModel();
  void createTable();
  void insertPerson(Person person);
  void insertPerson(std::string name, std::string favorite_drink, std::string gender, std::string age,
                    Color cloth_color, Color skin_color, std::string posture, float height,
                    geometry_msgs::Point position, float distance);
  std::vector<Person> getPersons();
  int getLastPersonId();
  int getFirstPersonId();
  Person getLastPerson();
  Person getPerson(int id);
  void updatePerson(int id, Person person);
  void clearPerson();
  void deletePerson(int id);
};
}  // namespace database
}  // namespace robobreizh
#endif  // _PNP_ROBOBREIZH_PERSON_DATABASE_MODEL_
