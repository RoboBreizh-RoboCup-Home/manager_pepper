#include "database_model/database_utils.hpp"
#include "database_model/person_model.hpp"
#include "database_model/color_model.hpp"
#include <string>
#include <SQLiteCpp/SQLiteCpp.h>

namespace robobreizh {
namespace database {
/**
 * @brief Constructor
 */
PersonModel::PersonModel() {
}

/**
 * @brief Destructor
 */
PersonModel::~PersonModel() {
}

/**
 * @brief Create person table
 */
void PersonModel::createTable() {
  try {
    db.exec(R"(CREATE TABLE IF NOT EXISTS person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    name TEXT,
    favorite_drink TEXT,
    gender TEXT,
    age TEXT,
    clothes_style TEXT,
    cloth_color_id INTEGER,
    skin_color_id INTEGER,
    posture TEXT,
    height REAL,
    x REAL,
    y REAL,
    z REAL,
    distance REAL,
    FOREIGN KEY(cloth_color_id) REFERENCES color(id),
    FOREIGN KEY(skin_color_id) REFERENCES color(id)
))");
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert person in the database
 *
 * @param person Person
 */
void PersonModel::insertPerson(Person person) {
  try {
    ColorModel cm;
    int skin_color_id = cm.getColorId(person.skin_color.label);
    int cloth_color_id = cm.getColorId(person.cloth_color.label);
    SQLite::Statement query(
        db,
        R"(INSERT INTO person (name, favorite_drink, gender, age, clothes_style, cloth_color_id, skin_color_id, 
        posture, height, x, y, z, distance) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?))");
    query.bind(1, person.name);
    query.bind(2, person.favorite_drink);
    query.bind(3, person.gender);
    query.bind(4, person.age);
    query.bind(5, person.clothes_style);
    query.bind(6, cloth_color_id);
    query.bind(7, skin_color_id);
    query.bind(8, person.posture);
    query.bind(9, person.height);
    query.bind(10, person.position.x);
    query.bind(11, person.position.y);
    query.bind(12, person.position.z);
    query.bind(13, person.distance);
    query.exec();
  } catch (SQLite::Exception& e) {
    std::cerr << "Insert person didn't went through" << std::endl;
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert person fields in the database
 *
 * @param name-string Name of the person
 * @param favorite_drink-string Favorite drink of the person
 * @param gender-string gender of the person
 * @param age-string age of the person
 * @param clothes_style string clothes style of the person
 * @param cloth_color-Color cloth color of the person
 * @param skin_color-Color skin color of the person
 * @param posture-string posture of the person (waving, standing, sitting)
 * @param height-float height of the person
 * @param postition-Point position of the person
 * @param distance-float distance between the robot and the person
 */
void PersonModel::insertPerson(std::string name, std::string favorite_drink, std::string gender, std::string age,
                               std::string clothes_style, Color cloth_color, Color skin_color, std::string posture,
                               float height, geometry_msgs::Point position, float distance) {
  try {
    ColorModel cm;
    int skin_color_id = cm.getColorId(skin_color.label);
    int cloth_color_id = cm.getColorId(cloth_color.label);

    SQLite::Statement query(
        db,
        R"(INSERT INTO person (name, favorite_drink, gender, age, clothes_style, cloth_color_id, skin_color_id, 
        posture, height, x, y, z, distance) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?))");
    query.bind(1, name);
    query.bind(2, favorite_drink);
    query.bind(3, gender);
    query.bind(4, age);
    query.bind(5, clothes_style);
    query.bind(6, cloth_color_id);
    query.bind(7, skin_color_id);
    query.bind(8, posture);
    query.bind(9, height);
    query.bind(10, position.x);
    query.bind(11, position.y);
    query.bind(12, position.z);
    query.bind(13, distance);
    query.exec();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief get all the person from the database
 *
 * @return std::vector<Person>
 */
std::vector<Person> PersonModel::getPersons() {
  std::vector<Person> persons;
  try {
    SQLite::Statement query(db,
                            R"(SELECT person.name, person.favorite_drink, person.gender, person.age, 
        person.clothes_style, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id, 
        person.posture,person.height, person.x, person.y, person.z, person.distance, person.id
        FROM person
        LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
        LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id)");
    while (query.executeStep()) {
      Person person;
      person.name = query.getColumn(0).getText();
      person.favorite_drink = query.getColumn(1).getText();
      person.gender = query.getColumn(2).getText();
      person.age = query.getColumn(3).getText();
      person.clothes_style = { query.getColumn(4).getText() };
      person.skin_color = { query.getColumn(5).getText() };
      person.cloth_color = { query.getColumn(6).getText() };
      person.posture = query.getColumn(7).getText();
      person.height = query.getColumn(8).getDouble();

      // ros structs do not provide {} initialization for struct
      geometry_msgs::Point point;
      point.x = query.getColumn(9).getDouble();
      point.y = query.getColumn(10).getDouble();
      point.z = query.getColumn(11).getDouble();
      person.position = point;

      person.distance = query.getColumn(12).getDouble();
      persons.push_back(person);
      person.id = query.getColumn(13).getInt();
    }
  } catch (SQLite::Exception& e) {
    std::cerr << "Get persons didn't went through" << std::endl;
    std::cerr << e.what() << std::endl;
  }
  return persons;
}

/**
 * @brief get the id of the last person inserted in the database
 *
 * @return int- id of the person if return -1 then no id was found
 */
int PersonModel::getLastPersonId() {
  try {
    SQLite::Statement query(db, R"(SELECT person.id FROM person ORDER BY person.id DESC LIMIT 1)");
    while (query.executeStep()) {
      return query.getColumn(0).getInt();
    }
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return -1;
}

/**
 * @brief get the id of the first person inserted in the database
 *
 * @return int- id of the person if return -1 then no id was found
 */
int PersonModel::getFirstPersonId() {
  try {
    SQLite::Statement query(db, R"(SELECT person.id FROM person ORDER BY person.id ASC LIMIT 1)");
    while (query.executeStep()) {
      return query.getColumn(0).getInt();
    }
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return -1;
}

/**
 * @brief get the last person inserted in the database
 *
 * @return Person
 */
Person PersonModel::getLastPerson() {
  Person person;
  try {
    SQLite::Statement query(db, R"(SELECT person.name, person.favorite_drink, person.gender, person.age, 
        person.clothes_style, color_skin.label as skin_color_id, color_cloth.label as cloth_color_id,
        person.posture,person.height, person.x, person.y, person.z, person.distance, person.id
        FROM person
        LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
        LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id
        ORDER BY person.id DESC 
        LIMIT 1)");
    query.executeStep();
    person.name = query.getColumn(0).getText();
    person.favorite_drink = query.getColumn(1).getText();
    person.gender = query.getColumn(2).getText();
    person.age = query.getColumn(3).getText();
    person.clothes_style = query.getColumn(4).getText();
    person.skin_color = { query.getColumn(5).getText() };
    person.cloth_color = { query.getColumn(6).getText() };
    person.posture = query.getColumn(7).getText();
    person.height = query.getColumn(8).getDouble();
    // ros structs do not provide {} initialization for struct
    geometry_msgs::Point point;
    point.x = query.getColumn(9).getDouble();
    point.y = query.getColumn(10).getDouble();
    point.z = query.getColumn(11).getDouble();
    person.position = point;
    person.distance = query.getColumn(12).getDouble();
    person.id = query.getColumn(13).getInt();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return person;
}

/**
 * @brief get the person with the given id
 *
 * @param id-int id of the person
 * @return Person
 */
Person PersonModel::getPerson(int id) {
  Person person;
  try {
    SQLite::Statement query(db, R"(SELECT person.name, person.favorite_drink, person.gender, person.age, 
        person.clothes_style, color_skin.label as skin_color_id, color_cloth.label as cloth_color_id,
        person.posture,person.height, person.x, person.y, person.z, person.distance 
        FROM person
        LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
        LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id
        WHERE person.id = ?)");
    query.bind(1, id);
    query.executeStep();
    person.name = query.getColumn(0).getText();
    person.favorite_drink = query.getColumn(1).getText();
    person.gender = query.getColumn(2).getText();
    person.age = query.getColumn(3).getText();
    person.clothes_style = query.getColumn(4).getText();
    person.skin_color = { query.getColumn(5).getText() };
    person.cloth_color = { query.getColumn(6).getText() };
    person.posture = query.getColumn(7).getText();
    person.height = query.getColumn(8).getDouble();
    // ros structs do not provide {} initialization for struct
    geometry_msgs::Point point;
    point.x = query.getColumn(9).getDouble();
    point.y = query.getColumn(10).getDouble();
    point.z = query.getColumn(11).getDouble();
    person.position = point;
    person.distance = query.getColumn(12).getDouble();
    person.id = query.getColumn(13).getInt();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return person;
}

/**
 * @brief
 *
 * @param id
 * @param person
 */
void PersonModel::updatePerson(int id, Person person) {
  try {
    ColorModel cm;
    int skin_color_id = cm.getColorId(person.skin_color.label);
    int cloth_color_id = cm.getColorId(person.cloth_color.label);
    SQLite::Statement query(
        db,
        R"(UPDATE person SET name = ?, favorite_drink = ?, gender = ?, age = ?, clothes_style = ?, cloth_color_id = ?, 
      skin_color_id = ?, posture = ?, height = ?, x = ?, y = ?, z = ?, distance = ?
      WHERE id = ?)");
    query.bind(1, person.name);
    query.bind(2, person.favorite_drink);
    query.bind(3, person.gender);
    query.bind(4, person.age);
    query.bind(5, person.clothes_style);
    query.bind(6, skin_color_id);
    query.bind(7, cloth_color_id);
    query.bind(8, person.posture);
    query.bind(9, person.height);
    query.bind(10, person.position.x);
    query.bind(11, person.position.y);
    query.bind(12, person.position.z);
    query.bind(13, person.distance);
    query.bind(14, id);
    query.exec();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief get person by name
 */
Person PersonModel::getPersonByName(std::string name) {
  Person person;
  try {
    SQLite::Statement query(db, R"(SELECT person.name, person.favorite_drink, person.gender, person.age, 
        color_skin.label as skin_color_id, color_cloth.label as cloth_color_id,
        person.posture,person.height, person.x, person.y, person.z, person.distance 
        FROM person
        LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
        LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id
        WHERE person.name = ?)");
    query.bind(1, name);
    query.executeStep();
    person.name = query.getColumn(0).getText();
    person.favorite_drink = query.getColumn(1).getText();
    person.gender = query.getColumn(2).getText();
    person.age = query.getColumn(3).getText();
    person.clothes_style = query.getColumn(4).getText();
    person.skin_color = { query.getColumn(5).getText() };
    person.cloth_color = { query.getColumn(6).getText() };
    person.posture = query.getColumn(7).getText();
    person.height = query.getColumn(8).getDouble();
    // ros structs do not provide {} initialization for struct
    geometry_msgs::Point point;
    point.x = query.getColumn(9).getDouble();
    point.y = query.getColumn(10).getDouble();
    point.z = query.getColumn(11).getDouble();
    person.position = point;
    person.distance = query.getColumn(12).getDouble();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return person;
}

/**
 * @brief delete all persons in the database
 */
void PersonModel::clearPerson() {
  try {
    db.exec(R"(DELETE FROM person)");
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief delete the person with the given id
 *
 * @param id-int id of the person
 */
void PersonModel::deletePerson(int id) {
  try {
    SQLite::Statement query(db, R"(DELETE FROM person WHERE id = ?)");
    query.bind(1, id);
    query.exec();
  } catch (SQLite::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}  // namespace database
}  // namespace robobreizh
