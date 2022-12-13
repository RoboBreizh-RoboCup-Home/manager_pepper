#include "database_model/color_model.hpp"
#include <string>
#include <iostream>
#include <vector>

namespace robobreizh
{
namespace database
{
ColorModel::ColorModel()
{
}

ColorModel::~ColorModel()
{
}

/**
 * @brief Create color table in the database
 */
void ColorModel::createTable()
{
  try
  {
    db.exec(R"(CREATE TABLE IF NOT EXISTS color (
          id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
          label TEXT NOT NULL UNIQUE) )");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert a color in the database
 * @param color Color to insert
 */
void ColorModel::insertColor(Color color)
{
  try
  {
    SQLite::Statement query(db, R"(
  INSERT INTO color (label) VALUES (?))");
    query.bind(1, color.label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << "can t insert color" << std::endl;
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Insert a color in the database
 * @param label Label of the color
 */
void ColorModel::insertColor(std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  INSERT INTO color (label) VALUES (?))");
    query.bind(1, label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update a color in the database
 * @param id Id of the color
 * @param color Color to update
 */
void ColorModel::updateColor(int id, Color color)
{
  try
  {
    SQLite::Statement query(db, R"(
  UPDATE color SET label = ? WHERE id = ?)");
    query.bind(1, color.label);
    query.bind(2, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Update a color in the database
 * @param id Id of the color
 * @param label Label of the color
 */
void ColorModel::updateColor(int id, std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  UPDATE color SET label = ? WHERE id = ?)");
    query.bind(1, label);
    query.bind(2, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete a color in the database
 * @param id Id of the color
 */
void ColorModel::deleteColor(int id)
{
  try
  {
    SQLite::Statement query(db, R"(
  DELETE FROM color WHERE id = ?)");
    query.bind(1, id);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete a color in the database
 * @param label Label of the color
 */
void ColorModel::deleteColor(std::string label)
{
  try
  {
    SQLite::Statement query(db, R"(
  DELETE FROM color WHERE label = ?)");
    query.bind(1, label);
    query.exec();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Delete all colors in the database
 */
void clearColor()
{
  try
  {
    db.exec("DELETE FROM color");
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/**
 * @brief Get all colors from the database
 * @return std::vector<Color> Vector of colors
 */
std::vector<Color> ColorModel::getAllColors()
{
  std::vector<Color> colors;
  try
  {
    SQLite::Statement query(db, "SELECT label FROM color");
    while (query.executeStep())
    {
      Color color;
      color.label = query.getColumn(0).getText();
      colors.push_back(color);
    }
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return colors;
}

/**
 * @brief Get a color from its id
 * @param id Id of the color
 * @return Color Color found
 */
Color ColorModel::getColorFromId(int id)
{
  Color color;
  try
  {
    SQLite::Statement query(db, "SELECT label FROM color WHERE id = ?");
    query.bind(1, id);
    query.executeStep();
    color.label = query.getColumn(0).getText();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return color;
}

/**
 * @brief Get the id of a color
 * @param label Label of the color
 * @return int Id of the color
 */
int ColorModel::getColorId(std::string label)
{
  try
  {
    SQLite::Statement query(db, "SELECT id FROM color WHERE label = ?");
    query.bind(1, label);
    query.executeStep();
    return query.getColumn(0).getInt();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return -1;
}

/**
 * @brief Get the id of a color
 * @param color Color to find
 * @return int Id of the color
 */
int ColorModel::getColorId(Color color)
{
  try
  {
    SQLite::Statement query(db, "SELECT id FROM color WHERE label = ?");
    query.bind(1, color.label);
    query.executeStep();
    return query.getColumn(0).getInt();
  }
  catch (SQLite::Exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  return -1;
}

}  // namespace database
}  // namespace robobreizh
