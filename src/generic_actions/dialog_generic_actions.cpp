#include <ros/ros.h>
#include <math.h>
#include <string>
#include <thread>
#include <queue>
#include <std_msgs/String.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <chrono>
/* #include <boost/thread/thread.hpp> */
#include <boost/filesystem.hpp>
#include <dialog_pepper/Msg.h>
#include <dialog_pepper/TranscriptIntent.h>
#include <dialog_pepper/TranscriptContains.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include "generic_actions/dialog_generic_actions.hpp"
#include "database_model/location_model.hpp"
#include "database_model/person_model.hpp"
#include "database_model/dialog_model.hpp"
#include "database_model/database_utils.hpp"
#include "manager_utils.hpp"

using namespace std;

string intent;

namespace robobreizh
{
namespace dialog
{
namespace generic
{
bool robotSpeech(string text)
{
  ROS_INFO("Text to pronounce: %s", text.c_str());
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dialog_pepper::Msg>("/robobreizh/dialog_pepper/text_to_speech");
  dialog_pepper::Msg srv;
  srv.request.sentence = text;

  if (client.call(srv))
  {
    RoboBreizhManagerUtils::pubVizBoxRobotText(text);
    ROS_INFO("TTS success: %d", srv.response.success);
    return true;
  }
  else
  {
    ROS_INFO("Failed to call service pepper_speech");
    return false;
  }
}

std::vector<string> wavToIntent(std::string* sentence)
{
  std::vector<string> intent;
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dialog_pepper::Wti>("/robobreizh/dialog_pepper/wav_to_intent");
  dialog_pepper::Wti srv;
  srv.request.start = true;
  if (client.call(srv))
  {
    for (int i = 0; i < srv.response.intent.size(); i++)
    {
      ROS_INFO("Intent received: %s", srv.response.intent[i].c_str());
      intent.push_back(srv.response.intent[i].c_str());
    }
    *sentence = srv.response.parsed_sentence;
  }
  else
  {
    ROS_INFO("Failed to call service wav_to_intent");
  }
  return intent;
}

bool ListenSpeech()
{
  robobreizh::database::DialogModel dm;
  dm.updateDialog(1);

  bool b_isListening = true;
  double timeout = 10.0;
  auto start_timer = std::chrono::system_clock::now();
  do
  {
    b_isListening = dm.isListening();
    ros::Duration(0.5).sleep();
    // if more than 10 seconds passed then abort the function
    std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start_timer;
    std::cout << "elapsed time : " << elapsed.count() << std::endl;
    if (elapsed.count() > timeout)
    {
      // set boolean to false
      dm.updateDialog(0);
      ROS_INFO("Speech recognition timed out");
      return false;
    }
  } while (b_isListening);
  return true;
}

std::vector<std::string> getIntent(std::string transcript)
{

  std::vector<std::string> intent;
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<dialog_pepper::TranscriptIntent>("/robobreizh/dialog_pepper/transcript_intent");
  dialog_pepper::TranscriptIntent srv;
  srv.request.transcript = transcript;
  if (client.call(srv))
  {
    intent = srv.response.intent;
    ROS_INFO("Transcript : %s -> intent: %s", transcript.c_str(), str(intent.begin(), intent.end()));
  }
  else
  {
    ROS_INFO("Failed to call service wav_to_intent");
  }
  return intent;
}

std::string wavToParsedParam(std::string param, std::string* sentence)
{
  std::string param_res;
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<dialog_pepper::WavString>("/robobreizh/dialog_pepper/parser_from_file_srv");
  dialog_pepper::WavString srv;
  srv.request.file_name = param;
  if (client.call(srv))
  {
    param_res = srv.response.result;
    *sentence = srv.response.parsed_sentence;
    ROS_INFO("Typed parsed: %s, res: %s", param.c_str(), param_res.c_str());
  }
  else
  {
    ROS_INFO("Failed to call service wav_to_intent");
  }
  return param_res;
}


bool presentPerson(robobreizh::database::Person person)
{
  std::string sentence = "";
  std::string pronoun = "He";
  std::string possessive = "his";

  if (person.gender.compare("F") == 0)
  {
    pronoun = "She";
    possessive = "Her";
  }

  if (!person.name.empty())
  {
    sentence += pronoun + " is " + person.name + ". ";
  }

  if (person.gender.compare("F") == 0)
  {
    sentence += pronoun + " is a female.";
  }
  else
  {
    sentence += pronoun + " is a male.";
  }

  if (!person.favorite_drink.empty())
  {
    sentence += pronoun + " likes drinking " + person.favorite_drink + ". ";
  }
  if (!person.age.empty())
  {
    sentence += pronoun + " is between " + person.age + " years old. ";
  }
  dialog::generic::robotSpeech(sentence);
  sentence = "";
  if (!person.cloth_color.label.empty())
  {
    sentence += pronoun + " wears " + person.cloth_color.label + " cloth. ";
  }
  if (!person.skin_color.label.empty())
  {
    sentence += possessive + " skin is " + person.skin_color.label + ". ";
  }
  if (person.posture != "sit down")
  {
    int size = (int)trunc(person.height * 100);
    if (size > 145)
    {
      sentence += pronoun + " is " + std::to_string(size) + " centimeters tall.";
    }
  }
  if (!person.posture.empty())
  {
    // waving
    // standing
    if (person.posture == "standing")
    {
      sentence += pronoun + " is standing ";
      sentence += ".";
    }
    else if (person.posture == "sit down")
    {
      sentence += pronoun + " is sitting down.";
    }
  }
  std::cout << sentence << std::endl;
  return dialog::generic::robotSpeech(sentence);
}

bool presentPerson(std::vector<robobreizh::database::Person> listPerson)
{
  bool serviceWentThrough = true;

  for (auto& person : listPerson)
  {
    serviceWentThrough = serviceWentThrough && presentPerson(person);
  }
  return serviceWentThrough;
}

string cleanString(string& str)
{
  std::replace(str.begin(), str.end(), '\'', ' ');
  std::replace(str.begin(), str.end(), '{', ' ');
  std::replace(str.begin(), str.end(), '}', ' ');
  return str;
}

float getDistance(float x1, float y1, float z1, float x2, float y2, float z2)
{
  float distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
  return distance;
}

bool getClosestPerson(int nbPerson, std::vector<robobreizh::database::Person> listPerson,
                      robobreizh::database::Person* closestPerson, int comparedPersonIndex)
{
  if (nbPerson > 1)
  {
    float shortestDistance = 10.0;
    for (int j = 0; j < nbPerson; j++)
    {
      // avoid comparing the same person
      if (comparedPersonIndex != j)
      {
        float distance =
            getDistance(listPerson[comparedPersonIndex].position.x, listPerson[comparedPersonIndex].position.y,
                        listPerson[comparedPersonIndex].position.z, listPerson[j].position.x, listPerson[j].position.y,
                        listPerson[j].position.z);
        if (distance < shortestDistance)
        {
          shortestDistance = distance;
          std::cout << "The shortest distance between 2 person : " << std::to_string(shortestDistance) << std::endl;
          std::cout << "gender of the closest person" << listPerson[j].gender << std::endl;
          *closestPerson = listPerson[j];
        }
      }
    }
    return true;
  }
  return false;
}

int getAngleABC(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c)
{
  geometry_msgs::Point ab;
  ab.x = (b.x - a.x);
  ab.y = (b.y - a.y);
  geometry_msgs::Point cb;
  cb.x = (b.x - c.x);
  cb.y = (b.y - c.y);

  float dot = (ab.x * cb.x + ab.y * cb.y);    // dot product
  float cross = (ab.x * cb.y - ab.y * cb.x);  // cross product

  float alpha = atan2(cross, dot);

  return (int)lrint(alpha * (float)(180.0 / M_PI));
}

/*
 * a and c are the points to be compared
 * b is the reference point of the angle
 */
bool isRight(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c)
{
  int theta = getAngleABC(a, b, c);
  // is right is positive because the reference point of the angle is the robot
  // hence right and left side are opposite
  if (theta > 0)
  {
    return true;
  }
  return false;
}

// descent european if forest for describing someone
void describeClosestPersonComparedToPerson(robobreizh::database::Person closestPerson,
                                           robobreizh::database::Person currentPerson)
{
  robobreizh::database::LocationModel lm;
  robobreizh::database::Location np = lm.getLocationFromName("living room");
  geometry_msgs::Point personPose1 = currentPerson.position;
  geometry_msgs::Point personPose2 = closestPerson.position;

  std::string sentence = "";
  std::string demonstrative = "";
  std::string possessive = "";
  if (currentPerson.gender.compare("H"))
  {
    demonstrative = "Him";
    possessive = "His";
  }
  else
  {
    demonstrative = "Her";
    possessive = "Her";
  }

  std::string position = "";

  position = (isRight(personPose1, np.pose.position, personPose2)) ? "right" : "left";

  sentence += "The closest person to " + demonstrative;

  if (!closestPerson.posture.empty())
  {
    sentence += " is " + closestPerson.posture + " up to " + possessive + " " + position;
  }

  std::string pronoun = "";
  if (closestPerson.gender.compare("H"))
  {
    pronoun = "He";
    possessive = "His";
    sentence += pronoun + " is a guy.";
  }
  else
  {
    pronoun = "She";
    possessive = "Her";
    sentence += pronoun + " is a girl.";
  }

  if (!closestPerson.age.empty())
  {
    // sentence += pronoun + " is between " + closestPerson.age + " years old. ";
    sentence += " between " + closestPerson.age + " years old. ";
  }
  dialog::generic::robotSpeech(sentence);
  sentence = "";
  if (!closestPerson.cloth_color.label.empty())
  {
    sentence += pronoun + " is dressed with " + closestPerson.cloth_color.label + " clothes. ";
  }
  if (!closestPerson.skin_color.label.empty())
  {
    sentence += possessive + " skin is " + closestPerson.skin_color.label + ". ";
  }
  if (closestPerson.posture == "standing")
  {
    int size = (int)trunc(closestPerson.height * 100);
    if (size > 145)
    {
      sentence += pronoun + " is " + std::to_string(size) + " centimeters tall.";
    }
  }
  std::cout << sentence << std::endl;
  dialog::generic::robotSpeech(sentence);
}

bool isVowel(char c)
{
  // evaluates to 1 (true) if c is a lowercase vowel
  bool isLowercaseVowel = (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u');
  // evaluates to 1 (true) if c is an uppercase vowel
  bool isUppercaseVowel = (c == 'A' || c == 'E' || c == 'I' || c == 'O' || c == 'U');
  if (isLowercaseVowel || isUppercaseVowel)
  {
    return true;
  }
  return false;
}

void describeObjectComparedToPerson(pair<float, robobreizh::database::Object> pairObject,
                                    robobreizh::database::Person person)
{
  float distance = pairObject.first;
  robobreizh::database::Object object = pairObject.second;
  robobreizh::database::LocationModel lm;
  robobreizh::database::Location np = lm.getLocationFromName("living room");
  geometry_msgs::Point objectPoint = object.position;
  geometry_msgs::Point personPoint = person.position;

  std::string sentence = "";
  std::string position = "";
  std::string positionDescription = "";
  if (distance > 0.5)
  {
    position = (isRight(personPoint, np.pose.position, objectPoint)) ? "right" : "left";
    positionDescription = " on the " + position + " of our guest.";
  }
  else
  {
    positionDescription = " on our guest.";
  }

  if (isVowel(object.color.label[0]))
  {
    sentence += "I found an ";
  }
  else
  {
    sentence += "I found a ";
  }

  sentence += object.color.label + " " + object.label + positionDescription;

  std::cout << sentence << std::endl;
  dialog::generic::robotSpeech(sentence);
}

bool presentFMMGuests(std::vector<robobreizh::database::Person> listPerson,
                      std::vector<robobreizh::database::Object> listObject)
{
  // get nb object
  int nbObject = listObject.size();
  // get nb person
  int nbPerson = listPerson.size();
  // say you found n person

  robotSpeech("I ll describe you the person I found from right to left");
  // for each person
  for (auto i = 0; i < nbPerson && i < 3; i++)
  {
    switch (i)
    {
      case 0:
        robotSpeech("Here is the first person I found. ");
        break;
      case 1:
        robotSpeech("Then comes the second person. ");
        break;
      case 2:
        robotSpeech("Last, ");
        break;
      default:
        ROS_INFO(" wow it looks like we found more than 3 person here. That is not supposed to happen.");
        robotSpeech(" wow it looks like we found more than 3 person here. That is not supposed to happen.");
        break;
    }
    // present the person
    presentPerson(listPerson[i]);

    // get the closest person
    // TO DO : implement it in a thread with a shared mutex for speaking
    robobreizh::database::Person closestPerson;
    if (getClosestPerson(nbPerson, listPerson, &closestPerson, i))
    {
      // present the closest person feature
      describeClosestPersonComparedToPerson(closestPerson, listPerson[i]);
    }

    // get the 3 closest objects
    // create a priority queue with distances between person and objects
    std::priority_queue<pair<float, robobreizh::database::Object>> closestObject;
    for (auto object : listObject)
    {
      float distance = getDistance(object.position.x, object.position.y, object.position.z, listPerson[i].position.x,
                                   listPerson[i].position.y, listPerson[i].position.z);
      closestObject.push(make_pair(distance, object));
    }

    // present the closest 3 objects
    // take the top 3 of the queue
    for (int j = 0; j < 3 && !closestObject.empty(); j++, closestObject.pop())
    {
      pair<float, robobreizh::database::Object> pairClosestObject = closestObject.top();
      describeObjectComparedToPerson(pairClosestObject, listPerson[i]);
    }
  }
  return true;
}

database::GPSRAction getActionFromString(string& str)
{
  database::GPSRAction gpsrAction;
  std::vector<std::string> out;

  if (str.empty())
  {
    gpsrAction.intent = "DEBUG_EMPTY";
    return gpsrAction;
  }

  string cleanStr = cleanString(str);  // remove unnecessary characters

  boost::split(out, cleanStr, boost::is_any_of(","));  // split string by ','

  for (auto& s : out)  // construct gpsrAction by extracting features and their values
  {
    std::vector<std::string> tokens;
    boost::split(tokens, s, boost::is_any_of(":"));
    boost::algorithm::trim(tokens[0]);  // removing white spaces
    boost::algorithm::trim(tokens[1]);

    if (tokens[0] == "intent")
      gpsrAction.intent = tokens[1];

    if (tokens[0] == "destination")
      gpsrAction.destination = tokens[1];

    if (tokens[0] == "person")
      gpsrAction.person = tokens[1];

    if (tokens[0] == "object")
      gpsrAction.object_item = tokens[1];

    if (tokens[0] == "who")
      gpsrAction.who = tokens[1];

    if (tokens[0] == "what")
      gpsrAction.what = tokens[1];
  }
  return gpsrAction;
}

bool validateTranscriptActions(vector<string>& transcript)
{
  bool flag = true;
  if (!transcript.empty())
  {
    // Add Verify if each Intent has all necessary parameters
    for (int i = 0; i < transcript.size(); i++)
    {
      bool flag = true;
      database::GPSRAction gpsrAction = generic::getActionFromString(transcript.at(i));
      if (gpsrAction.intent != "DEBUG_EMPTY")
      {
        if (gpsrAction.intent == "take")
        {
          if (gpsrAction.object_item.empty() && gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "go")
        {
          if (gpsrAction.destination.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "follow")
        {
          if (gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "to find something")
        {
          if (gpsrAction.object_item.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "to find someone")
        {
          if (gpsrAction.person.empty())
            flag = false;
        }

        else if (gpsrAction.intent == "say")
        {
          if (gpsrAction.what.empty())
            flag = false;
        }

        //  Checking if the object name is valid

        if (!gpsrAction.object_item.empty())
          flag = isValidObject(gpsrAction.object_item);

        //  Checking if the Place  name is valid
        if (!gpsrAction.destination.empty())
          flag = isValidPlace(gpsrAction.destination);
      }
    }
  }
  return flag;
}
bool isValidObject(string objName)
{
  std::vector<string> objects{ "Water",  "Milk",    "Coke",        "Tonic",      "Bubble Tea", "Ice tea",  "Cloth",
                               "Sponge", "Cleaner", "Corn Flakes", "Tuna Can",   "Sugger",     "Mustard",  "Apple",
                               "Peach",  "Orange",  "Banana",      "Strawberry", "Pockys",     "Pringles", "Spoon",
                               "Fork",   "Plate",   "Bowl",        "Mug",        "Knife" };

  for (auto obj : objects)
  {
    std::string lowerObj = boost::to_upper_copy(obj);
    std::string lowerobjName = boost::to_upper_copy(objName);
    bool found = boost::algorithm::contains(lowerObj, lowerobjName);
    if (found)
    {
      return true;
    }
  }
  return false;
}

bool isValidPlace(string placeName)
{
  std::vector<string> Places;
  Places.push_back("House Plant");
  Places.push_back("Coat Rack");
  Places.push_back("Sofa");
  Places.push_back("Couch Table");
  Places.push_back("TV");
  Places.push_back("Side Table");
  Places.push_back("Book Shelf");
  Places.push_back("Pantry");
  Places.push_back("Dinner Table");
  Places.push_back("Kitchen Bin");
  Places.push_back("Fridge");
  Places.push_back("Washing Machine");
  Places.push_back("Sink");
  Places.push_back("Small Shelf");
  Places.push_back("Cupboard");
  Places.push_back("Big Shelf");
  Places.push_back("Bed");
  Places.push_back("Desk");
  Places.push_back("Show Rack");
  Places.push_back("Bin");
  Places.push_back("Office Shelf");

  for (auto place : Places)
  {
    std::string lowerPlace = boost::to_upper_copy(place);
    std::string lowerPlaceName = boost::to_upper_copy(placeName);
    bool found = boost::algorithm::contains(lowerPlace, lowerPlaceName);
    if (found)
    {
      return true;
    }
  }
  return false;
}

}  // namespace generic
}  // namespace dialog
}  // namespace robobreizh
