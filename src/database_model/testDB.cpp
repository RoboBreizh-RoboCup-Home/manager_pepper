#include "DatabaseModel/NavigationModel.hpp"
#include "DatabaseModel/DialogModel.hpp"
#include "DatabaseModel/VisionModel.hpp"

int main()
{
  /* navigation model test */
  robobreizh::NavigationPlace np;
  robobreizh::database::NavigationModel nm;
  std::cout << "Test getLocationFromName" << std::endl;
  np = nm.getLocationFromName("instructionPoint");
  std::cout << np.name << std::endl;
  std::cout << np.frame << std::endl;
  std::cout << np.pose.position.x << std::endl;
  std::cout << np.pose.position.y << std::endl;
  std::cout << np.pose.position.z << std::endl;
  std::cout << np.pose.orientation.x << std::endl;
  std::cout << np.pose.orientation.y << std::endl;
  std::cout << np.pose.orientation.z << std::endl;
  std::cout << np.pose.orientation.w << std::endl;
  std::cout << std::endl;

  /* dialog model test */
  robobreizh::database::DialogModel dm;

  std::cout << "Test selectLastPersonId" << std::endl;
  std::cout << dm.selectLastPersonId() << std::endl;
  std::cout << std::endl;

  std::cout << "Test updatePersonName" << std::endl;
  dm.updatePersonName("Thomas");
  std::cout << std::endl;

  std::cout << "Test updatePersonFavoriteDrink" << std::endl;
  dm.updatePersonFavoriteDrink("Wine");
  std::cout << std::endl;

  /* Vision model test */
  robobreizh::database::VisionModel vm;
  std::cout << "Test getColorByLabel" << std::endl;
  int colorId = vm.getColorByLabel("black");
  std::cout << colorId << std::endl;
  std::cout << std::endl;

  std::cout << "Test createPersonFromFeatures" << std::endl;
  vm.createPersonFromFeatures("male", 25, "black", "white");
  std::cout << std::endl;

  std::cout << "Test selectLastPerson" << std::endl;
  robobreizh::Person person;
  person = vm.selectLastPerson();
  std::cout << "name: " << person.name << std::endl;
  std::cout << "drink: " << person.favorite_drink << std::endl;
  std::cout << "gender: " << person.gender << std::endl;
  std::cout << "age: " << person.age << std::endl;
  std::cout << "cloth color: " << person.cloth_color << std::endl;
  std::cout << "skin color: " << person.skin_color << std::endl;
  std::cout << std::endl;
  return 0;
}
