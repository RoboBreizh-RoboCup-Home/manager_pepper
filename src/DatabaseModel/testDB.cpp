#include "DatabaseModel/NavigationModel.hpp"
#include "DatabaseModel/DialogModel.hpp"

int main(){
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
    return 0;
}
