#include "DatabaseModel/NavigationModel.hpp"

int main(){
    robobreizh::NavigationPlace np;
    robobreizh::database::NavigationModel nm;
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
    return 0;
}
