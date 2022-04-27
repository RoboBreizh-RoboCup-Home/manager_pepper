#include <std_msgs/String.h>

#include "PlanHighLevelActions/ManipulationPlanActions.hpp"

using namespace std;

namespace robobreizh
{
namespace manipulation
{
namespace plan
{

void aGrabHandle(std::string params, bool* run)
{
    int i_object=params.find("_");
    int i_hand=params.find("_", i_object + 1);
    string object=params.substr(0, i_object);
    string hand=params.substr(i_object + 1, i_hand);


}

} // namespace plan
} // namespace manipulation
}// namespace robobreizh
