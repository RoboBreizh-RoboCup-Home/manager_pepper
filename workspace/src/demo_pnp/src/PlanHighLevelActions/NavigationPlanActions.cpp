#include <std_msgs/String.h>

#include "PlanHighLevelActions/NavigationPlanActions.hpp"
#include "GenericActions/NavigationGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace navigation
{
namespace plan
{
void aMoveTowardsObject(std::string params, bool* run)
{
    string object = params;
    cout << "moveTowardsObject - Currently moving torwards " << object << endl;
    robobreizh::navigation::generic::moveTowardsObject(object);
}
} // namespace plan
} // namespace navigation
} // namespace robobreizh
