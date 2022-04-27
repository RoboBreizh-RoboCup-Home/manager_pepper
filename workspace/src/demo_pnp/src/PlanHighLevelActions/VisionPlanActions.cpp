#include <std_msgs/String.h>

#include "PlanHighLevelActions/VisionPlanActions.hpp"
#include "GenericActions/VisionGenericActions.hpp"

using namespace std;

namespace robobreizh
{
namespace vision
{
namespace plan
{
void aWaitForOperator(string params, bool* run)
{
    *run = generic::waitForHuman();
}

void aFindObject(string params, bool* run)
{
    string objectToFind = params;
    cout << "FindObject - Currently looking for " << objectToFind << endl;
    *run = robobreizh::vision::generic::findObject(objectToFind);
}
} // namespace plan
} // namespace vision
}// namespace robobreizh