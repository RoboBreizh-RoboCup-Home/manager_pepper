#include <std_msgs/String.h>

#include "PlanHighLevelActions/InitiailisationPlanActions.hpp"

using namespace std;

namespace robobreizh
{
namespace initialisation
{
namespace plan
{

void aInitCarryMyLuggage (string params, bool* run)
{
    cout << "1.1 Carry My Luggage - initialisation done" << endl;
    *run = 1;
}

} // namespace plan
} // namespace initialisation
} // namespace robobreizh
