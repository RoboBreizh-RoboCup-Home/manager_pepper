#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pnp_ros/PNPActionServer.h>
#include <robobreizh_demo_components/PepperSpeech.h>
#include <robobreizh_demo_components/Person.h>

#include <boost/thread/thread.hpp>

std::string robotname="Roy Batty";

using namespace std;

/*** GENERIC ACTIONS ***/
void robot_speech(string params, bool*run)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robobreizh_demo_components::PepperSpeech>("pepper_speech");
  robobreizh_demo_components::PepperSpeech srv;

  srv.request.Text = params;

  if (client.call(srv))
  {
    // TODO Add other use case if speech fails inside server (we already have this information)
    *run = 1;
  }
  else
  {
    cout << "Failed to call service pepper_speech" << endl;
    *run = 0;
  }
}

/*** ACTIONS ***/
void start_init(string params, bool* run) {
  cout << "inside start_init" << endl;
  *run = 1;
}

void wait_for_go_signal(string params, bool* run)
{
  ros::NodeHandle nh;
  cout << "Initialisation done, waiting for go signal from /robobreizh/manager/go" << endl;
  *run = 1;
  boost::shared_ptr<std_msgs::String const> shared_msg;
  std_msgs::String msg;
  shared_msg = ros::topic::waitForMessage<std_msgs::String>("/robobreizh/manager/go", nh);
  
  if (shared_msg != NULL)
  {
    msg = *shared_msg;
    std::cout<<"Let's go!"<< msg.data <<std::endl;
    *run = 1;
  }
  
  else 
  {
    std::cout<<"ERROR"<<std::endl;
    *run = 0;
  }
}
void greetings(string params, bool* run)
{
  string text = "Hello I'm Pepper, Welcome. Can you come closer please";
  robot_speech(text, run);
}

void wait_for_human(string params, bool* run)
{
  ros::NodeHandle nh;
  cout << "Waiting for person information from /robobreizh/perception/face_info" << endl;
  *run = 1;
  boost::shared_ptr<robobreizh_demo_components::Person const> shared_msg;
  robobreizh_demo_components::Person person_info;
  shared_msg = ros::topic::waitForMessage<robobreizh_demo_components::Person>("/robobreizh/perception/face_info", nh);
  
  if (shared_msg != NULL)
  {
    std::cout<<"Found someone"<<std::endl;
    *run = 1;
  }
  
  else 
  {
    std::cout<<"ERROR"<<std::endl;
    *run = 0;
  }
}

// Action implementation

void ainit(string params, bool* run) {
  cout << "### Executing Init ... " << params << endl;
  start_init(params, run);
  if (*run)
      cout << "### Finished Init " << endl;
  else
      cout << "### Aborted Init  " << endl;
}

void await_for_go_signal(string params, bool* run) {
  cout << "### Executing Wait for go signal ... " << params << endl;
  wait_for_go_signal(params, run);
  if (*run)
      cout << "### Finished Wait for go signal " << endl;
  else
      cout << "### Aborted Wait for go Wait for go signal  " << endl;
}

void agreetings(string params, bool* run) {
  cout << "### Executing greetings ... " << params << endl;
  greetings(params, run);
  if (*run)
    cout << "### Finished greetings " << endl;
  else
    cout << "### Aborted greetings  " << endl;
}

void await_for_human(string params, bool* run) {
  cout << "### Executing Wait for human ... " << params << endl;
  wait_for_human(params, run);
  if (*run)
    cout << "### Finished Wait for human " << endl;
  else
    cout << "### Aborted Wait for human  " << endl;
}