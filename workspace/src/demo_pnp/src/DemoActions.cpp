#include <ros/ros.h>
//#include <actionlib/server/simple_action_server.h>
//#include <actionlib/client/simple_action_client.h>
//#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <pnp_ros/PNPActionServer.h>

//#include <rp_action_msgs/TurnAction.h>

#include <boost/thread/thread.hpp>

std::string robotname="Roy Batty";

using namespace std;

/*** ACTIONS ***/

void start_init(string params, bool* run) {
  cout << "inside start_init" << endl;
  *run = 1;
}

void check_color(string params, bool* run)
{
  ros::NodeHandle nh;
  cout << "Waiting input from /computer_vision/color" << endl;
  *run = 1;
  boost::shared_ptr<std_msgs::String const> shared_msg;
  std_msgs::String msg;
  shared_msg = ros::topic::waitForMessage<std_msgs::String>("/computer_vision/color", nh);
  
  if (shared_msg != NULL)
  {
    msg = *shared_msg;
    std::cout<<"Color = "<< msg.data <<std::endl;
    *run = 1;
  }
  
  else 
  {
    std::cout<<"No object found !"<<std::endl;
    *run = 0;
  }
  
}

void demo_ros_service(string params, bool* run)
{
  // not there yet
  *run = 1;
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

void acheck_color(string params, bool* run) {
  cout << "### Executing check_color ... " << params << endl;
  check_color(params, run);
  if (*run)
    cout << "### Finished check_color " << endl;
  else
    cout << "### Aborted check_color  " << endl;
}

void ademo_ros_service(string params, bool* run) {
  cout << "### Executing demo_ros_service ... " << params << endl;
  demo_ros_service(params, run);
  if (*run)
    cout << "### Finished demo_ros_service " << endl;
  else
    cout << "### Aborted demo_ros_service  " << endl;
}