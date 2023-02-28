
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <class_loader/class_loader.hpp>
#include "ta_navigate/temoto_action.h"
#include "temoto_robot_manager/robot_manager_interface.h"
#include "tf/tf.h"

/* 
 * ACTION IMPLEMENTATION of TaNavigate 
 */
class TaNavigate : public TemotoAction
{
public:

/*
 * Function that gets invoked only once (when the action is initialized) throughout the action's lifecycle
 */
void initializeTemotoAction()
{
  /* * * * * * * * * * * * * * * * * * * * * * *
   *                          
   * ===> YOUR INITIALIZATION ROUTINES HERE <===
   *                          
   * * * * * * * * * * * * * * * * * * * * * * */

  TEMOTO_INFO_STREAM("Action initialized");
}

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  
  rmi_.initialize();
  rmi_.loadRobot(robot_name);

  /*
   * Move the robot
   */
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "anchor_ef7f288e_fa46_420e_b0f0_4fcf45c1a04d";
  target_pose.pose.position.x = in_param_pose_2d_x;
  target_pose.pose.position.y = in_param_pose_2d_y;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, in_param_pose_2d_yaw);

  bool goal_reached = false;
  while (!goal_reached && actionOk())
  try
  {
    TEMOTO_INFO_STREAM_("Sending a navigation goal to " << robot_name << " ...");
    rmi_.navigationGoal(robot_name, target_pose);

    TEMOTO_INFO_STREAM_("Done navigating");
    goal_reached = true;
  }
  catch(const resource_registrar::TemotoErrorStack &e)
  {
    TEMOTO_WARN_STREAM_("Caught an error: " << e.what() << "\nRequesting the same navigation goal again ... ");
  }
}

// Destructor
~TaNavigate()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_pose_2d_x = GET_PARAMETER("pose_2d::x", double);
  in_param_pose_2d_y = GET_PARAMETER("pose_2d::y", double);
  in_param_pose_2d_yaw = GET_PARAMETER("pose_2d::yaw", double);
  in_param_location = GET_PARAMETER("location", std::string);
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
temoto_robot_manager::RobotManagerInterface rmi_;

// Declaration of input parameters
double in_param_pose_2d_x;
double in_param_pose_2d_y;
double in_param_pose_2d_yaw;
std::string in_param_location;
std::string robot_name = "vaultbot";


}; // TaNavigate class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaNavigate, ActionBase);
