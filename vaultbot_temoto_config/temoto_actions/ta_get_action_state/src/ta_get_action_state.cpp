
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
#include "ta_get_action_state/temoto_action.h"
#include "temoto_action_engine/GetUmrfGraphs.h"
#include "std_msgs/String.h"

/* 
 * ACTION IMPLEMENTATION of TaGetActionState 
 */
class TaGetActionState : public TemotoAction
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

  action_status = nh_.advertise<std_msgs::String>(action_status_topic, 10);
  get_action_state = nh_.serviceClient<temoto_action_engine::GetUmrfGraphs>(get_umrf_graphs_srv_name_);
  
  TEMOTO_INFO_STREAM("Call service ");
  temoto_action_engine::GetUmrfGraphs get_umrfg;
  get_umrfg.request.requested_graphs.push_back("");
  
  while (actionOk())
  {
    std_msgs::String msg;
    if (get_action_state.call(get_umrfg))
    {
      // TEMOTO_INFO_STREAM("size: " + std::to_string(get_umrfg.response.action_name.size()));
      for (int i=0; i < get_umrfg.response.action_name.size(); i++)
      {
        msg.data = get_umrfg.response.action_name[i] + ";" + get_umrfg.response.action_state[i];
        action_status.publish(msg);
        ros::Duration(0.1).sleep();
      }
    }
    ros::Duration(2).sleep();
  }

  setOutputParameters();
}

// Destructor
~TaGetActionState()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

ros::NodeHandle nh_;
ros::Publisher action_status;
ros::ServiceClient get_action_state;

std::string get_umrf_graphs_srv_name_ = "get_umrf_graphs";
std::string action_status_topic = "/temoto/umrf_status";

}; // TaGetActionState class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaGetActionState, ActionBase);
