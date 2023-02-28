
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
#include "ta_take_picture/temoto_action.h"
#include "temoto_component_manager/component_manager_interface.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/CompressedImage.h"
/* 
 * ACTION IMPLEMENTATION of TaTakePicture 
 */
class TaTakePicture : public TemotoAction
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
  
  cmi_.initialize();
  cmi_.startComponent(component);

  image_sub = nh_.subscribe("/vaultbot/camera_left/color/image_raw/compressed", 1, &TaTakePicture::imageCallback, this);

  image_pub = nh_.advertise<sensor_msgs::CompressedImage>("/vaultbot/photo",1000);
  
  auto component_list = cmi_.listComponents(component);
  std::string image_topic = "camera/image";
  
  if (component_list.local_components.size()>0)
  {
    std::string temoto_namespace_comp = component_list.local_components.begin()->temoto_namespace;
    std::string camera_topic;
    std::vector<diagnostic_msgs::KeyValue> output_topic_vec = component_list.local_components.begin()->output_topics;
    for (const auto& l_topic : output_topic_vec)
    {
      if (l_topic.key == "camera_data")
      {
        camera_topic = l_topic.value;
        TEMOTO_INFO_STREAM("Found local component with topic: '" << camera_topic);
        break;
      }
    }
    image_topic = "/" + temoto_namespace_comp + "/" + camera_topic;                             
    TEMOTO_INFO_STREAM("image_topic: '" << image_topic << std::endl);      
  }
  else if (component_list.remote_components.size())
  {      
    std::string temoto_namespace_comp = component_list.remote_components.begin()->temoto_namespace;
    std::string camera_topic;
    std::vector<diagnostic_msgs::KeyValue> output_topic_vec = component_list.remote_components.begin()->output_topics;
    for (const auto& r_topic : output_topic_vec)
    {
      if (r_topic.key == "camera_data")
      {        
        camera_topic = r_topic.value;
        TEMOTO_INFO_STREAM("Found remote component with topic: '" << camera_topic);
        break;
      }
    }
    image_topic = "/" + temoto_namespace_comp + "/" + camera_topic;
    TEMOTO_INFO_STREAM("image_topic: '" << image_topic << std::endl);
  }
  else
  {
    TEMOTO_INFO_STREAM("Component description not found: " << component << std::endl);  
  }

  // Save Image

  if (ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, ros::Duration(30)))
  {
    ComponentTopicsReq req_topics_save;
    req_topics_save.addInputTopic("image_topic", image_topic);  
    std::string file_name = "image_" + std::to_string(ros::Time::now().toSec()) + ".png";

    req_topics_save.addInputTopic("filename", file_name);
    cmi_.startComponent("save_image", req_topics_save);
    
    save_image = nh_.serviceClient<std_srvs::Empty>("camera_controller/save");  
    std_srvs::Empty srv;
    save_image.waitForExistence();

    if (save_image.call(srv))
    {
      ROS_INFO_STREAM("Message: Save Image with name: " << file_name);

      sensor_msgs::CompressedImage image_out;
      image_out.header.frame_id = file_name;
      image_out.format = "rgb8; jpeg compressed bgr8";
      image_out.data = item.data;
      image_pub.publish(image_out);

    }
    else
    {
      ROS_INFO_STREAM("Failed to call service");
    }
    //wait while the image is saved
    ros::Duration(15).sleep();
  }
  else
  {
    ROS_INFO_STREAM("No message on topic " << image_topic << " received");    
  }
  setOutputParameters();

  TEMOTO_INFO_STREAM("\n***************************************************\n********************** TAKE PHOTO *****************\n***************************************************\n");


  setOutputParameters();
}

// Destructor
~TaTakePicture()
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

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    item.format = msg->format;
    // std::cout << msg->data.size() << std::endl;
    item.data = msg->data;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ros::NodeHandle nh_;
temoto_component_manager::ComponentManagerInterface cmi_;
ros::ServiceClient save_image;

ros::Subscriber image_sub;
ros::Publisher image_pub;
sensor_msgs::CompressedImage item;

std::string component = "realsense_left";

}; // TaTakePicture class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTakePicture, ActionBase);

