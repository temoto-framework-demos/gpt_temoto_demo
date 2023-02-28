
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
#include "ta_take_photo/temoto_action.h"
#include "temoto_component_manager/component_manager_interface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>

// FROM THE EXAMPLE: 
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages


/* 
 * ACTION IMPLEMENTATION of TaTakePhoto 
 */
class TaTakePhoto : public TemotoAction
{
public:

TaTakePhoto() : it_(nh_)
{
  image_sub_ = it_.subscribe("/operator/usb_cam/image_raw", 1, &TaTakePhoto::imageCb, this);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  TEMOTO_INFO_STREAM("Callback");
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
   // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);

  // Output modified video stream
  //image_pub_.publish(cv_ptr->toImageMsg());
}

/*
 * Function that gets invoked only once (when the action is initialized) throughout the action's lifecycle
 */
void initializeTemotoAction()
{
  
  TEMOTO_INFO_STREAM("Action initialized");
}

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();

  // Subscrive to input video feed and publish output video feed
  
  //image_pub_ = it_.advertise("/image_converter/output_video", 1);

  //cv::namedWindow(OPENCV_WINDOW);
  
  cmi_.initialize();

  // ComponentTopicsReq req_topics;
  // std::string image_topic = "/camera_left/color/image_raw";
  // req_topics.addOutputTopic("camera_data", image_topic);
  
  // cmi_.startComponent("realsense_left", "", "", req_topics);

  cmi_.startComponent("2d_camera");

  ComponentTopicsReq req_topics_save;
  // req_topics_save.addInputTopic("image_topic", "/vaultbot/camera_left/color/image_raw");
  req_topics_save.addInputTopic("image_topic", "usb_cam/image_raw");

  //ros::topic::waitForMessage<sensor_msgs::Image>("usb_cam/image_raw");
  
  //sensor_msgs::ImagePtr wft = ros::topic::waitForMessage("usb_cam/image_raw");

  ros::Duration(20).sleep();
  // std::string file_name = "image/" + std::to_string(ros::Time::now().toSec());
  // req_topics_save.addInputTopic("_filename_format", file_name + "%04d.%s");
  // cmi_.startComponent("save_image", req_topics_save);
  //cmi_.startComponent("save_image");

  setOutputParameters();

  TaTakePhoto ic;
}

// Destructor
~TaTakePhoto()
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

ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
// image_transport::Publisher image_pub_;
temoto_component_manager::ComponentManagerInterface cmi_;

}; // TaTakePhoto class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTakePhoto, ActionBase);
