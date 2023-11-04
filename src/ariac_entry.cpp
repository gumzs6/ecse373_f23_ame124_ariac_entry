/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"


// receiving order message
std::vector<osrf_gear::Order> order_vector;

// to track product type and bin its in
std::vector<std::string> pt_vector;
std::vector<std::vector<osrf_gear::StorageUnit>> pb_vector;

// logical cameras
std::vector<osrf_gear::LogicalCameraImage> agv_vector;
std::vector<osrf_gear::LogicalCameraImage> bin_vector;
std::vector<osrf_gear::LogicalCameraImage> qcs_vector;

// start comp
int service_call_succeeded;
std_srvs::Trigger begin_comp;

void orderCallback(const osrf_gear::Order::ConstPtr& order_msg)
{
  order_vector.clear();
  order_vector.push_back(*order_msg);
}

void agv1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  agv_vector[0] = *image_msg;   
}

void agv2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  agv_vector[1] = *image_msg;   
}

void bin1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[0] = *image_msg;   
}

void bin2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[1] = *image_msg;
}

void bin3Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[2] = *image_msg;   
}

void bin4Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[3] = *image_msg;
}

void bin5Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[4] = *image_msg;
}

void bin6Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  bin_vector[5] = *image_msg;
}

void qcs1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{ 
  qcs_vector[0] = *image_msg;
}

void qcs2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
  qcs_vector[1] = *image_msg;   
}

geometry_msgs::Pose getPose(const std::string& product_type, const osrf_gear::StorageUnit& stor_unit) 
{
      geometry_msgs::Pose product_pose;
      std::vector<osrf_gear::Model> models;
      std::string bin = stor_unit.unit_id;
  
      char number = bin.at(3);
      int binNum = number - '0';
      
      models = bin_vector[binNum - 1].models;

      // for every model in the cameras list, see if the type matches the desired product type
      for (const auto& model : models)
        {
          if (model.type == product_type) 
          {
             product_pose = model.pose;
          }
        }

      return product_pose;
}

void getProducts(const osrf_gear::Order order_msg, ros::ServiceClient *mat_loc_client) 
{
  for (const auto& shipment : order_msg.shipments)
  {
    for (const auto& product : shipment.products)
    {

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped tfStamped;
      geometry_msgs::PoseStamped part_pose, goal_pose;

      // get product type
      pt_vector.push_back(product.type);
  
      // get matieral location by setting the type to found product and returning the bins
      osrf_gear::GetMaterialLocations mat_loc;
      mat_loc.request.material_type = product.type;
      mat_loc_client->call(mat_loc);
  
      // get bin location from material locations service
      pb_vector.push_back(mat_loc.response.storage_units);

      // get product type, bin, and pose
      std::string bin = mat_loc.response.storage_units.front().unit_id;
      part_pose.pose = getPose(product.type, mat_loc.response.storage_units.front());
      ROS_INFO("Type: [%s], Bin: [%s], Pose: [%f] [%f] [%f]", product.type.c_str(), mat_loc.response.storage_units.front().unit_id.c_str(), part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
    
      // transformation between arm and logical camera frame

      

      try
      {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + bin + "_frame", ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
      }
      catch (tf2::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      
      tf2::doTransform(part_pose, goal_pose, tfStamped);
    
      goal_pose.pose.position.z += 0.10;
      goal_pose.pose.orientation.w = 0.707;
      goal_pose.pose.orientation.x = 0.0;
      goal_pose.pose.orientation.y = 0.707;
      goal_pose.pose.orientation.z = 0.0;

    }
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "ariac_entry");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  
  ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 10, orderCallback);
  
  ros::Subscriber logical_camera1_subscriber = n.subscribe("/ariac/logical_camera_agv1", 10, agv1Callback);
  ros::Subscriber logical_camera2_subscriber = n.subscribe("/ariac/logical_camera_agv2", 10, agv2Callback);
  ros::Subscriber logical_camera7_subscriber = n.subscribe("/ariac/logical_camera_bin1", 10, bin1Callback);
  ros::Subscriber logical_camera8_subscriber = n.subscribe("/ariac/logical_camera_bin2", 10, bin2Callback);
  ros::Subscriber logical_camera3_subscriber = n.subscribe("/ariac/logical_camera_bin3", 10, bin3Callback);
  ros::Subscriber logical_camera4_subscriber = n.subscribe("/ariac/logical_camera_bin4", 10, bin4Callback);
  ros::Subscriber logical_camera5_subscriber = n.subscribe("/ariac/logical_camera_bin5", 10, bin5Callback);
  ros::Subscriber logical_camera6_subscriber = n.subscribe("/ariac/logical_camera_bin6", 10, bin6Callback);
  ros::Subscriber logical_camera9_subscriber = n.subscribe("/ariac/quality_control_sensor_1", 10, qcs1Callback);
  ros::Subscriber logical_camera10_subscriber = n.subscribe("/ariac/quality_control_sensor_2", 10, qcs2Callback);

  ros::ServiceClient mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

  // start_competition service
  if (!begin_client.exists()) {
    ROS_WARN("WAiting for the compeition to be ready...");
    begin_client.waitForExistence();
    ROS_WARN("Compeition ready.");
  }
  service_call_succeeded = begin_client.call(begin_comp);

  if (!service_call_succeeded) {
  ROS_ERROR("Competition service call failed!");
  } 
  else if (service_call_succeeded) {
  ROS_INFO_STREAM("Competition service called successfully");
  }
  else 
  {
  ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
  }
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    order_vector.clear();
    agv_vector.clear();
    bin_vector.clear();
    qcs_vector.clear();
    pb_vector.clear();
    pt_vector.clear();

    for (const auto& order : order_vector) 
      {
        getProducts(order, &mat_loc_client);
      }

    ROS_INFO("First product has type [%s] and can be found in [%s]", pt_vector.front().c_str(), pb_vector.front().front().unit_id.c_str());
    
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
