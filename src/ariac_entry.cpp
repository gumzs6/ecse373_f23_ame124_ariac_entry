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
#include "ik_service/PoseIK.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

// receiving order message
std::vector<osrf_gear::Order> order_vector;

// material location service
ros::ServiceClient mat_loc_client;

//inverse kinematic service
ros::ServiceClient ik_client;

ros::Publisher trajectory_pub;
control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

// camera to robot transformations
tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;

// current state of joints of robot
sensor_msgs::JointState joint_states;

// start comp
int service_call_succeeded;
std_srvs::Trigger begin_comp;

// for kinematic system
ik_service::PoseIK ik_pose;
int count = 0;

//  logical camera images
std::map<std::string, osrf_gear::LogicalCameraImage> cam_images;

void logCamCallback(const ros::MessageEvent<osrf_gear::LogicalCameraImage const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  std::string binName = topic.substr(22);
  boost::shared_ptr<osrf_gear::LogicalCameraImage const> msg = event.getMessage();
  
  cam_images[binName] = *msg;

}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
  joint_states = *joint_msg;
}

// callback when goal becomes active
void goalACtiveCallback() {
  //actionlib::SimpleClientGoalState goal_state = trajectory_ac.getState();
  //ROS_INFO("TrajectoryFollow in state: [%s]", goal_state.toString().c_str());
}

// callback when feedback provided
void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& fb) {
  //actionlib::SimpleClientGoalState goal_state = trajectory_ac.getState();
  //ROS_INFO("TrajectoryFollow in state: [%s]", goal_state.toString().c_str());
}

// callback when action complete and provides result
void resultCallback(const control_msgs::FollowJointTrajectoryResultConstPtr& res) {
  //actionlib::SimpleClientGoalState goal_state = trajectory_ac.getState();
  //ROS_INFO("TrajectoryFollow in state: [%s]", goal_state.toString().c_str());
}


trajectory_msgs::JointTrajectory jointTrajectorySetup(trajectory_msgs::JointTrajectory joint_trajectory) 
{
  
  // Fill out the joint trajectory header.
  // Each joint trajectory should have an non-monotonically increasing sequence number.
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
  joint_trajectory.header.frame_id = "/world";

  // Set the names of the joints being used. All must be present.
  joint_trajectory.joint_names.clear();
  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");

  // Set a start and end point.
  joint_trajectory.points.resize(2);

  return joint_trajectory;
  
}

void armTrajectory()
{
  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory = jointTrajectorySetup(joint_trajectory);

// Set the start point to the current position of the joints from joint_states.
  joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < joint_states.name.size(); indz++) {
      if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
        break;
      }
    }
  }
    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    
    // Must select which of the num_sols solutions to use. Just start with the first.
    int q_sols_indx = 0;
    
    // Set the end point for the movement
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    
    // Set the linear_arm_actuator_joint from joint_states because not part of the ik sols
    joint_trajectory.points[1].positions[0] = joint_states.position[1];
    
    ik_pose.request.part_pose = goal_pose.pose;
    ik_client.call(ik_pose);

    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) 
    {
      joint_trajectory.points[1].positions[indy + 1] = ik_pose.response.joint_solutions.front().joint_angles.at(indy);
    }

    // How long to take for the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
    
    joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;

    trajectory_pub.publish(joint_trajectory);
 }

void getPose(const std::string& product_type, const std::string& bin) 
 {

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

      part_pose.pose = cam_images[bin].models[1].pose;
      
      tf2::doTransform(part_pose, goal_pose, tfStamped);

      goal_pose.pose.position.z += 0.10;
      goal_pose.pose.orientation.w = 0.707;
      goal_pose.pose.orientation.x = 0.0;
      goal_pose.pose.orientation.y = 0.707;
      goal_pose.pose.orientation.z = 0.0;  

      armTrajectory();

      ROS_WARN_STREAM("Pose in reference frame of robot: \n" << goal_pose.pose);
      
  }

void orderCallback(const osrf_gear::Order::ConstPtr& order_msg)
{
  order_vector.push_back(*order_msg);
  osrf_gear::Order currentOrder = *order_msg;

  for (osrf_gear::Shipment shipment : currentOrder.shipments)
  {
    for (osrf_gear::Product product : shipment.products)
    {
      osrf_gear::GetMaterialLocations mat_loc;

      mat_loc.request.material_type = product.type;
      mat_loc_client.call(mat_loc);
      std::string bin = mat_loc.response.storage_units.front().unit_id;

      ROS_WARN_STREAM("Type: " << product.type.c_str() << "\nBin: " << bin.c_str() << "\nPose in frame of camera:\n" << product.pose);

      getPose(product.type, bin);
  }
}
}

void getFirstProduct() 
{
    osrf_gear::GetMaterialLocations mat_loc;

    std::string prod_type = order_vector.front().shipments.front().products.front().type; 

    mat_loc.request.material_type = prod_type;
    mat_loc_client.call(mat_loc);
    
    std::string stor_unit = mat_loc.response.storage_units.front().unit_id;
    
    ROS_INFO_ONCE("First product in the first shipment has type %s and can be found in %s", prod_type.c_str(), stor_unit.c_str());

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
  
  ros::Subscriber logical_camera1_subscriber = n.subscribe("/ariac/logical_camera_agv1", 10, logCamCallback);
  ros::Subscriber logical_camera2_subscriber = n.subscribe("/ariac/logical_camera_agv2", 10, logCamCallback);
  ros::Subscriber logical_camera7_subscriber = n.subscribe("/ariac/logical_camera_bin1", 10, logCamCallback);
  ros::Subscriber logical_camera8_subscriber = n.subscribe("/ariac/logical_camera_bin2", 10, logCamCallback);
  ros::Subscriber logical_camera3_subscriber = n.subscribe("/ariac/logical_camera_bin3", 10, logCamCallback);
  ros::Subscriber logical_camera4_subscriber = n.subscribe("/ariac/logical_camera_bin4", 10, logCamCallback);
  ros::Subscriber logical_camera5_subscriber = n.subscribe("/ariac/logical_camera_bin5", 10, logCamCallback);
  ros::Subscriber logical_camera6_subscriber = n.subscribe("/ariac/logical_camera_bin6", 10, logCamCallback);
  ros::Subscriber logical_camera9_subscriber = n.subscribe("/ariac/quality_control_sensor_1", 10, logCamCallback);
  ros::Subscriber logical_camera10_subscriber = n.subscribe("/ariac/quality_control_sensor_2", 10, logCamCallback);

  ros::Subscriber joint_states_subscriber = n.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);
  trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 10);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm1/arm/follow_joint_trajectory", true);  

  ik_client = n.serviceClient<ik_service::PoseIK>("/ik_service");
  
  mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  if (!ros::service::waitForService("/ariac/material_locations", 10)) 
  {
    ROS_WARN("Get Material Locations service not found");
    mat_loc_client.waitForExistence();
  }
  else
  {
    ROS_WARN("Get Material Locations service found");
  }
  
  if (!ros::service::waitForService("pose_ik", 10)) 
  {
    ROS_WARN("Inverse Kinematic service not found");
  }
  else
  {
    ROS_WARN("Inverse Kinematic service found");
  }
  
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  
  tf2_ros::TransformListener tfListener(tfBuffer);

  // start_competition service
  if (!begin_client.exists()) {
    ROS_WARN("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_WARN("Competition ready.");
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

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%

    
    if (order_vector.size() == 1) 
    {
      getFirstProduct();
    }

    ROS_INFO_STREAM_THROTTLE(10, joint_states);

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
//  ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
