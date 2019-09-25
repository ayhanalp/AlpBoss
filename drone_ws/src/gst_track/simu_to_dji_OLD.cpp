#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

/* DJI SDK includes */
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include "dji_sdk/dji_sdk.h"


float requested_x_velocity = 0;
float requested_y_velocity = 0;
float requested_z_velocity = 0;
float requested_yaw_rate = 0;
float chatter1_enable = 0;

float requested_roll = 0;
float requested_pitch = 0;
float requested_height = 0;
float requested_yaw_rate_rp = 0;  //rp = roll pitch
float chatter2_enable = 0;

float requested_roll_gen = 0; //gen = generic: with FLAG
float requested_pitch_gen = 0;
float requested_height_gen = 0;
float requested_yaw_gen = 0;
float chatter3_enable = 0;

float requested_roll_gen2 = 0; //gen = generic: with FLAG
float requested_pitch_gen2 = 0;
float requested_thrust_gen2 = 0;
float requested_yaw_gen2 = 0;
float chatter4_enable = 0;


/*bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }
  else
  {
    ROS_INFO("obtain control Succesfull!");
  }

  return true;
}
*/


void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  requested_x_velocity = msg->linear.x;
  requested_y_velocity = msg->linear.y;
  requested_z_velocity = msg->linear.z;
  requested_yaw_rate = msg->angular.x;
  chatter1_enable = msg->angular.z;
  ROS_INFO("CHATTER 1: I heard: linear.x = [%f], linear.y = [%f], linear.z = [%f], angular.x = [%f]", requested_x_velocity, requested_y_velocity, requested_z_velocity, requested_yaw_rate);
}

void chatter2Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  requested_roll = msg->linear.x;
  requested_pitch = msg->linear.y;
  requested_height = msg->linear.z;
  requested_yaw_rate_rp = msg->angular.x;
  chatter2_enable = msg->angular.z;
  ROS_INFO("CHATTER 2: I heard: linear.x = [%f], linear.y = [%f], linear.z = [%f], angular.x = [%f]", requested_roll, requested_pitch, requested_height, requested_yaw_rate_rp);
}

void chatter3Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  requested_roll_gen = msg->linear.x;
  requested_pitch_gen = msg->linear.y;
  requested_height_gen = msg->linear.z;
  requested_yaw_gen = msg->angular.x;
  chatter3_enable = msg->angular.z;
  ROS_INFO("CHATTER 3: I heard: linear.x = [%f], linear.y = [%f], linear.z = [%f], angular.x = [%f]", requested_roll_gen, requested_pitch_gen, requested_height_gen, requested_yaw_gen);
}

void chatter4Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  requested_roll_gen2 = msg->linear.x;
  requested_pitch_gen2 = msg->linear.y;
  requested_thrust_gen2 = msg->linear.z;
  requested_yaw_gen2 = msg->angular.x;
  chatter4_enable = msg->angular.z;
  ROS_INFO("CHATTER 4: I heard: linear.x = [%f], linear.y = [%f], linear.z = [%f], angular.x = [%f]", requested_roll_gen2, requested_pitch_gen2, requested_thrust_gen2, requested_yaw_gen2);
}

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
  ros::init(argc, argv, "simu_to_dji");
  ROS_INFO("Starting Simulink to DJI translating node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   /* Basic subscribers */
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("chatter", 10, chatterCallback);
  ros::Subscriber sub2 = n.subscribe<geometry_msgs::Twist>("chatter2", 10, chatter2Callback);
  ros::Subscriber sub3 = n.subscribe<geometry_msgs::Twist>("chatter3", 10, chatter3Callback);
  ros::Subscriber sub4 = n.subscribe<geometry_msgs::Twist>("chatter4", 10, chatter4Callback);

  /* Basic publishers */
  ros::Publisher ctrlPosYawPub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ros::Publisher ctrlAttYawRatePub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  ros::Publisher ctrlAttYawPub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ros::Publisher ctrlAttThrustYawPub = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  /* Basic services */
  ros::ServiceClient sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");


  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("Obtain control failed");
  }
  else
  {
    ROS_INFO("Obtain control succesfull");
  }


  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    /* While ros is running, the Twist at chatter is converted into a sensor_msgs/Joy and send on dji_sdk/flight_control_setpoint_ENUposition_yaw*/
    //ROS_INFO("Obtain control result = [%d]", obtain_control_result);
    dji_sdk::SDKControlAuthority authority;

    if(!authority.response.result)
    {
      authority.request.control_enable=1;
      sdk_ctrl_authority_service.call(authority);
      //ROS_INFO("Trying to regain control");
      if(!authority.response.result)
      {
        //ROS_ERROR("Obtain control failed");
      }
      else
      {
        //ROS_INFO("Obtain control succesfull");
      }
    }

    sensor_msgs::Joy controlPosYaw;
    sensor_msgs::Joy controlAttYawRate;
    sensor_msgs::Joy controlAttYaw;

    if (chatter1_enable == 1){
      controlPosYaw.axes.push_back(requested_x_velocity);
      controlPosYaw.axes.push_back(requested_y_velocity);
      controlPosYaw.axes.push_back(requested_z_velocity);
      controlPosYaw.axes.push_back(requested_yaw_rate);
      ctrlPosYawPub.publish(controlPosYaw);
    }
    else if (chatter2_enable == 1) {
      controlAttYawRate.axes.push_back(requested_roll);
      controlAttYawRate.axes.push_back(requested_pitch);
      controlAttYawRate.axes.push_back(requested_height);
      controlAttYawRate.axes.push_back(requested_yaw_rate_rp);
      ctrlAttYawRatePub.publish(controlAttYawRate);
    }
    else if (chatter3_enable == 1) {
      uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_ANGLE            |
                  DJISDK::HORIZONTAL_BODY   |
                  DJISDK::STABLE_DISABLE);
      controlAttYaw.header.seq = 0;
      controlAttYaw.axes.push_back(requested_roll_gen);
      controlAttYaw.axes.push_back(requested_pitch_gen);
      controlAttYaw.axes.push_back(requested_height_gen);
      controlAttYaw.axes.push_back(requested_yaw_gen);
      controlAttYaw.axes.push_back(flag);
      ctrlAttYawPub.publish(controlAttYaw);
    }
    else if (chatter4_enable == 1) {
      uint8_t flag2 = (DJISDK::VERTICAL_THRUST   |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_ANGLE            |
                  DJISDK::HORIZONTAL_BODY   |
                  DJISDK::STABLE_DISABLE);
      controlAttYaw.header.seq = 0;
      controlAttYaw.axes.push_back(requested_roll_gen2);
      controlAttYaw.axes.push_back(requested_pitch_gen2);
      controlAttYaw.axes.push_back(requested_thrust_gen2);
      controlAttYaw.axes.push_back(requested_yaw_gen2);
      controlAttYaw.axes.push_back(flag2);
      ctrlAttYawPub.publish(controlAttYaw);
    }
    else {
      //ROS_INFO("All chatters disabled, to enable: angular.z == 1");
    }


    ros::spinOnce();

    loop_rate.sleep();
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();  // Is dit nodig??

  return 0;
}
