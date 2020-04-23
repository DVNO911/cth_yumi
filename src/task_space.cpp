#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <std_msgs/Float64.h>
#include <generic_control_toolbox/kdl_manager.hpp>


sensor_msgs::JointState state; // The current state as recorded by the subscriber 
float Kp = 1.5; 
std::string chain_end_effector_name_r = "gripper_r_base"; // Kept as global variables because it might be useful to use other frames as reference
std::string chain_end_effector_name_l = "gripper_l_base";

// Subscriber 
void stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
}


KDL::Vector getDesiredPosition_r(){
  KDL::Vector desired_position;
  // TO DO: Parse from YAMLfile
  desired_position.x(0.288638);
  desired_position.y(-0.328583);
  desired_position.z(0.478045);
  return desired_position;
}

KDL::Vector getDesiredPosition_l(){
  KDL::Vector desired_position;
  // TO DO: Parse from YAMLfile
  desired_position.x(0.288638);
  desired_position.y(0.328583);
  desired_position.z(0.378045);
  return desired_position;
}

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "task_space");
  ros::NodeHandle nh;

  // Subscriber
  ros::Subscriber state_sub = nh.subscribe("/joint_states", 1, &stateCb);

  // Publishers
  ros::Publisher publishers[] = {nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_1_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_2_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_7_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_3_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_4_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_5_r/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_6_r/command", 1),

                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_1_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_2_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_7_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_3_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_4_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_5_l/command", 1),
                           nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_6_l/command", 1),

                           // MISSING: GRIPPER PUBLISHERS
                          };



  // Initialize a KDL manager on the robot's left arm
  generic_control_toolbox::KDLManager manager("yumi_base_link", nh);  
  manager.initializeArm(chain_end_effector_name_r);
  manager.initializeArm(chain_end_effector_name_l);

  KDL::Frame pose_r, pose_l;
  KDL::Vector desired_position_r, desired_position_l, error_r, error_l;
  KDL::JntArray q_dot_r(7), q_dot_l(7);
  KDL::Twist command_vel_r = KDL::Twist::Zero();
  KDL::Twist command_vel_l = KDL::Twist::Zero();
  sensor_msgs::JointState command_r, command_l;

  // Parse goal pose
  desired_position_r = getDesiredPosition_r();
  desired_position_l = getDesiredPosition_l();


  // LOOP
  while (ros::ok())
  {
    // Get the end-effector pose as a KDL::Frame.
    if (manager.getEefPose(chain_end_effector_name_r, state, pose_r)) // When code runs its gonna complain a couple times that it hasnt found a chain joint state, this is because the subscriber hasnt recieved a message yet
    {
      manager.getEefPose(chain_end_effector_name_l, state, pose_l);
      ROS_INFO_STREAM("getEefPose computed");
      error_r = desired_position_r - pose_r.p;
      error_l = desired_position_l - pose_l.p;
      command_vel_r.vel = Kp*error_r;
      command_vel_l.vel = Kp*error_l;
      ROS_INFO_THROTTLE(0.1,
                        "Position error_r: (%.4f, %.4f, %.4f)",
                        error_r.x(), error_r.y(), error_r.z());

      ROS_INFO_THROTTLE(0.1,
                        "Position error_l: (%.4f, %.4f, %.4f)",
                        error_l.x(), error_l.y(), error_l.z());

      // Get the IK solution for the desired control command.
      manager.getVelIK(chain_end_effector_name_r, state, command_vel_r, q_dot_r);
      manager.getVelIK(chain_end_effector_name_l, state, command_vel_l, q_dot_l);

      command_r = state;
      command_l = state;

      manager.getJointState(chain_end_effector_name_r, q_dot_r.data, command_r);
      manager.getJointState(chain_end_effector_name_l, q_dot_l.data, command_l);

    
      // Publish velocities_r
      for (int i = 0; i < 7; i++){
      publishers[i].publish(command_r.velocity[i]);
      }

      for (int i = 0; i < 7; i++){
      publishers[i+7].publish(command_l.velocity[i+7]);
      }

    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}

