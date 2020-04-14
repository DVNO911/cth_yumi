#include <ros/ros.h>
#include <std_msgs/String.h>
#include <generic_control_toolbox/kdl_manager.hpp>


// sensor_msgs::JointState state;
float Kp = 3.5; 
String chain_end_effector_name_r = "gripper_r_base"; // Kept as global variables because it might be useful to use other frames as reference
String chain_end_effector_name_l = "gripper_l_base";

void stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
}

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "task_space");
  ros::NodeHandle nh;

  // Define the communications with the robot

  // Subscriber
  ros::Subscriber state_sub = nh.subscribe("/joint_states", 1, &stateCb);

  // Publishers
  // ros::Publisher command_pub = nh.advertise<sensor_msgs::JointState>("/joint_command", 1);
  std::list<ros::Publisher> publishers;
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_1_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_2_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_7_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_3_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_4_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_5_r/command", 1)
  publishers.append(nh.advertise<sensor_msgs::JointState>("/yumi/joint_vel_controller_6_r/command", 1)

  // Initialize a KDL manager on the robot's left arm
  generic_control_toolbox::KDLManager manager("yumi_base_link");
  manager.initializeArm(chain_end_effector_name_r);

  KDL::Frame pose;
  KDL::Vector desired_position, error;
  KDL::JntArray q_dot(7);
  KDL::Twist command_vel = KDL::Twist::Zero();
  sensor_msgs::JointState command;

  // Parse goal pose
  desired_position = getDesiredPosition();

  // LOOP
  while (ros::ok())
  {
    // Get the end-effector pose as a KDL::Frame.
    if (manager.getEefPose(chain_end_effector_name_r, state, pose))
    {
      error = desired_position - pose.p;
      command_vel.vel = Kp*error;
      ROS_INFO_THROTTLE(Kp,
                        "Position error: (%.2f, %.2f, %.2f)",
                        error.x(), error.y(), error.z());

      // Get the IK solution for the desired control command.
      manager.getVelIK(chain_end_effector_name_r, state, command_vel, q_dot);
      command = state;
      manager.getJointState(chain_end_effector_name_r, q_dot.data, command);
      command_pub.publish(command);
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}

KDL::Vector getDesiredPosition(){
  KDL::Vector desired_position;
  // TO DO: Parse from YAMLfile
  desired_position.x(0.288638);
  desired_position.y(-0.328583);
  desired_position.z(0.478045);
  return desired_position;
}