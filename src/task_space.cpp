#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include <generic_control_toolbox/kdl_manager.hpp>


sensor_msgs::JointState state; // The current state as recorded by the subscriber 
float Kp = 1; 
std::string chain_end_effector_name_r = "gripper_r_finger_l"; // Kept as global variables because it might be useful to use other frames as reference
std::string chain_end_effector_name_l = "gripper_l_finger_r";

int counter = 0;
int pose_counter = 0;

// Subscriber 
void stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
}


KDL::Vector getDesiredPosition_r1(){
  KDL::Vector desired_position;
  // TO DO: Parse from YAMLfile
  desired_position.x(0.288638);
  desired_position.y(-0.328583);
  desired_position.z(0.478045);  
  return desired_position;
}

  KDL::Vector getDesiredPosition_r2(){
  KDL::Vector desired_position;
  desired_position.x(0.288638);
  desired_position.y(-0.228583);
  desired_position.z(0.278045);
return desired_position;
  }

KDL::Vector getDesiredPosition_r3(){
  KDL::Vector desired_position;
  desired_position.x(0.288638);
  desired_position.y(-0.328583);
  desired_position.z(0.378045);
  return desired_position;
}

KDL::Vector getDesiredPosition_l1(){
  KDL::Vector desired_position;
  // TO DO: Parse from YAMLfile
  desired_position.x(0.288638);
  desired_position.y(0.328583);
  desired_position.z(0.378045);
  return desired_position;
}
KDL::Vector getDesiredPosition_l2(){
  KDL::Vector desired_position;
  desired_position.x(0.288638);
  desired_position.y(0.128583);
  desired_position.z(0.478045);
  return desired_position;
}

KDL::Vector getDesiredPosition_l3(){
  KDL::Vector desired_position;
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

  KDL::Frame pose_r, pose_l, pose_ref;
  // KDL::Vector desired_position_r1, desired_position_l1, desired_position_r2, desired_position_l2, desired_position_r3, desired_position_l3;
  KDL::Vector error_r, error_l, error_ref;
  KDL::JntArray q_dot_r(7), q_dot_l(7), q_dot_ref(7);
  KDL::Twist twist_r = KDL::Twist::Zero();
  KDL::Twist twist_l = KDL::Twist::Zero();
  KDL::Twist twist_ref = KDL::Twist::Zero();
  sensor_msgs::JointState command_r, command_l;

  KDL::Jacobian jacobian_r = KDL::Jacobian();
  KDL::Jacobian jacobian_l = KDL::Jacobian();


  // Parse goal poses
  // desired_position_r1 = getDesiredPosition_r1();
  KDL::Vector desired_position_r1;

  desired_position_r1.data[0] = 0.288638;
  desired_position_r1.data[1] = -0.128583;
  desired_position_r1.data[2] = 0.478045;
  // desired_position_l1 = getDesiredPosition_l1();

  // desired_position_r2 = getDesiredPosition_r2();
  // desired_position_l2 = getDesiredPosition_l2();

  // desired_position_r3 = getDesiredPosition_r3();
  // desired_position_l3 = getDesiredPosition_l3();


  // LOOP
  while (ros::ok())
  {

      ROS_INFO_STREAM("~ ~ ~ NEW LOOP ~ ~ ~");
    // Get the end-effector pose as a KDL::Frame.
    // When code runs its gonna complain a couple times that it hasnt found a chain joint state, this is because the subscriber hasnt recieved a message yet
    if (manager.getEefPose(chain_end_effector_name_r, state, pose_r)) 
    {
      manager.getEefPose(chain_end_effector_name_l, state, pose_l);
      ROS_INFO_STREAM("getEefPose computed");

      // Calculate Errors
      error_r = pose_r.p - desired_position_r1 ;
      error_l = pose_l.p - desired_position_r1 ;
      error_ref = pose_r.p - pose_l.p;


      twist_r.vel = - Kp*error_r; //P prick
      twist_l.vel = - Kp*error_l;  // Not used anymore

      //Pref = Pr - Pl
      pose_ref = pose_r;
      pose_ref.p.operator-=(pose_l.p); 

      ROS_INFO_STREAM("1 pose_r:");
      ROS_INFO_STREAM(pose_r.p[0]);
      ROS_INFO_STREAM(pose_r.p[1]);
      ROS_INFO_STREAM(pose_r.p[2]);
      ROS_INFO_STREAM("1.2 pose_l:");
      ROS_INFO_STREAM(pose_l.p[0]);
      ROS_INFO_STREAM(pose_l.p[1]);
      ROS_INFO_STREAM(pose_l.p[2]);
      ROS_INFO_STREAM("1.3 pose_ref:");
      ROS_INFO_STREAM(pose_ref.p[0]);
      ROS_INFO_STREAM(pose_ref.p[1]);
      ROS_INFO_STREAM(pose_ref.p[2]);

      // Pref_dot = Vr = - Kr * (Pref - Prd)
      // twist_ref.vel = - Kp*(pose_ref.p  - desired_position_r1);  
      twist_ref.vel.data[0] = Kp * (pose_ref.p.data[0] - desired_position_r1.data[0]);
      twist_ref.vel.data[1] = Kp * (pose_ref.p.data[1] - desired_position_r1.data[1]);
      twist_ref.vel.data[2] = Kp * (pose_ref.p.data[2] - desired_position_r1.data[2]);       

      ROS_INFO_STREAM("1.4 twist_r:");
      ROS_INFO_STREAM(twist_r.vel.data[0]);
      ROS_INFO_STREAM(twist_r.vel.data[1]);
      ROS_INFO_STREAM(twist_r.vel.data[2]);
      ROS_INFO_STREAM("1.5");
      ROS_INFO_STREAM("1.6 twist_ref:");
      ROS_INFO_STREAM(twist_ref.vel.data[0]);
      ROS_INFO_STREAM(twist_ref.vel.data[1]);
      ROS_INFO_STREAM(twist_ref.vel.data[2]);
      ROS_INFO_STREAM("1.7");

      // Compute jacobian_r, jacobian_l
      manager.getJacobian(chain_end_effector_name_r, state, jacobian_r);
      manager.getJacobian(chain_end_effector_name_l, state, jacobian_l);

      ROS_INFO_STREAM("2.0 Jacobian_r is:");
      ROS_INFO_STREAM(jacobian_r.data);
      ROS_INFO_STREAM("2.1 Jacobian_l is:");
      ROS_INFO_STREAM(jacobian_l.data);

      // KDL Manager uses Eigen, a template library for matrices with which we can invert the jacobian.
      // Extract Eigenmatrix from jacobian for easier computations(KDL::Jacobian only allows for a maximum of 6 rows which is problematic when computing transpose)
      Eigen::MatrixXd matrix_r = jacobian_r.data;
      matrix_r = matrix_r.block(0,0,6,7);
      Eigen::MatrixXd matrix_l = jacobian_l.data;
      matrix_l = matrix_l.block(0,0,6,7);

      ROS_INFO_STREAM("2.0 matrix_r is:");
      ROS_INFO_STREAM(matrix_r);
      ROS_INFO_STREAM("2.1 matrix_l is:");
      ROS_INFO_STREAM(matrix_l);
      // invert jacobian_r
      Eigen::MatrixXd matrix_r_transposed;
      matrix_r_transposed = matrix_r.transpose();
      Eigen::MatrixXd matrix_inv_r;
      matrix_inv_r = matrix_r * matrix_r_transposed;
      matrix_inv_r = matrix_inv_r.inverse();
      matrix_inv_r = matrix_r_transposed * matrix_inv_r;

      ROS_INFO_STREAM("2.2 matrix_r_transposed is:");
      ROS_INFO_STREAM(matrix_r_transposed);
      ROS_INFO_STREAM("2.3 matrix_inv_r is:");
      ROS_INFO_STREAM(matrix_inv_r);

      // concatenate matrices to create reference jacobian size 6x16
      // Jacobian_ref = [Jacobian_r, -Jacobian_l]
      Eigen::MatrixXd matrix_ref(matrix_r.rows(), matrix_r.cols()+matrix_l.cols());
      Eigen::MatrixXd matrix_l_negativ;
      matrix_l_negativ = matrix_l * -1;
      matrix_ref << matrix_r, matrix_l_negativ;
      
      ROS_INFO_STREAM("2.5 matrix_l_negativ is:");
      ROS_INFO_STREAM(matrix_l_negativ);
      ROS_INFO_STREAM("3 matrix_ref is:");
      ROS_INFO_STREAM(matrix_ref);

      // invert reference jacobian
      Eigen::MatrixXd matrix_ref_transposed;
      matrix_ref_transposed = matrix_ref.transpose();
      Eigen::MatrixXd matrix_inv_ref;
      matrix_inv_ref = matrix_ref * matrix_ref_transposed;
      matrix_inv_ref = matrix_inv_ref.inverse();
      matrix_inv_ref = matrix_ref_transposed * matrix_inv_ref;


      //q_dot_r = jacobian_inv_r * twist_r_vel
      // 8x3 * 3x1 => 8x1
      Eigen::Vector3d r_vel( twist_r.vel.data[0],  twist_r.vel.data[1],  twist_r.vel.data[2]);
      q_dot_r.data = matrix_inv_r * r_vel;

      ROS_INFO_STREAM("3.1 jmatrix_inv_r is:");
      ROS_INFO_STREAM(matrix_inv_r);
      ROS_INFO_STREAM("3.1 r_vel is:");
      ROS_INFO_STREAM(r_vel);
      ROS_INFO_STREAM("3.1 q_dot_r is:");
      ROS_INFO_STREAM(q_dot_r.data);
      
      // Calculate q_dot_ref = J+ * p_dot_ref
      // 16x3 * 3x1 => 16x1
      
      // Eigen::Vector3d ref_vel(twist_ref.vel.data[0],twist_ref.vel.data[1],twist_ref.vel.data[2]);
      
      // TEST: ref_vel = p_dot_r - p_dot_l
      Eigen::Vector3d ref_vel(twist_r.vel.data[0] - twist_l.vel.data[0], twist_r.vel.data[1] - twist_l.vel.data[1],  twist_r.vel.data[2] - twist_l.vel.data[2]);

      q_dot_ref.data = matrix_inv_ref * ref_vel; // q_dot_ref blir 16 rader lång? 
      q_dot_ref.data = q_dot_ref.data.tail(8); // vi tar de sista 8 raderna från q_dot_ref

      ROS_INFO_STREAM("3.1 matrix_inv_ref is:");
      ROS_INFO_STREAM(matrix_inv_ref);
      ROS_INFO_STREAM("3.1 ref_vel is:");
      ROS_INFO_STREAM(r_vel);
      ROS_INFO_STREAM("3.2 q_dot_ref is:");
      ROS_INFO_STREAM(q_dot_ref.data);
      ROS_INFO_STREAM("4");

      // manager.getVelIK(chain_end_effector_name_r, state, twist_r, q_dot_r);  //TWIST
      // manager.getVelIK(chain_end_effector_name_l, state, twist_l, q_dot_l);

      command_r = state;
      command_l = state;
      manager.getJointState(chain_end_effector_name_r, q_dot_r.data, command_r);
      manager.getJointState(chain_end_effector_name_l, q_dot_ref.data, command_l);

      ROS_INFO_STREAM("command_r:");  
      ROS_INFO_STREAM(command_r);
      ROS_INFO_STREAM("command_l:");  
      ROS_INFO_STREAM(command_l);
      
      // Publish velocities_r
      ROS_INFO_STREAM("Velocities_r:");
      for (int i = 0; i < 7; i++){
      publishers[i].publish(command_r.velocity[i]);
      ROS_INFO_STREAM(command_r.velocity[i]);
      }

      // Publish velocities_l
      ROS_INFO_STREAM("Velocities_l:");
      for (int i = 0; i < 7; i++){
      publishers[i+7].publish(command_l.velocity[i+7]);
      ROS_INFO_STREAM(command_l.velocity[i+7]);
      }
    }

    // this part is for calculating when its time to move to a new pose
    counter++;
    if(counter>4000){
      counter = 0;
      pose_counter++;
      ROS_INFO_STREAM(".        ~~~        .");
      ROS_INFO_STREAM(".        ~~~        .");
      ROS_INFO_STREAM(".time for a new pose.");
      ROS_INFO_STREAM(".        ~~~        .");
      ROS_INFO_STREAM(".        ~~~        .");
    }

    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  return 0;
}

