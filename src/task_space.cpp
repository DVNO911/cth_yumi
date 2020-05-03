#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include <generic_control_toolbox/kdl_manager.hpp>


sensor_msgs::JointState state; // The current state as recorded by the subscriber 
float Kp = 3.5; 
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
  KDL::Vector error_r, error_l;
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
  desired_position_r1.data[1] = -0.328583;
  desired_position_r1.data[2] = 0.478045;
  // desired_position_l1 = getDesiredPosition_l1();

  // desired_position_r2 = getDesiredPosition_r2();
  // desired_position_l2 = getDesiredPosition_l2();

  // desired_position_r3 = getDesiredPosition_r3();
  // desired_position_l3 = getDesiredPosition_l3();


  // LOOP
  while (ros::ok())
  {
    // Get the end-effector pose as a KDL::Frame.
    if (manager.getEefPose(chain_end_effector_name_r, state, pose_r)) // When code runs its gonna complain a couple times that it hasnt found a chain joint state, this is because the subscriber hasnt recieved a message yet
    {
      manager.getEefPose(chain_end_effector_name_l, state, pose_l);
      ROS_INFO_STREAM("getEefPose computed");

      if(pose_counter == 0){    // OLD
      error_r = desired_position_r1 - pose_r.p;
      // error_l = desired_position_l1 - pose_l.p;

      KDL::Frame pose_temp = pose_r;
      pose_temp.p.y(0);
      error_l = pose_temp.p - pose_l.p;
      }

      /*
      else if (pose_counter == 1){
      error_r = desired_position_r2 - pose_r.p;
      // error_l = desired_position_l2 - pose_l.p;
      
      KDL::Frame pose_temp = pose_r;
      pose_temp.p.y(0);
      error_l = pose_temp.p - pose_l.p;
      } 

      else{
      error_r = desired_position_r3 - pose_r.p;
      // error_l = desired_position_l3 - pose_l.p;

      KDL::Frame pose_temp = pose_r;
      pose_temp.p.y(0);
      error_l = pose_temp.p - pose_l.p;
      }
      */
      twist_r.vel = Kp*error_r; //P prick
      twist_l.vel = Kp*error_l;  // TWIST

      //EIGEN TEST
        Eigen::Matrix2d a;
         a << 1, 2,
              3, 4;
        Eigen::MatrixXd b(2,2);
         b << 2, 3,
              1, 4;
        a = a-b;
        
      //Pref = Pr - Pl
      pose_ref = pose_r;
      pose_ref.p.operator-=(pose_l.p); 
      ROS_INFO_STREAM("1");
      // Pref_prick = Vr = - Kr * (Pref - Prd)
      // twist_ref.vel = - Kp*(pose_ref.p  - desired_position_r1);  
      twist_ref.vel = pose_ref.p;                               
      twist_ref.vel.operator-=(desired_position_r1); 
      // twist_ref.vel.operator*(-Kp, twist_ref.vel);
      twist_ref.vel.data[0] *= Kp;
      twist_ref.vel.data[1] *= Kp;
      twist_ref.vel.data[2] *= Kp; //fullösning
      ROS_INFO_STREAM("1.1");
      ROS_INFO_STREAM(twist_ref.vel.data[0]);
      ROS_INFO_STREAM(twist_ref.vel.data[1]);
      ROS_INFO_STREAM(twist_ref.vel.data[2]);
      ROS_INFO_STREAM("1.2");
      // PUBLISH ERRORS
      /*
      ROS_INFO_THROTTLE(0.1,
                        "Position error_r: (%.4f, %.4f, %.4f)",
                        error_r.x(), error_r.y(), error_r.z());

      ROS_INFO_THROTTLE(0.1,
                        "Position error_l: (%.4f, %.4f, %.4f)",
                        error_l.x(), error_l.y(), error_l.z());
      */
      ROS_INFO_STREAM("2");
      manager.getJacobian(chain_end_effector_name_r, state, jacobian_r);
      manager.getJacobian(chain_end_effector_name_l, state, jacobian_l);
      ROS_INFO_STREAM("2.1 Jacobian_r is:");
      ROS_INFO_STREAM(jacobian_r.data);
      ROS_INFO_STREAM("2.1 Jacobian_l is:");
      ROS_INFO_STREAM(jacobian_l.data);
      // Extract Eigenmatrix from jacobian for easier computations(KDL::Jacobian only allows for a maximum of 6 rows which is problematic when computing transpose)
      Eigen::MatrixXd matrix_r = jacobian_r.data;
      Eigen::MatrixXd matrix_l = jacobian_l.data;
      // KDL Manager uses Eigen, a template library for matrices with which we can invert the jacobian.
      // invert jacobian_r

      Eigen::MatrixXd matrix_r_transposed(jacobian_r.data.cols(), jacobian_r.data.rows());
      matrix_r_transposed = matrix_r.transpose();
      ROS_INFO_STREAM("2.2 matrix_r_transposed is:");
      ROS_INFO_STREAM(matrix_r_transposed);
      Eigen::MatrixXd matrix_inv_r;
      matrix_inv_r = matrix_r * matrix_r_transposed;
      matrix_inv_r.inverse();
      matrix_inv_r = matrix_r_transposed * matrix_inv_r;
      ROS_INFO_STREAM("2.3 matrix_inv_r is:");
      ROS_INFO_STREAM(matrix_inv_r);
      
      /*
      Eigen::MatrixXd jacobian_r_transposed(jacobian_r.data.cols(), jacobian_r.data.rows()); //cant be of type jacobian since max 6 rows allowed 
      jacobian_r_transposed = jacobian_r.data.transpose();
      KDL::Jacobian jacobian_inv_r;
      ROS_INFO_STREAM("2.2 jacobian_r_transposed is:");
      ROS_INFO_STREAM(jacobian_r_transposed);
      jacobian_inv_r.data = jacobian_r.data*jacobian_r_transposed;
      jacobian_inv_r.data = jacobian_inv_r.data.inverse();
      jacobian_inv_r.data = jacobian_r_transposed * jacobian_inv_r.data;
      ROS_INFO_STREAM("2.3 jacobian_inv_r is:");
      ROS_INFO_STREAM(jacobian_inv_r.data);
      */

      // concatenate matrices to create reference jacobian size 3x14 
      // Jacobian_ref = [Jacobian_r -Jacobian_l]
      Eigen::MatrixXd matrix_ref(matrix_r.rows(), matrix_r.cols()+matrix_l.cols());
      Eigen::MatrixXd matrix_l_negativ;
      matrix_l_negativ = matrix_l * -1;
      ROS_INFO_STREAM("2.5 matrix_l_negativ is:");
      ROS_INFO_STREAM(matrix_l_negativ);
      // Eigen::MatrixXd temp(jacobian_r.data.rows(), jacobian_r.data.cols()+jacobian_l.data.cols());
      // jacobian_ref.data = temp;
      // jacobian_ref.data << jacobian_r.data, jacobian_l_negativ.data;
      matrix_ref << matrix_r, matrix_l_negativ;
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
      // I assume that pseudoinverse of 3x7 matrix is 7x3?
      // 7x3 * 3x1 => 7x1
      // make 3x1 vector into Eigenmatrix to make multiplication, then back to vector?
      //NOTE! Q_DOT CONTAINS VECTOR TYPE EIGEN CALLED DATA!
      Eigen::Vector3d r_vel(twist_r.vel.data[0],twist_r.vel.data[1],twist_r.vel.data[2]);
      q_dot_r.data = matrix_inv_r * r_vel;  //we are now giving it a matrix but it wants a vector
      // q_dot_r.data = VectorXd q_dot_r.data()
      ROS_INFO_STREAM("3.1 jmatrix_inv_r is:");
      ROS_INFO_STREAM(matrix_inv_r);
      ROS_INFO_STREAM("3.1 r_vel is:");
      ROS_INFO_STREAM(r_vel);
      ROS_INFO_STREAM("3.1 q_dot_r is:");
      ROS_INFO_STREAM(q_dot_r.data);
      // I assume that pseudoinverse of 3x16 matrix is 16x3?
      // 16x3 * 3x1 => 16x1
      Eigen::Vector3d ref_vel(twist_ref.vel.data[0],twist_ref.vel.data[1],twist_ref.vel.data[2]);
      q_dot_ref.data = matrix_inv_ref * ref_vel; // qprick blir 16 rader lång? 
      q_dot_ref.data = q_dot_ref.data.tail(8);
      ROS_INFO_STREAM("3.1 matrix_inv_ref is:");
      ROS_INFO_STREAM(matrix_inv_ref);
      ROS_INFO_STREAM("3.1 ref_vel is:");
      ROS_INFO_STREAM(r_vel);
      ROS_INFO_STREAM("3.2 q_dot_ref is:");
      ROS_INFO_STREAM(q_dot_ref.data);
      ROS_INFO_STREAM("4");
      // manager.getVelIK(chain_end_effector_name_r, state, twist_r, q_dot_r);  //TWIST
      //manager.getVelIK(chain_end_effector_name_l, state, twist_l, q_dot_l);

      command_r = state;
      command_l = state;

      manager.getJointState(chain_end_effector_name_r, q_dot_r.data, command_r);
      manager.getJointState(chain_end_effector_name_l, q_dot_ref.data, command_l);

  
      // Publish velocities_r
      for (int i = 0; i < 7; i++){
      publishers[i].publish(command_r.velocity[i]);
      }

      // Publish velocities_l
      for (int i = 0; i < 7; i++){
      publishers[i+7].publish(command_l.velocity[i+7]);
      }
    }

    // this part is for calculating when its time to move to a new pose
    counter++;
    if(counter>400){
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

