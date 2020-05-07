#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include <generic_control_toolbox/kdl_manager.hpp>


sensor_msgs::JointState state; // The current state as recorded by the subscriber 
float Kp = 1; 
float Ko = 1;
std::string chain_end_effector_name_r = "gripper_r_finger_r"; // Kept as global variables because it might be useful to use other frames as reference
std::string chain_end_effector_name_l = "gripper_l_finger_r";


int counter = 0;
int pose_counter = 0;

// Subscriber 
void stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
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
  KDL::Vector vel_error_r, vel_error_l, rot_error_r, rot_error_ref; //vel_error_ref missing?
  KDL::JntArray q_dot_r(7), q_dot_l(7), q_dot_ref(7);
  KDL::Twist twist_r = KDL::Twist::Zero();
  KDL::Twist twist_l = KDL::Twist::Zero();
  KDL::Twist twist_ref = KDL::Twist::Zero();
  sensor_msgs::JointState command_r, command_l;

  KDL::Jacobian jacobian_r = KDL::Jacobian();
  KDL::Jacobian jacobian_l = KDL::Jacobian();


  // Parse goal poses
  KDL::Vector desired_position_r1;
  desired_position_r1.data[0] = 0.488638;
  desired_position_r1.data[1] = -0.128583;
  desired_position_r1.data[2] = 0.178045;


  // Construct an object of type KDL::Rotation that can be given to object KDL::Frame
  // This way you can input euler angles instead of quaternions
  KDL::Rotation desired_rot_r1;
  KDL::Rotation desired_rot_l1;
  //desired_rot_r1 = KDL::Rotation::EulerZYX(0.2, 0.2, 0.2); 
  desired_rot_r1 = KDL::Rotation::Quaternion(-0.5, 0.5, -0.5, 0.5); 
  desired_rot_l1 = KDL::Rotation::Quaternion(-0.5, 0.5, -0.5, 0.5); 
  desired_rot_r1 = KDL::Rotation::Quaternion(0.707, 0, 0.707, 0); 

  desired_rot_l1 = KDL::Rotation::Quaternion(0.707, 0, 0.707, 0); 
  //desired_rot_r1 = KDL::Rotation::Quaternion(0.7071068 , 0, 0, 0.7071068 ); 
  
  //desired_rot_r1 = KDL::Rotation::Quaternion(0.715402, -0.201775, 0.634136, -0.212975); 
  //desired_rot_r1 = KDL::Rotation::Quaternion(-0.831,  0.363,  0.384, -0.173); FUNKAR EJ
  //desired_rot_r1 = KDL::Rotation::Quaternion( 0.454,  0.203,  0.354, -0.792); FUNKAR EJ
  //desired_rot_r1 = KDL::Rotation::Quaternion(-0.5, -0.846,  0.129,  0.129); FUNKAR EJ
  //desired_rot_r1 = KDL::Rotation::Quaternion(-0.14052, 0.74946, -0.15362, 0.62846); 

  // LOOP
  while (ros::ok())
  {

      ROS_INFO_STREAM("~ ~ ~ NEW LOOP ~ ~ ~");
    // Get the end-effector pose as a KDL::Frame.
    // When code runs its gonna complain a couple times that it hasnt found a chain joint state, this is because the subscriber hasnt recieved a message yet
    if (manager.getEefPose(chain_end_effector_name_r, state, pose_r) &&  manager.getEefPose(chain_end_effector_name_l, state, pose_l)) 
    {
      ROS_INFO_STREAM("getEefPose computed");

      // Calculate Errors
      vel_error_r = pose_r.p - desired_position_r1 ;
      vel_error_l = pose_l.p - desired_position_r1 ;


      double epsilond_x_r, epsilond_y_r, epsilond_z_r, etad_r; //destination_r
      double epsilond_x_l, epsilond_y_l, epsilond_z_l, etad_l; //destination_l
      double epsilone_x_r, epsilone_y_r, epsilone_z_r, etae_r; //current_r
      double epsilone_x_l, epsilone_y_l, epsilone_z_l, etae_l; //current_l

      desired_rot_r1.GetQuaternion(etad_r, epsilond_x_r, epsilond_y_r, epsilond_z_r); 
      desired_rot_l1.GetQuaternion(etad_l, epsilond_x_l, epsilond_y_l, epsilond_z_l); 
      pose_r.M.GetQuaternion(etae_r, epsilone_x_r, epsilone_y_r, epsilone_z_r);
      pose_l.M.GetQuaternion(etae_l, epsilone_x_l, epsilone_y_l, epsilone_z_l);

      // rot_error_r[0] = etae_r * epsilond_x_r - etad_r * epsilone_x_r - epsilond_x_r * epsilone_x_r;
      //rot_error_r[1] = etae_r * epsilond_y_r - etad_r * epsilone_y_r - epsilond_y_r * epsilone_y_r;
      // rot_error_r[2] = etae_r * epsilond_z_r - etad_r * epsilone_z_r - epsilond_z_r * epsilone_z_r;

      //nytt försök där sista termen är kryssprodukt
      rot_error_r[0] = etae_r * epsilond_x_r - etad_r * epsilone_x_r - (epsilond_y_r * epsilone_z_r - epsilond_z_r * epsilone_y_r);
      rot_error_r[1] = etae_r * epsilond_y_r - etad_r * epsilone_y_r - -1 * (epsilond_x_r * epsilone_z_r - epsilond_z_r * epsilone_x_r);
      rot_error_r[2] = etae_r * epsilond_z_r - etad_r * epsilone_z_r - (epsilond_x_r * epsilone_y_r - epsilond_y_r * epsilone_x_r);

      // SLAVE ORIENTATION
      rot_error_ref[0] = etae_l * epsilone_x_r - etae_r * epsilone_x_l - (epsilone_y_r * epsilone_z_l - epsilone_z_r * epsilone_y_l);
      rot_error_ref[1] = etae_l * epsilone_y_r - etae_r * epsilone_y_l - -1 * (epsilone_x_r * epsilone_z_l - epsilone_z_r * epsilone_x_l);
      rot_error_ref[2] = etae_l * epsilone_z_r - etae_r * epsilone_z_l - (epsilone_x_r * epsilone_y_l - epsilone_y_r * epsilone_x_l);

      // slave orientation funkar piss, vi testar vanlig på left arm
      // rot_error_ref[0] = etae_l * epsilond_x_l - etad_l * epsilone_x_l - (epsilond_y_l * epsilone_z_l - epsilond_z_l * epsilone_y_l);
      // rot_error_ref[1] = etae_l * epsilond_y_l - etad_l * epsilone_y_l - -1 * (epsilond_x_l * epsilone_z_l - epsilond_z_l* epsilone_x_l);
      // rot_error_ref[2] = etae_l * epsilond_z_l - etad_l * epsilone_z_l - (epsilond_x_l * epsilone_y_l - epsilond_y_l * epsilone_x_l);

      ROS_INFO_STREAM("\nrot_error_r:");
      ROS_INFO_STREAM(rot_error_r[0]);
      ROS_INFO_STREAM(rot_error_r[1]);
      ROS_INFO_STREAM(rot_error_r[2]);
      ROS_INFO_STREAM("rot_error_ref:");
      ROS_INFO_STREAM(rot_error_ref[0]);
      ROS_INFO_STREAM(rot_error_ref[1]);
      ROS_INFO_STREAM(rot_error_ref[2]);

      twist_r.vel = - Kp*vel_error_r; // P dot
      twist_l.vel = - Kp*vel_error_l; // Not used anymore

      twist_r.rot = - Ko*rot_error_r; 

      
      // twist_l.rot = - Ko*rot_error_l;



      // Pref = Pr - Pl
      pose_ref = pose_r;
      pose_ref.p.operator-=(pose_l.p); 

      ROS_INFO_STREAM("\n1 pose_r:");
      ROS_INFO_STREAM(pose_r.p[0]);
      ROS_INFO_STREAM(pose_r.p[1]);
      ROS_INFO_STREAM(pose_r.p[2]);
      ROS_INFO_STREAM("\n1.2 pose_l:");
      ROS_INFO_STREAM(pose_l.p[0]);
      ROS_INFO_STREAM(pose_l.p[1]);
      ROS_INFO_STREAM(pose_l.p[2]);
      ROS_INFO_STREAM("\n1.3 pose_ref:");
      ROS_INFO_STREAM(pose_ref.p[0]);
      ROS_INFO_STREAM(pose_ref.p[1]);
      ROS_INFO_STREAM(pose_ref.p[2]);

      // Pref_dot = Vr = - Kr * (Pref - Prd)
      // twist_ref.vel = - Kp*(pose_ref.p  - desired_position_r1);  
      twist_ref.vel.data[0] = - Kp * (pose_ref.p.data[0]);
      twist_ref.vel.data[1] = - Kp * (pose_ref.p.data[1] + 0.2); // +0.2 offset
      twist_ref.vel.data[2] = - Kp * (pose_ref.p.data[2]);    
      twist_ref.rot = - Ko*rot_error_ref; 
      //twist_ref.rot.data[0] = - Ko * (rot_error_ref[0]);
      //twist_ref.rot.data[1] = - Ko * (rot_error_ref[1]);
      //twist_ref.rot.data[2] = - Ko * (rot_error_ref[2]);   

      ROS_INFO_STREAM("\n1.4 twist_r:");
      ROS_INFO_STREAM(twist_r.vel.data[0]);
      ROS_INFO_STREAM(twist_r.vel.data[1]);
      ROS_INFO_STREAM(twist_r.vel.data[2]);
      ROS_INFO_STREAM(twist_r.rot.data[0]);
      ROS_INFO_STREAM(twist_r.rot.data[1]);
      ROS_INFO_STREAM(twist_r.rot.data[2]);
      ROS_INFO_STREAM("\n1.5\n");
      ROS_INFO_STREAM("1.6 twist_ref:");
      ROS_INFO_STREAM(twist_ref.vel.data[0]);
      ROS_INFO_STREAM(twist_ref.vel.data[1]);
      ROS_INFO_STREAM(twist_ref.vel.data[2]);
      ROS_INFO_STREAM(twist_ref.rot.data[0]);
      ROS_INFO_STREAM(twist_ref.rot.data[1]);
      ROS_INFO_STREAM(twist_ref.rot.data[2]);
      ROS_INFO_STREAM("\n1.7 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

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
      Eigen::MatrixXd matrix_l = jacobian_l.data;

      //Yiannis ville att sista kolumnen skulle vara nollställd ty interferens
      for(int i = 0 ; i<7; i++){
        matrix_r(i,7) = 0;
        matrix_l(i,7) = 0;
      }

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
      // 8x6 * 6x1 => 8x1
      Eigen::VectorXd r_x(6);
      r_x[0] = twist_r.vel.data[0];
      r_x[1] = twist_r.vel.data[1];
      r_x[2] = twist_r.vel.data[2];
      r_x[3] = twist_r.rot.data[0];
      r_x[4] = twist_r.rot.data[1];
      r_x[5] = twist_r.rot.data[2];
     
      q_dot_r.data = matrix_inv_r * r_x;

      ROS_INFO_STREAM("3.1 jmatrix_inv_r is:");
      ROS_INFO_STREAM(matrix_inv_r);
      ROS_INFO_STREAM("3.1 r_ is:");
      ROS_INFO_STREAM(r_x);
      ROS_INFO_STREAM("3.1 q_dot_r is:");
      ROS_INFO_STREAM(q_dot_r.data);
      
      // Calculate q_dot_ref = J+ * p_dot_ref
      // 16x6 * 6x1 => 16x1
      Eigen::VectorXd ref_x(6);
      ref_x[0] = twist_ref.vel.data[0];
      ref_x[1] = twist_ref.vel.data[1];
      ref_x[2] = twist_ref.vel.data[2];
      ref_x[3] = twist_ref.rot.data[0];
      ref_x[4] = twist_ref.rot.data[1];
      ref_x[5] = twist_ref.rot.data[2];
      //ref_x[3] = 0;
      //ref_x[4] = 0;
      //ref_x[5] = 0;
      q_dot_ref.data = matrix_inv_ref * ref_x; // q_dot_ref blir 16 rader lång? 
      q_dot_ref.data = q_dot_ref.data.tail(8); // vi tar de sista 8 raderna från q_dot_ref

      ROS_INFO_STREAM("3.1 matrix_inv_ref is:");
      ROS_INFO_STREAM(matrix_inv_ref);
      ROS_INFO_STREAM("3.1 ref_x is:");
      ROS_INFO_STREAM(ref_x);
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

