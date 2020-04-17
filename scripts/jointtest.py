#!/usr/bin/env python

# Control System node that uses angle as reference
# Node takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# THUMB RULE: RIGHT ARM FIRST, THEN LEFT ARM

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import InverseKinematics
from sensor_msgs.msg import JointState
Kp = 3.5
Ki = 0.02
I_r = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller
I_l = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller
x=0
close=20
open=-20
gripper_l_cmd_msg = close
gripper_r_cmd_msg = close


class Subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_callback)
        self.current_state = None

    def sub_callback(self, msg):
        self.current_state = msg


def run(goal_angles_r, goal_angles_l):
    # Set loop frequency
    rate = rospy.Rate(10)

    # Initialize publishers
    # Note: The order in which these are appended to the list matters! (1, 2, 7, 3, 4, 5, 6)
    publishers = list()

    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_1_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_2_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_7_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_3_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_4_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_5_r/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_6_r/command', Float64, queue_size=1))

    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_1_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_2_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_7_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_3_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_4_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_5_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_6_l/command', Float64, queue_size=1))
    # MISSING: GRIPPER PUBLISHERS
    publishers.append(rospy.Publisher("/yumi/gripper_l_effort_cmd", Float64, queue_size=1))
    publishers.append(rospy.Publisher("/yumi/gripper_r_effort_cmd", Float64, queue_size=1))

# initialize subscriber
    subscriber = Subscriber()

    # LOOP
    while not rospy.is_shutdown():
            # Wait for answer from kinematic service node
            rospy.spin
            if subscriber.current_state is None:
                    rate.sleep
            else:
                #print(goal_poses_l0)
                publishers[14].publish(open)
                publishers[15].publish(open)
                for z in range(4000): #just a test loop, would like a if statement or a while loop to test against the error.
                    print("~~~NEW LOOP~~~")
                    print("names are")
                    print(subscriber.current_state.name)
                    print("1")
                    # Separate into DX/LX
                    current_angles_r = []
                    current_angles_l = []
                    for i in range(7):
                        current_angles_r.append(subscriber.current_state.position[i])
                    for i in range(7):
                        current_angles_l.append(subscriber.current_state.position[i+7])


                    # Compute the new velocities
                    velocities_r = get_velocities_r(goal_angles_r[0], current_angles_r)
                    velocities_l = get_velocities_l(goal_angles_l[0], current_angles_l)


                    # Publish the new velocities
                    for i in range(7):
                        publishers[i].publish(velocities_r[i])
                        print("published " + str(velocities_r[i]) + " onto " + str(publishers[i].name))
                    for i in range(7):
                        publishers[i+7].publish(velocities_l[i])
                        print("published " + str(velocities_l[i]) + " onto " + str(publishers[i+7].name))

                publishers[14].publish(close)
                publishers[15].publish(close)
                for z in range(4000):
                    print("~~~NEW LOOP~~~")
                    print("names are")
                    print(subscriber.current_state.name)
                    print("2")
                    # Separate into DX/LX
                    current_angles_r = []
                    current_angles_l = []
                    for i in range(7):
                        current_angles_r.append(subscriber.current_state.position[i])
                    for i in range(7):
                        current_angles_l.append(subscriber.current_state.position[i + 7])

                    # Compute the new velocities
                    velocities_r = get_velocities_r(goal_angles_r[1], current_angles_r)
                    velocities_l = get_velocities_l(goal_angles_l[1], current_angles_l)

                    # Publish the new velocities
                    for i in range(7):
                        publishers[i].publish(velocities_r[i])
                        print("published " + str(velocities_r[i]) + " onto " + str(publishers[i].name))
                    for i in range(7):
                        publishers[i + 7].publish(velocities_l[i])
                        print("published " + str(velocities_l[i]) + " onto " + str(publishers[i + 7].name))

                # Repeat
                rate.sleep()

#Fix a nice loop that goes through all the inputs
def get_input_r():

    # Parse YAML data into array of Posestamps
    # Data is stored in /cth_yumi/config/..
    desired_poses = []
    for i in range(1,4):
        desired_poses.append(PoseStamped())
        desired_poses[i-1].header.frame_id = "yumi_base_link"
        desired_poses[i-1].header.stamp = rospy.Time.now()
        desired_poses[i-1].pose.position.x = rospy.get_param('/posestamp' + str(i) + '/pose/position/x')
        desired_poses[i-1].pose.position.y = rospy.get_param('/posestamp' + str(i) + '/pose/position/y')
        desired_poses[i-1].pose.position.z = rospy.get_param('/posestamp' + str(i) + '/pose/position/z')
        desired_poses[i-1].pose.orientation.x = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/x')
        desired_poses[i-1].pose.orientation.y = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/y')
        desired_poses[i-1].pose.orientation.z = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/z')
        desired_poses[i-1].pose.orientation.w = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/w')

    return desired_poses

def get_input_l():

    # Parse YAML data into array of Posestamps
    # Data is stored in /cth_yumi/config/..
    desired_poses = []
    for i in range(4,7):
        desired_poses.append(PoseStamped())
        desired_poses[i-4].header.frame_id = "yumi_base_link"
        desired_poses[i-4].header.stamp = rospy.Time.now()
        desired_poses[i-4].pose.position.x = rospy.get_param('/posestamp' + str(i) + '/pose/position/x')
        desired_poses[i-4].pose.position.y = rospy.get_param('/posestamp' + str(i) + '/pose/position/y')
        desired_poses[i-4].pose.position.z = rospy.get_param('/posestamp' + str(i) + '/pose/position/z')
        desired_poses[i-4].pose.orientation.x = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/x')
        desired_poses[i-4].pose.orientation.y = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/y')
        desired_poses[i-4].pose.orientation.z = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/z')
        desired_poses[i-4].pose.orientation.w = rospy.get_param('/posestamp' + str(i) + '/pose/orientation/w')

    return desired_poses



def compute_ik(goal_poses, chain_end_effector_name):

    # call diogos I_K service node and return solution
    # service node needs to be running
    computeik = rospy.ServiceProxy('/compute_ik', InverseKinematics)
    solutions = computeik(chain_end_effector_name, "", goal_poses)  # this service requires two strings, dont know what the second is

    print(solutions)
    print(len(goal_poses))
    goal_angles=list(range(len(goal_poses)))
    for i in range(len(goal_poses)):
        print(i)
        print(solutions.sols[i].ik_solution)
        if solutions.sols[i].status.code == 0: # check if all computations worked
            goal_angles[i] = solutions.sols[i].ik_solution  # currently only returns one solution
            print("IK COMPUTATION WORKED. ANSWER IS ")
            print(goal_angles)
        else:
            print(solutions)
            print("IK COMPUTATION FAILED")
            goal_angles=[0, 0 , 0]

    return goal_angles




def get_velocities_r(goal_angles, current_angles):  # PI-controller

    global I_r
    errors = []
    current_velocities = []

    for i in range(7):
        errors.append(goal_angles[i] - current_angles[i])
        current_velocities.append(Kp * errors[i] + I_r[i])

        # Update integral part
        I_r[i] = I_r[i] + Ki * errors[i]

        # Clamp velocities to max of +/-3 Rad/s
        current_velocities[i] = max(min(current_velocities[i], 3), -3)


    print("current errors for right arm are ")
    print(errors)
    return current_velocities


def get_velocities_l(goal_angles, current_angles):  # PI-controller

    global I_l
    errors = []
    current_velocities = []

    for i in range(7):
        errors.append(goal_angles[i] - current_angles[i])
        current_velocities.append(Kp * errors[i] + I_l[i])

        # Update integral part
        I_l[i] = I_l[i] + Ki * errors[i]

        # Clamp velocities to max of +/-3 Rad/s
        current_velocities[i] = max(min(current_velocities[i], 3), -3)


    print("current errors for left arm are ")
    print(errors)
    return current_velocities


if __name__ == '__main__':
    # Initiate node
    # Get inputs and compute inverse kinematics
    rospy.init_node('control_node', anonymous=True)
    goal_poses_r=get_input_r()
    goal_poses_l=get_input_l()
    goal_angle_r=compute_ik(goal_poses_r,"gripper_r_base")
    goal_angle_l=compute_ik(goal_poses_l,"gripper_l_base")

    # Run
    try:
        run(goal_angle_r, goal_angle_l)
    except rospy.ROSInterruptException:
        pass
