#!/usr/bin/env python

# Control System node that uses angle as reference
# Node takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# Only works for left arm

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import InverseKinematics
from sensor_msgs.msg import JointState

Kp = 0.5
Ki = 0.1
I = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller

class Subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_callback)
        self.current_state = None

    def sub_callback(self, msg):
        self.current_state = msg


def run(goal_angles_r, goal_angles_l):
    # Set loop frequency
    rate = rospy.Rate(10)

    # initialize publishers
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

    # initialize subscriber
    subscriber = Subscriber()

    # LOOP
    while not rospy.is_shutdown():

        # Wait for answer from kinematic service node
        rospy.spin
        if subscriber.current_state is None:
                rate.sleep
        else:
            print("~~~NEW LOOP~~~")
            print("names are")
            print(subscriber.current_state.name)

            current_angles_r = []
            current_angles_l = []
            for i in range(7):
                current_angles_r.append(subscriber.current_state.position[i])
            for i in range(7):
                current_angles_l.append(subscriber.current_state.position[i+7])



            # Compute the new velocities
            # velocities_r = get_velocities(goal_angles_r, current_angles_r)
            velocities_l = get_velocities(goal_angles_l, current_angles_l)


            # Publish the new velocities
            for i in range(7):
                print("do nothing for now")
                # publishers[i].publish(velocities_r[i])
                # print("published " + str(velocities_r[i]) + " onto " + str(publishers[i].name))
            for i in range(7):
                publishers[i+7].publish(velocities_l[i])
                print("published " + str(velocities_l[i]) + " onto " + str(publishers[i+7].name))

            # Repeat
            rate.sleep()


def get_input_l():

    # Parse YAML data into array of Posestamps
    # Data is stored in /cth_yumi/config/..
    desired_poses = []
    for i in range(1):
        desired_poses.append(PoseStamped())
        desired_poses[i].header.frame_id = "yumi_base_link"
        desired_poses[i].header.stamp = rospy.Time.now()
        desired_poses[i].pose.position.x = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/x')
        desired_poses[i].pose.position.y = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/y')
        desired_poses[i].pose.position.z = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/z')
        desired_poses[i].pose.orientation.x = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/x')
        desired_poses[i].pose.orientation.y = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/y')
        desired_poses[i].pose.orientation.z = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/z')
        desired_poses[i].pose.orientation.w = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/w')

    return desired_poses

def get_input_r():

    # Empty for now
    desired_poses = []

    return desired_poses


def compute_ik(goal_poses):
    # call diogos I_K service node and return solution
    # service node needs to be running
    computeik = rospy.ServiceProxy('/compute_ik', InverseKinematics)
    solutions = computeik("gripper_l_base", "", goal_poses)  # this service requires two strings, dont know what the second is


    if solutions.sols[0].status.code == 0:
        goal_angles = solutions.sols[0].ik_solution
        print("IK COMPUTATION WORKED. ANSWER IS ")
        print(goal_angles)

    else:
        print("IK COMPUTATION FAILED")

    return goal_angles


def get_velocities(goal_angles, current_angles):  # PI-controller
    global I
    errors = []
    current_velocities = []
    for i in range(7):

        errors.append(goal_angles[i] - current_angles[i])
        current_velocities.append(Kp * errors[i] + I[i])

        # Update integral part
        I[i] = I[i] + Ki * errors[i]

        # Clamp velocities to max of +/-3 Rad/s
        current_velocities[i] = max(min(current_velocities[i], 3), -3)

    print("current errors are ")
    print(errors)
    return current_velocities


if __name__ == '__main__':

    rospy.init_node('control_node', anonymous=True)  # initiate node
    goal_poses_l = get_input_l()  # input is of type PoseStamped[]
    goal_poses_r = get_input_r()
    goal_angles = compute_ik(goal_poses_l, goal_poses_r)

    try:
        run(goal_angles)
    except rospy.ROSInterruptException:
        pass
