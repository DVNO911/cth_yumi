#!/usr/bin/env python

# Control System node that uses angle as reference
# Node takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# Only works for left arm

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import InverseKinematics
from sensor_msgs.msg import JointState

Kp = 3.5
Ki = 0.02
I = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller

class Subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_callback)
        self.current_state = None

    def sub_callback(self, msg):
        self.current_state = msg


def run(goal_angles):
    # Set loop frequency
    rate = rospy.Rate(10)

    # initialize publishers
    publishers = list()
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_1_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_2_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_7_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_3_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_4_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_5_l/command', Float64, queue_size=1))
    publishers.append(rospy.Publisher('/yumi/joint_vel_controller_6_l/command', Float64, queue_size=1))

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
            # Set current angles(this is a mess for now)
            current_angles = subscriber.current_state.position
            print(subscriber.current_state.name)
            current_angles_names = subscriber.current_state.name
            current_angles2 = []
            current_angles_names2 = []
            for i in range (7): # Everything needs to be shifter 7 steps to access left controllers(temporary solution).
                current_angles2.append(current_angles[i+7])
                current_angles_names2.append(current_angles_names[i+7])
            print("current angles are ")
            print(current_angles_names2)
            print(current_angles2)

            # Compute the new velocities
            velocities = get_velocities(goal_angles, current_angles2)

            # Publish the new velocities
            for i in range(7):
                publishers[i].publish(velocities[i])
                print("published " + str(velocities[i]) + " onto " + str(publishers[i].name))

            # Repeat
            rate.sleep()


def get_input():
    # for now generates an empty pose, adds it to an array(because compute_ik requires a PoseStamped[] array) and returns it
    # how should it take in inputs? parameter in launch file?
    p = PoseStamped()
    p.header.frame_id = "yumi_base_link"  # important for ik computation
    p.header.stamp = rospy.Time.now() # currently not working?
    p.pose.position.x = 0.288638  # this placeholder pose is necessary because we need to send in a valid pose
    p.pose.position.y = 0.328583
    p.pose.position.z = 0.478045
    p.pose.orientation.x = 0.715402
    p.pose.orientation.y = -0.201775
    p.pose.orientation.z = 0.634136
    p.pose.orientation.w = -0.212975

    print(p)
    desired_poses = [p]
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

        # Clamp velocities to max of 3 Rad/s
        current_velocities[i] = max(min(current_velocities[i], 3), -3)

    print("current errors are ")
    print(errors)
    return current_velocities


if __name__ == '__main__':

    # this should be looped so that code runs from here when new input is detected
    rospy.init_node('control_node', anonymous=True)  # initiate node
    goal_poses = get_input()  # input is of type PoseStamped[]
    goal_angles = compute_ik(goal_poses)

    try:
        run(goal_angles)
    except rospy.ROSInterruptException:
        pass
