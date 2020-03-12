#!/usr/bin/env python

# Control System node that uses angle as reference
# Node takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# Currently full of pseudocode but is a shell of how the final control node is going to work

# Import python control library

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import InverseKinematics
# from simple_pid import PID
# import control as cl

Kp = 2
Ki = 0.5
I = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller


# attempt at using class for subscriber
class Subscriber(object):
    def __init__(self):
        self.pub = rospy.Subscriber('yumi', String, self.sub_callback)
        self.current_angles = ()  # dont know yet what the message looks like

    def sub_callback(self, msg):
        self.current_angles = msg


def run(goal_angles):
    # Set loop frequency
    rate = rospy.Rate(10)

    # initialize publishers
    publishers = list()
    for i in range(7):
        publishers.append(rospy.Publisher('/yumi/joint_vel_controller_' + str(i+1) + '_r/command', Float64, queue_size=1))

    # initialize subscriber
    subscriber = Subscriber()
    #SUBSCRIBER NEEDS TO TAKE IN ANGLES HERE



    # LOOP
    while not rospy.is_shutdown():
        rospy.spin
        if subscriber.current_angles is None:  # for when node has just been started up
                rate.sleep
        else:
            # Compute the new velocities
            temp_angles = (1, 2, 1, 0, 1.4, 0.3, 0.2)
            velocities = get_velocities(goal_angles, temp_angles)
            #velocities = get_velocities(goal_angles, subscriber.current_angles)

            # Publish the new velocities
            for i in range(7):
                publishers[i].publish(velocities[i])
                print("published " + str(velocities[i]) + " onto " + str(publishers[i]))

            # Update the current velocities by listening to sensors
            #subscriber.sub_callback()

            # Repeat
            rate.sleep()


def get_input():
    # for now generates an empty pose, adds it to an array(because compute_ik requires a PoseStamped[] array) and returns it
    # how should it take in inputs? parameter in launch file?
    p = PoseStamped()
    p.header.frame_id = "yumi_base_link"  # important for ik computation
    p.header.stamp = rospy.Time.now() # currently not working?
    p.pose.position.x = 0.388638  # this placeholder pose is necessary because we need to send in a valid pose
    p.pose.position.y = 0.328583
    p.pose.position.z = 0.278045
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
    print("here")
    print(solutions)
    print("then here")
    print(solutions.sols[0].status.code)

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
        print(len(current_angles))
        print(" i is " + str(i))
        print(len(goal_angles))
        errors.append(current_angles[i] - goal_angles[i])
        current_velocities.append(Kp * errors[i] + I[i])

        # Update integral part
        I[i] = I[i] + Ki * errors[i]
        # MISSING: CLAMPING VELOCITIES

    print(current_velocities)
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
