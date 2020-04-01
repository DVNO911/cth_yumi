#!/usr/bin/env python

# Control System node that uses position as reference
# Node takes in a position, listens to yumi for current position, regulates velocities of each encoder by computing inverse jacobian and publishing on appropriate topics
# Currently full of pseudocode but is a shell of how the final control node is going to work

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import ForwardKinematics
from robot_kinematic_services.srv import TaskSpaceKinematics
from sensor_msgs.msg import JointState

Kp = 3.5
# Ki = 0.02
# I_r = [0, 0, 0, 0, 0, 0, 0]  # integral part of controller
# I_l = [0, 0, 0, 0, 0, 0, 0]  # first three slots are x, y, z position and last 4 x, y, z, w quaternion


class Subscriber(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_callback)
        self.current_state = None

    def sub_callback(self, msg):
        self.current_state = msg


def run(goal_poses_r, goal_poses_l):
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

    # initialize subscriber
    subscriber = Subscriber()

    # LOOP
    while not rospy.is_shutdown():
        rospy.spin
        while subscriber.current_state is None: #for when node has just been started up
            rospy.spin
        # only handling SX for now
        velocities_r = compute_ts("gripper_r_base", subscriber.current_state, Kp, goal_poses_r[0].pose)

        # Publish the new velocities
        for i in range(7):
            publishers[i].publish(velocities_r[i])
            print("published " + str(velocities_r[i]) + " onto " + str(publishers[i].name))



        rate.sleep()


def get_input_r():

    # Empty for now
    desired_poses = []
    for i in range(1, 2):
        desired_poses.append(PoseStamped())
        desired_poses[i-1].header.frame_id = "yumi_base_link"
        desired_poses[i-1].header.stamp = rospy.Time.now()
        desired_poses[i-1].pose.position.x = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/x')
        desired_poses[i-1].pose.position.y = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/y')
        desired_poses[i-1].pose.position.z = rospy.get_param('/posestamp' + str(i+1) + '/pose/position/z')
        desired_poses[i-1].pose.orientation.x = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/x')
        desired_poses[i-1].pose.orientation.y = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/y')
        desired_poses[i-1].pose.orientation.z = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/z')
        desired_poses[i-1].pose.orientation.w = rospy.get_param('/posestamp' + str(i+1) + '/pose/orientation/w')

    return desired_poses


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



def compute_ts(chain_end_effector_name, current_state, kp, goal_pose):
    current_velocities = []
    computets = rospy.ServiceProxy('/compute_ts', TaskSpaceKinematics)
    solution = computets(chain_end_effector_name, "", kp, current_state, goal_pose)

    if solution.status.code == 0:
        new_state = solution.ts_solution
        print("TS COMPUTATION WORKED. ANSWER IS ")
        print(new_state)

        for i in range(7):
            current_velocities.append(new_state.velocity[i])

    return current_velocities


# maybe useless?
def compute_fk(current_state, chain_end_effector_name):

    # call diogos F_K service node, send current state and return current pose
    # service node needs to be running
    computefk = rospy.ServiceProxy('/compute_fk', ForwardKinematics)
    solution = computefk(chain_end_effector_name, "", current_state)  # this service requires two strings, dont know what the second is

    if solution.status.code == 0:  # check if all computations worked
        current_pose = solution.fk_solution  # TYPE POSESTAMPED
        print("FK COMPUTATION WORKED. ANSWER IS ")
        print(current_pose)
    else:
        print("FK COMPUTATION FAILED")

    return current_pose


if __name__ == '__main__':
    # Initiate node
    rospy.init_node('control_node', anonymous=True)

    goal_poses_r = get_input_r()
    goal_poses_l = get_input_l()
    # Run
    try:
        run(goal_poses_r, goal_poses_l)
    except rospy.ROSInterruptException:
        pass
