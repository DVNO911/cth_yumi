#!/usr/bin/env python

# Control System node that uses angle as reference
# Node takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# Currently full of pseudocode but is a shell of how the final control node is going to work

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from robot_kinematic_services.srv import InverseKinematics

#attempt at using class for subscriber
class Subscriber(object):
    def __init__(self):
        self.pub = rospy.Subscriber('yumi', String, self.sub_callback)
        self.current_angles = ()  #dont know yet what the message looks like

    def sub_callback(self, msg):
        self.current_angles = msg




def run(goal_angles):
    pub1 = rospy.Publisher('/yumi/joint_vel_controller_1_r/command', Float64, queue_size=1)  # initiate publishers, incomplete
    pub2 = rospy.Publisher('/yumi/joint_vel_controller_2_r/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/yumi/joint_vel_controller_3_r/command', Float64, queue_size=1)
    pub4 = rospy.Publisher('/yumi/joint_vel_controller_4_r/command', Float64, queue_size=1)
    pub5 = rospy.Publisher('/yumi/joint_vel_controller_5_r/command', Float64, queue_size=1)
    pub6 = rospy.Publisher('/yumi/joint_vel_controller_6_r/command', Float64, queue_size=1)
    pub7 = rospy.Publisher('/yumi/joint_vel_controller_7_r/command', Float64, queue_size=1)
    subscriber = Subscriber()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rospy.spin
        while subscriber.current_angles is None: #for when node has just been started up
            rospy.spin
        current_velocities = get_current_velocities(goal_angles, subscriber.current_angles)


        pub1.publish(current_velocities(0)) #rough example of how publications are gonnawork
        pub2.publish(current_velocities(1))
        # rospy.loginfo(current_velocities)

        rate.sleep()

#def callback(data): #obsolete?
#   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



def get_input():
    # for now generates an empty pose, adds it to an array(because compute_ik requires a PoseStamped[] array) and returns it
    # how should it take in inputs? parameter in launch file?
    p = PoseStamped()
    p.header.frame_id = ""
    p.header.stamp = rospy.Time.now()
    print(p)
    desired_poses = [p]
    return desired_poses


def compute_ik(goal_poses):
    # call diogos I_K service node and return solution
    # assume this node is launched
    computeik = rospy.ServiceProxy('/compute_ik', InverseKinematics)
    goal_angles = computeik("a", "b", goal_poses)  #this service requires two strings
    return goal_angles


def get_current_velocities(goal_angles, current_angles):  #P-controller, not implemented, returns array of velocities
    current_velocities = ()
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
