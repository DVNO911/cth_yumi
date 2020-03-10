#!/usr/bin/env python

# Node that takes in a position, calls to calculate i_k, regulates velocities of each encoder by publishing onto the appropriate topics
# Currently full of pseudocode but is a shell of how the final control node is going to work

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

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
    rospy.init_node('control_node', anonymous=True)  # initiate node
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

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



def get_input():
    # for now generates an empty pose and returns it
    # maybe should take in a pose and apply a header onto it here
    pose = PoseStamped()
    return pose


def compute_ik(goal_pose):  # needs 2x strings?
    # for now takes in a PoseStamp and returns array of float64(?)
    # in future will create message that will be sent to diogos I_K service node and return solution
    # assume this node is launched
    return goal_angles


def get_current_velocities(goal_angles, current_angles): #P-controller, not implemented, returns array of velocities
    current_velocities = ()
    return current_velocities

if __name__ == '__main__':
    # this should be looped so that code runs from here when new input is detected
    goal_pose = get_input()  # input is of type PoseStamped
    goal_angles = compute_ik(goal_pose)

    # from here not really clear how node should listen and publish at the same time
    try:
        run(goal_angles)
    except rospy.ROSInterruptException:
        pass
    # need a listener/talker loop?
    # listen to current angles, use goal_angles to calculate new velocities, publish velocities, repeat
