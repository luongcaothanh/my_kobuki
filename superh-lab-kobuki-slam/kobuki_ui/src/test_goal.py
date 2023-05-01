#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion
import subprocess
def newgoal(x,y,z,qx,qy,qz,qw):
    command = 'rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal "{header: {stamp: now, frame_id: \'map\'}, goal_id: {stamp: now, id: \'goal\'}, goal: {target_pose: {header: {stamp: now, frame_id: \'map\'}, pose: {position: {x: %f, y: %f, z: %f}, orientation: {x: %f, y: %f, z: %f, w: %f}}}}}"' % (x, y, z, qx, qy, qz, qw)
    subprocess.call(command, shell=True)

def send_goal():
    rospy.init_node('send_goal_node')
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = 'base_footprint'
    goal.goal.target_pose.pose.position = Point(x=-7.0, y=3.0, z=0.0)
    goal.goal.target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.986368628605, w=0.164550686733)
    pub.publish(goal)
    rospy.loginfo("Goal sent")
    rospy.spin()

if __name__ == '__main__':
    #send_goal()
    newgoal(-2.0727455616, 2.15493369102, 0.0, 0.0, 0.0, 0.468585683443, 0.883418053513)
    print("goal send")

