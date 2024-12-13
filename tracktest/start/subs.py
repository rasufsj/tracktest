#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    x,y,z = data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z
    print(x,y,z)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/uav1/control_manager/control_reference", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()