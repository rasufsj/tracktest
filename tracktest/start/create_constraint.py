import rospy
import numpy as np

from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference
from nav_msgs.msg import Odometry

class Goto:
    def __init__(self):
        self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)
        rospy.loginfo('Waiting for path service...')
        rospy.wait_for_service('/uav1/control_manager/landoff_tracker/land')
        rospy.loginfo('Path service available')

        self.path_msg = PathSrvRequest()
        self.path_msg.path.header.frame_id = ""
        self.path_msg.path.use_heading = True
        self.path_msg.path.fly_now = True
        self.path_msg.path.max_execution_time = 5.0
        self.path_msg.path.max_deviation_from_path = 0.0
        self.path_msg.path.dont_prepend_current_state = False

        self.subscriber = rospy.Subscriber("/uav1/control_manager/control_reference", Odometry, self.callback)
        self.callback_executed = False

    def callback(self, data):
        if not self.callback_executed:
            x, y, z = data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z
            for i in range(0, 15):
                point = Reference()
                point.position.x = i * 0.2 + x
                point.position.y = i * 0.2 + np.cos(i * np.pi * 0.1) + y
                point.position.z = np.sin(i * np.pi * 0.1) + z
                point.heading = 0.78
                self.path_msg.path.points.append(point)
            try:
                response = self.sc_path.call(self.path_msg)
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)
            self.callback_executed = True
            self.subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('goto', anonymous=True)
    goto = Goto()
    rospy.spin()  # Keep the node running until shutdown
