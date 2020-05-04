import sys
from nav_msgs.msg import Odometry
import numpy as np
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import rospy
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import tf
import os

cmd = "rosnode kill /ekf_localization"
os.system(cmd)

class AbsPosPub:

    def __init__(self):
        self.odom = Odometry()
        self.pub = rospy.Publisher('/odom', Odometry)
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback)
        self.br = tf.TransformBroadcaster()

    def callback(self, data):
        # data_index = [i for (i, x) in enumerate(data.name) if (x == 'jackal::base_link')]
        data_index = [i for (i, x) in enumerate(data.name) if (x == 'rbcar::base_footprint')]
        pose = data.pose[data_index[0]]
        twist = data.twist[data_index[0]]
        self.odom.pose.pose.position.x = pose.position.x
        self.odom.pose.pose.position.y = pose.position.y
        self.odom.pose.pose.orientation.x = pose.orientation.x
        self.odom.pose.pose.orientation.y = pose.orientation.y
        self.odom.pose.pose.orientation.z = pose.orientation.z
        self.odom.pose.pose.orientation.w = pose.orientation.w

        self.odom.twist.twist.linear = twist.linear
        self.odom.twist.twist.angular = twist.angular

        self.odom.header.frame_id = 'odom'
        self.odom.header.stamp = rospy.Time.now()
        self.pub.publish(self.odom)
        self.br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                              (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                              rospy.Time.now(), 'base_footprint', 'odom')

if __name__ == "__main__":

    rospy.init_node('AbsPosPub')
    a = AbsPosPub()
    rospy.spin()