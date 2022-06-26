#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
 
class OdomEKF():
   def __init__(self):
       self.uwb_x=0.0
       self.uwb_y=0.0
       # Give the node a name
       rospy.init_node('odom_ekf', anonymous=False)
 
       # Publisher of type nav_msgs/Odometry
       self.ekf_pub = rospy.Publisher('odom_ekf', Odometry, queue_size=10)
       
       # Wait for the /odom_combined topic to become available
       rospy.wait_for_message('odom_ekf_output', PoseWithCovarianceStamped)
       
       # Subscribe to the /odom_combined topic
       rospy.Subscriber('odom_ekf_output', PoseWithCovarianceStamped, self.pub_ekf_odom)
       #rospy.Subscriber('odom', Odometry, self.pub_ekf_odom1111)
       rospy.loginfo("Publishing combined odometry on /odom_ekf")
       

   def pub_ekf_odom1111(self,msg):
       self.uwb_x = msg.pose.pose.position.x
       self.uwb_y = msg.pose.pose.position.y
   def pub_ekf_odom(self, msg):
       odom = Odometry()
       odom.header = msg.header
       odom.header.frame_id = '/odom_combined'
       odom.child_frame_id = 'base_footprint'
       odom.pose = msg.pose
       #odom.pose.pose.position.x=self.uwb_x
       #odom.pose.pose.position.y=self.uwb_y
       
       self.ekf_pub.publish(odom)
       
if __name__ == '__main__':
   try:
       OdomEKF()
       rospy.spin()
   except:
       pass