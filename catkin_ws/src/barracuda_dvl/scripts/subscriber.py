import rospy
from waterlinked_a50_ros_driver.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped

#!/usr/bin/env python


def dvl_callback(data):
    twist_msg = TwistWithCovarianceStamped()
    twist_msg.header.stamp = data.header.stamp
    twist_msg.header.frame_id = data.header.frame_id
    
    twist_msg.twist.twist.linear.x = data.velocity.x
    twist_msg.twist.twist.linear.y = data.velocity.y
    twist_msg.twist.twist.linear.z = data.velocity.z
    
    # Assuming no covariance information is provided by the DVL
    twist_msg.twist.covariance = [0] * 36
    
    pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('topic_mapper', anonymous=True)   
    rospy.Subscriber('dvl/data', DVL, dvl_callback)
    pub = rospy.Publisher('/barracuda/dvl/twist', TwistWithCovarianceStamped, queue_size=10)
    
    rospy.spin()