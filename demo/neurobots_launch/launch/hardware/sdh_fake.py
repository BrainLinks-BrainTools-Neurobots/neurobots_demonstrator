#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('/omnirob_lbr/joint_states', JointState, queue_size=10)
    rospy.init_node('sdh_fake')
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        js = JointState()
	js.header.stamp = rospy.get_rostime()
	js.name = ['sdh2_knuckle_joint', 'sdh2_finger_12_joint', 'sdh2_finger_13_joint', 'sdh2_finger_21_joint', 'sdh2_finger_22_joint', 'sdh2_finger_23_joint', 'sdh2_thumb_2_joint', 'sdh2_thumb_3_joint']
	js.position = [0, 0, 0, 0, 0, 0, 0, 0]
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
