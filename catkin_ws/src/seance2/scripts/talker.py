#!/usr/bin/python3

# license removed for brevity

import rospy

from std_msgs.msg import String

from std_srvs.srv import Empty

isSending = 0

def handle_on_off(req):
	global isSending
	if (isSending==1):
		rospy.loginfo('stop sending')
		isSending=0
	else:
		rospy.loginfo('start sending')
		isSending=1
	return {}
def talker(): 
    global isSending

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.Service('onOff', anonymous=True, Empty.handle_on_off)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    i = 0 
    while not rospy.is_shutdown():
	if isSending==1:
        	i=i+1
        	rospy.loginfo(i)
        	pub.publish(i)
        rate.sleep()	

if __name__ == '__main__':

    try:
        talker()

    except rospy.ROSInterruptException:

        pass
