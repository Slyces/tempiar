#!/usr/bin/env python

import rospy
from subsomption.msg import Channel 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import random

bumper_l=0
bumper_r=0
lasers=LaserScan()
speed=2
threshold=50
timer=30
bumped=False

def callback_right_bumper(data):
	global bumper_r,bumped
	bumper_r=data.data
	if (bumper_r==1 and not bumped):
		bumped=True
	#rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
	global bumper_l
	bumper_l=data.data
	global bumped
	if(bumper_l==1 and not bumped):
		bumped=True
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)


def callback_lasers(data):
    global lasers
    lasers=data
    #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# replace 'my_behavior' by the name of your behavior
def my_behavior():
	global timer
	global bumped
	
	rospy.init_node('my_behavior', anonymous=True)
    # remove the subscriptions you don't need.
	#rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
	rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
	rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
 	
    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
	pub = rospy.Publisher('/subsomption/channel3', Channel , queue_size=10)
	r = rospy.Rate(10) # 10hz
	v=Channel()
	while not rospy.is_shutdown():
		
		if (timer==0):
			v.activated=False	
			timer=50
			bumped=False
		else:
			if (bumped):
				v.activated=True
				if (timer<20):
					v.speed_left=0
					v.speed_right=1
					rospy.loginfo(" ROTATE")
				else:
					v.speed_left=-1
					v.speed_right=-1
				
					rospy.loginfo(" BACK")
				timer=timer-1
		count=0
	
		# compute the value of v that will be sent to the subsomption channel. 
		pub.publish(v)
		r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        my_behavior()
    except rospy.ROSInterruptException: pass
