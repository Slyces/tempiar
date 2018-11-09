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
timer=10


def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)


def callback_lasers(data):
    global lasers
    lasers=data
    #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# replace 'my_behavior' by the name of your behavior
def my_behavior():
    global timer
    rospy.init_node('my_behavior', anonymous=True)

    # remove the subscriptions you don't need.
    #rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
   # rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
   # rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
 
    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
    pub = rospy.Publisher('/subsomption/channel2', Channel , queue_size=10)
    r = rospy.Rate(10) # 10hz

    v=Channel()

    while not rospy.is_shutdown():
        v.activated=False

        if(timer==0):
            timer=10

        if((10*random.random())<5 or timer<10):
            v.activated=True
            v.speed_left=1
            v.speed_right=1
            timer-=1
        count=0
        # compute the value of v that will be sent to the subsomption channel. 
        pub.publish(v)
        r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        my_behavior()
    except rospy.ROSInterruptException: pass
