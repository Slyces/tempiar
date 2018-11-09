#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16,Float32,Bool,Float32MultiArray
from subsomption.msg import Channel
import sys

channel=[]

#-------------------------------------------
class CallBack_module_cl(object):

    def __init__(self, num):
        print "Creating callback for "+str(num)
        self.num = num

    def __call__(self, data):
        return callback_module(self.num,data)


#-------------------------------------------
def callback_module(n, data):
    channel[n]=data
    #rospy.loginfo(rospy.get_caller_id()+" n=%d Activated: %d speed_l: %f speed_r: %f",n,data.activated, data.speed_left, data.speed_right)


#-------------------------------------------
# nb is the number of behavioral modules in competition
#-------------------------------------------

def subsomption_architecture(nb):
    rospy.init_node('subsomption_architecture', anonymous=True)
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0

    # Le noeud publie des ordres de deplacement a destination de simu_fastsim :
    pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
    pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)

    for n in range(nb):
        rospy.Subscriber("/subsomption/channel"+str(n), Channel, CallBack_module_cl(n))

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        speed_l=0
        speed_r=0
        for i in range(nb-1,-1,-1): 
            if (channel[i].activated):
                rospy.loginfo(rospy.get_caller_id()+" module actif: %d",i)
                speed_l=channel[i].speed_left
                speed_r=channel[i].speed_right
                break

        for i in range(nb): 
            channel[i]=v
        pub_l.publish(speed_l)
        pub_r.publish(speed_r)
        r.sleep()   


if __name__ == '__main__':

    nbch=int(sys.argv[1])
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0
    channel=[v for i in range(nbch)]
    try:
        subsomption_architecture(nbch)
    except rospy.ROSInterruptException: pass
