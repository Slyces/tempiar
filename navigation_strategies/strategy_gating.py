#!/usr/bin/env python

import rospy
import math
import random #used for the random choice of a strategy
from std_msgs.msg import Int16,Float32,Bool,Float32MultiArray,Int16MultiArray
from nav_msgs.msg import Odometry
from fastsim.srv import *
from subsomption.msg import Channel
from sensor_msgs.msg import LaserScan
import sys
import numpy as np

channel=[]
lasers=LaserScan()
radar = []
odom = Odometry()
bumper_l=0
bumper_r=0
goalx = 300
goaly = 480

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
def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

#-------------------------------------------
def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)

#-------------------------------------------
def callback_lasers(data):
  global lasers
  lasers=data

#-------------------------------------------
def callback_radar(data):
  global radar
  radar=data.data

#-------------------------------------------
def callback_odom(data):
  global odom
  odom=data

# ajouter un call back radar avec le bon type de donnees

#-------------------------------------------
# nbCh is the number of behavioral modules (channels) in competition
# gatingType sets the gating algorithm to be used ['random','qlearning']
#-------------------------------------------

def strategy_gating(nbCh,gatingType):
    rospy.init_node('strategy_gating', anonymous=True)
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0

    # Parameters of State building
    th_neglectedWall = 35

    angleLMin = 0
    angleLMax = 55

    angleFMin=56
    angleFMax=143

    angleRMin=144
    angleRMax=199

    # The node publishes movement orders for simu_fastsim :
    pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
    pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)

    # The node receives order suggestions by the behavioral modules (channels):
    for n in range(nbCh):
        rospy.Subscriber("/navigation_strategies/channel"+str(n), Channel, CallBack_module_cl(n))

    # If necessary, the node receives sensory information from simu_fastsim:
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
    rospy.Subscriber("/simu_fastsim/radars", Int16MultiArray, callback_radar)
    rospy.Subscriber("/simu_fastsim/odom", Odometry, callback_odom)
    rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)

    # Targetted operating frequency of the node:
    r = rospy.Rate(10) # 10hz

    # Q-learning related stuff
    # definition of states at time t and t-1
    S_t = ''
    S_tm1 = ''
    Q = {}

    # start time and timing related things
    startT = rospy.get_time()
    rospy.loginfo("Start time"+str(startT))
    trial = 0
    nbTrials = 60
    trialDuration = np.zeros((nbTrials))

    choice = -1
    rew = 0

    i2strat = ['wall follower','guidance']

    # Main loop:
    while (not rospy.is_shutdown()) and (trial <nbTrials):
      speed_l=0
      speed_r=0
      # processing of the sensory data :
      #------------------------------------------------
      # 1) has the robot found the reward ?
      #rospy.loginfo("pose: "+str(odom.pose.pose.position.x)+", "+str(odom.pose.pose.position.y))
      dist2goal = math.sqrt((odom.pose.pose.position.x-goalx)**2+(odom.pose.pose.position.y-goaly)**2)
      #rospy.loginfo(dist2goal)
      # if so, teleport it:
      if (dist2goal<30):
        rospy.wait_for_service('simu_fastsim/teleport')
        try:
          # teleport robot
          teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
          x  = 300 #20+random.randrange(520)
          y  = 40
          th = random.randrange(360)/2*math.pi
          resp1 = teleport(x, y, th)
          # store information about the duration of the finishing trial:
          currT = rospy.get_time()
          trialDuration[trial] = currT - startT
          startT = currT
          rospy.loginfo("Trial "+str(trial)+" duration:"+str(trialDuration[trial]))
          trial +=1
          rew = 1 
          odom.pose.pose.position.x = x
          odom.pose.pose.position.y = y
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e

      # 2) has the robot bumped into a wall ?
      #rospy.loginfo("BUMPERS "+str(bumper_r)+' '+str(bumper_l))
      if bumper_r or bumper_l:
        rew = -1
        #rospy.loginfo("BING! A wall...")

      # 3) build the state, that will be used by learning, from the sensory data
      #rospy.loginfo("Nb laser scans="+str(len(lasers.ranges)))
      if len(lasers.ranges) == 200:
        S_tm1 = S_t
        S_t   = ''
        # determine if obstacle on the left:
        wall='0'
        for l in lasers.ranges[angleLMin:angleLMax]:
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall
        # determine if obstacle in front:
        wall='0'
        for l in lasers.ranges[angleFMin:angleFMax]:
          #rospy.loginfo("front:"+str(l))
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall
        # determine if obstacle in front:
        wall='0'
        for l in lasers.ranges[angleRMin:angleRMax]:
          if l < th_neglectedWall:
            wall ='1'
        S_t += wall

        # check if we are receiving radar measurements
        if radar != 0:
          radar_list = []
          for i in range(len(radar)):
            radar_list.append(radar[i])
          #rospy.loginfo(str(radar_list))

        S_t += str(radar_list[0])

        #rospy.loginfo("S(t)="+S_t+" ; S(t-1)="+S_tm1)
        

      # The chosen gating strategy is to be coded here:
      #------------------------------------------------
      if gatingType=='random':
        choice = random.randrange(nbCh)
        #choice = 1
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='randomPersist':
        # a choice is made every 5 steps
        rospy.loginfo("randomPersist: arbitrage a coder.")
        pass
      #------------------------------------------------
      elif gatingType=='guidance':
        choice = 1
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='wallFollower':
        choice = 0
        rospy.loginfo("Module actif: "+i2strat[choice])
        speed_l=channel[choice].speed_left
        speed_r=channel[choice].speed_right
      #------------------------------------------------
      elif gatingType=='qlearning':
        rospy.loginfo("qlearning: arbitrage a coder.")
        pass
      #------------------------------------------------
      else:
        rospy.loginfo(gatingType+' unknown.')
        exit()

      #for i in range(nbCh): 
      #  channel[i]=v

      pub_l.publish(speed_l)
      pub_r.publish(speed_r)
      r.sleep()   
    
    # Log files opening
    logDuration = open('DureesEssais_a_'+str(alpha)+'_b_'+str(beta)+'_g_'+str(gamma)+'_'+str(startT),'w')

    for i in range(nbTrials):
      rospy.loginfo('T = '+str(trialDuration[i]))
      logDuration.write(str(i)+' '+str(trialDuration[i])+'\n')

    logDuration.close()

#-------------------------------------------
if __name__ == '__main__':

    nbch=int(sys.argv[1])
    gatingType=sys.argv[2]
    v=Channel()
    v.activated=False
    v.speed_left=0
    v.speed_right=0
    channel=[v for i in range(nbch)]
    try:
        strategy_gating(nbch,gatingType)
    except rospy.ROSInterruptException: pass
