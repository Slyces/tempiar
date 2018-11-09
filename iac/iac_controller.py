#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16,Float32,Bool,Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import sys
import random
import math


from my_stat import dist_pt, median, mean, variance
from expert import *

odom=Odometry()
odom_old=Odometry()
obj_pose=Odometry()

bumper_l=0
bumper_r=0

p_sound=0
p_speed_left=0
p_speed_right=0

p_dist_obj=0
p_dist_obj_old=0

ros_update_freq=5

# callbacks to read values sent by the simulator
def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    #rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)


def callback_robot_odom(data):
    global odom
    odom=data
    #rospy.loginfo(rospy.get_caller_id()+" Odometry: pose x=%f y=%f",odom.pose.pose.position.x,odom.pose.pose.position.y)

def callback_obj_odom(data):
    global obj_pose
    obj_pose=data
    #rospy.loginfo(rospy.get_caller_id()+" Odom obj: pose x=%f y=%f",obj_pose.pose.pose.position.x,obj_pose.pose.pose.position.y)


def callback_p_sound(data):
    global p_sound
    p_sound=data.data
    #print("Callback SOUND:"+str(p_sound))

def callback_p_speed_left(data):
    global p_speed_left
    p_speed_left=data.data/4.0
    #print("Callback SPEED LEFT:"+str(p_speed_left))

def callback_p_speed_right(data):
    global p_speed_right
    p_speed_right=data.data/4.0
    #print("Callback SPEED RIGHT:"+str(p_speed_right))

def callback_p_dist_obj(data):
    global p_dist_obj
    p_dist_obj=data.data/600.0
    if (p_dist_obj>1):
        p_dist_obj=1

def callback_p_dist_obj_old(data):
    global p_dist_obj_old
    p_dist_obj_old=data.data/600.0
    if(p_dist_obj_old>1):
        p_dist_obj_old=1




def motivation(expert,sma):
    ## to be completed ##
    return 0


# the main function.
def iac_controller():
    global odom_old
    rospy.init_node('iac_controller', anonymous=True)

    # we subscribe to the required simulation topics
    rospy.Subscriber("/simu_fastsim/odom", Odometry, callback_robot_odom)
    rospy.Subscriber("/simu_fastsim/obj_pose", Odometry, callback_obj_odom)
    rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
    rospy.Subscriber("/simu_fastsim/sound_back", Float32, callback_p_sound)
    rospy.Subscriber("/simu_fastsim/speed_left_back", Float32, callback_p_speed_left)
    rospy.Subscriber("/simu_fastsim/speed_right_back", Float32, callback_p_speed_right)

    rospy.Subscriber("/simu_fastsim/dist_obj", Float32, callback_p_dist_obj)
    rospy.Subscriber("/simu_fastsim/dist_obj_old", Float32, callback_p_dist_obj_old)

    # we prepare for publication
    pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
    pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)
    pub_s = rospy.Publisher('/simu_fastsim/sound', Float32 , queue_size=10)

    random.seed()

    r = rospy.Rate(ros_update_freq)

    last_action=[0,0,0]
    

    # we create the first expert that covers the whole range of possible input points
    # experts are stored in a list. There is only one at first, and it will be splitted
    # once it contains a certain number of points, see expert.py for more details.
    lexpert =[Expert([0,0,0,0],[1.001,1.001,1.001,1.001], "dist_obj",5)]


    nbaction=0
    nb_still=0

    stat_sound=[]


    dist_obj_old=0

    # we keep the list of last actions/distances for the expert updates
    last_action_dist=[]

    while not rospy.is_shutdown():


        dist_move_x=odom.pose.pose.position.x-odom_old.pose.pose.position.x
        dist_move_y=odom.pose.pose.position.y-odom_old.pose.pose.position.y
        dist_move=math.sqrt(dist_move_x*dist_move_x+dist_move_y*dist_move_y)
        odom_old=odom
        # if the robot is blocked against the wall, we make it go back for a while
        #print("Dist move="+str(dist_move))
        if (dist_move<0.0001):
            nb_still=nb_still+1
        else:
            nb_still=0

        if (nb_still>2):
            nb_still=0
            count=2*ros_update_freq
            while(count>0):
                pub_l.publish(-4)
                pub_r.publish(-4)
                pub_s.publish(-1)
                count=count-1
                r.sleep()
            continue
        
        # the last action that was applied (as sent back from the fastsim simulator through messages)
        last_action=[p_speed_left, p_speed_right, p_sound]
        if ((p_speed_left>=0) and (p_speed_right>=0) and (p_sound>=0)): 
            dist_obj_tm1=dist_obj_old
            action_found=0
            #print("Last_action_dist: "+str(last_action_dist))
            #print("p_speed_left: "+str(p_speed_left)+" p_speed_right: "+str(p_speed_right)+" p_sound: "+str(p_sound))

            # we look for the distance to object at the moment when the action was decided.
            # to this end, we have stores past actions and perceptions in a list, and we look
            # the distance that corresponds to the actions that was applied.
            # because of the asynchronous nature of ROS, some actions may get lost or delayed
            # we need then (1) to look for the action that was applied and (2) find the corresponding 
            # sensori-motor context. 
            for n in range(len(last_action_dist)-1,-1,-1):
                if (abs(last_action_dist[n][0]-p_speed_left)<0.001) and (abs(last_action_dist[n][1]-p_speed_right)<0.001) and  (abs(last_action_dist[n][2]-p_sound)<0.001):
                    # we have found the right time step
                    dist_obj_tm1=last_action_dist[n][3]
                    last_action_dist=last_action_dist[n+1:]
                    action_found=1
                    break

            if (action_found==0):
                print ("WARNING: that action not found in the action history !")

            # update of the predictors
            e=update_predictor(lexpert,last_action+[dist_obj_tm1],p_dist_obj)
            print("SOUND: "+str(last_action[2])+" DIST: "+str(p_dist_obj)+" SM: "+" ".join(map(str,last_action+[dist_obj_tm1])))
            if (last_action[2]>0.67) and (p_dist_obj>0):
                print("WARNING: sound say to to go the robot and distance >0: "+str(p_dist_obj))
        else:
            #print("NEGATIVE SPEED, WE DON'T UPDATE")
            pass

        dist_obj_old=p_dist_obj

        n_expert=-1

        ##### Choice of the action to apply for the exploration of sensori-motor space

        # Choice of a new action: at random at the beginning and, afterwards, it is still possible with a relatively low probability
        if (len(lexpert)<2) or (random.random()<=0.35):
            candidate_actions=[[random.random(),random.random(),random.random()]]
            a=0
            print("random action: "+str(candidate_actions[0]))
        else:
            candidate_actions=[]
                
            # The principle of action generation is the following:
            # 1. we generate a list of actions
            # 2. for each of them, we compute the corresponding motivation level (error, motivation maximization, etc.)
            # 3. we select the one with the highest motivation value
            
            # 1. generation of random actions to choose from
            for a in range(0,nba):
                candidate_actions.append([random.random(),random.random(),random.random()])

            #print("Randomly generated actions: "+str(candidate_actions))
                    
            # 2. and 3. choice of the most appropriate action
            a=0
            n_expert_c=which_expert(lexpert,candidate_actions[a]+[p_dist_obj])
            if (n_expert_c<0):
                print "ERREUR: no expert, we quit !"
                sys.exit(1)
                    
            # the most appropriate action is the one for which the motivation is the highest
            lp=motivation(lexpert[n_expert_c],candidate_actions[a]+[p_dist_obj])
            n_expert=n_expert_c
            l_n_expert=[n_expert]
            l_action=[0]
            #print("action expert: "+str(n_expert_c)),
            for ac in range(1,nba):
                n_expert_c=which_expert(lexpert,candidate_actions[ac]+[p_dist_obj])
                #print(" "+str(n_expert_c)),#+" action: "+str(candidate_actions[ac]+sm))
                if (n_expert_c<0):
                    print "ERREUR: no expert, we quit !"
                    sys.exit(1)
                lpc=motivation(lexpert[n_expert_c],candidate_actions[ac]+[p_dist_obj])
                if (abs(lpc-lp)<0.001):
                    l_n_expert.append(n_expert_c)
                    l_action.append(ac)
                elif(lpc>lp):
                    a=ac
                    lp=lpc
                    n_expert=n_expert_c
                    l_n_expert=[n_expert]
                    l_action=[ac]
            #print(" ")
            ia=random.randint(0,len(l_action)-1)
            a=l_action[ia]
            n_expert=l_n_expert[ia]
            print("Chosen action: "+str(candidate_actions[a])+" from expert "+str(n_expert)+" sound="+str(candidate_actions[a][2]))
            stat_sound.append(candidate_actions[a][2])
            #print("stat_sound len="+str(len(stat_sound)))
            if (len(stat_sound)>100):
                stat_sound=stat_sound[1:]
                print("Rep_sound "+str(len(filter(lambda x:x<0.34,stat_sound)))+" "+str(len(filter(lambda x:(x>=0.34)and(x<0.67),stat_sound)))+" "+str(len(filter(lambda x:x>=0.67,stat_sound)))+" (random, no_move, on_robot)")
            nbaction=nbaction+1
            
        pub_l.publish(4*candidate_actions[a][0])
        pub_r.publish(4*candidate_actions[a][1])
        pub_s.publish(candidate_actions[a][2])

        last_action_dist.append(candidate_actions[a]+[p_dist_obj])

        r.sleep()   


if __name__ == '__main__':


    try:
        iac_controller()
    except rospy.ROSInterruptException: pass
