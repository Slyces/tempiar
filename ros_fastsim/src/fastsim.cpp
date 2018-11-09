#define USE_SDL
#include <sstream>
#include <unistd.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "tf/transform_broadcaster.h"

#include "fastsim.hpp"

// custom services msgs
#include "fastsim/Teleport.h"
#include "fastsim/UpdateDisplay.h"

using namespace fastsim;

namespace fastsim {
  // speed left & right
  float sp_left = 0;
  float sp_right = 0;
  // "sound"
  float sound = 0;
  bool display = true;
  bool teleport = false;
  float teleport_x = -1;
  float teleport_y = -1;
  float teleport_theta = 0;
  bool new_speed_left = false;
  bool new_speed_right = false;
  // the callbacks
  void speed_left_cb(const std_msgs::Float32::ConstPtr& msg) {
    new_speed_left = true;
    sp_left = msg->data;
  }
  void speed_right_cb(const std_msgs::Float32::ConstPtr& msg) {
    new_speed_right = true;
    sp_right = msg->data;
  }
  void sound_cb(const std_msgs::Float32::ConstPtr& msg) {
    sound = msg->data;
  }
  bool display_cb(fastsim::UpdateDisplay::Request &req,
			 fastsim::UpdateDisplay::Response &res) {
    display = req.state;
    res.ack = true;
    return true;
  }
  bool teleport_cb(fastsim::Teleport::Request &req,
		   fastsim::Teleport::Response &res) {
    teleport = true;
    teleport_x = req.x;
    teleport_y = req.y;
    teleport_theta = req.theta;
    res.ack = true;
    return true;
  }
}

void publish_radars(const ros::Publisher& sensor_radars, 
		    const boost::shared_ptr<Robot>& robot) {
  if (!robot->get_radars().empty()) {
    std_msgs::Int16MultiArray msg_radar;
    for (size_t i = 0; i < robot->get_radars().size(); ++i)
      msg_radar.data.push_back(robot->get_radars()[i].get_activated_slice());
    sensor_radars.publish(msg_radar);
  }
}

void publish_lasers(const ros::Publisher& sensor_lasers,		    
		    const boost::shared_ptr<Robot>& robot) {
  if (!robot->get_lasers().empty()) {
    // basic lasers
    std_msgs::Float32MultiArray laser_msg;
    for (size_t i = 0; i < robot->get_lasers().size(); ++i)
      laser_msg.data.push_back(robot->get_lasers()[i].get_dist());
    sensor_lasers.publish(laser_msg);
  }
}


void publish_laser_scan(const ros::Publisher& laser_scan,
			tf::TransformBroadcaster& tf,
			const boost::shared_ptr<Robot>& robot,
			const ros::Time& sim_time) {
  if (!robot->get_laser_scanners().empty()) {
    // ROS laser scan
    sensor_msgs::LaserScan scan_msg;
    const LaserScanner& scanner = robot->get_laser_scanners()[0];
    scan_msg.angle_min = scanner.get_angle_min();
    scan_msg.angle_max = scanner.get_angle_max();
    scan_msg.angle_increment = scanner.get_angle_increment();
    scan_msg.range_min = 0.0;
    scan_msg.range_max = scanner.get_range_max();
    for(size_t i = 0; i < scanner.get_lasers().size(); i++)  {
      // we ignore the value if there is nothing
      // I don't know if is right...
      // see: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
      if (scanner.get_lasers()[i].get_dist() > 0)
	scan_msg.ranges.push_back(scanner.get_lasers()[i].get_dist());
      else
	scan_msg.ranges.push_back(0);
    }
    scan_msg.header.frame_id = "base_laser_link";;
    scan_msg.header.stamp = sim_time;
    laser_scan.publish(scan_msg);
    
    // now the tf
    tf::Quaternion laserQ;
    laserQ.setRPY(0.0, 0.0, robot->get_pos().theta());
    tf::Transform txLaser = 
      tf::Transform(laserQ, tf::Point(0,//robot->get_pos().x(), 
				      0,//robot->get_pos().y(), 
				      0.15));
    tf.sendTransform(tf::StampedTransform(txLaser, sim_time, 
					  "base_link", "base_laser_link"));
    tf::Transform txIdentity(tf::createIdentityQuaternion(),
			     tf::Point(0, 0, 0));
    tf.sendTransform(tf::StampedTransform(txIdentity, sim_time, 
					  "base_footprint", "base_link"));
  }
}

void publish_odometry(const ros::Publisher& odom,
		      tf::TransformBroadcaster& tf,
		      const boost::shared_ptr<Robot>& robot,
		      const ros::Time& sim_time,
		      float sim_dt) {  
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = robot->get_pos().x();
  msg.pose.pose.position.y = robot->get_pos().y();
  msg.pose.pose.orientation = 
    tf::createQuaternionMsgFromYaw(robot->get_pos().theta());
  msg.twist.twist.linear.x = robot->get_vx() / sim_dt;
  msg.twist.twist.linear.y = robot->get_vy() / sim_dt;
  msg.twist.twist.angular.z = robot->get_va() / sim_dt;
  msg.header.frame_id = "odom";
  msg.header.stamp = sim_time;

  odom.publish(msg);

  tf::Quaternion odomQ;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, odomQ);
  tf::Transform txOdom(odomQ, 
		       tf::Point(msg.pose.pose.position.x,
				 msg.pose.pose.position.y, 0.0));
  tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
					"odom", "base_footprint"));
}

void publish_odometry_obj(const ros::Publisher& obj_pose,
		      const boost::shared_ptr<IlluminatedSwitch>& is,
		      const ros::Time& sim_time,
		      float sim_dt) {  
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = is->get_x();
  msg.pose.pose.position.y = is->get_y();
  msg.pose.pose.orientation = 
    tf::createQuaternionMsgFromYaw(0);
  msg.twist.twist.linear.x = 0;
  msg.twist.twist.linear.y = 0;
  msg.twist.twist.angular.z = 0;
  msg.header.frame_id = "obj_pose";
  msg.header.stamp = sim_time;

  obj_pose.publish(msg);
}

int main(int argc, char **argv) {
  // init
  ros::init(argc, argv, "fastsim");
  ros::NodeHandle n("~");

  // load ROS config
  std::string settings_name;
  n.param("settings", settings_name, std::string("envs/example.xml"));
  std::string path;
  n.param("path", path, std::string("."));
  bool sync = false;
  n.param("sync", sync, false);
  ROS_WARN_STREAM("changing path to "<<path);
  ROS_WARN_STREAM("settings is "<<settings_name);
  ROS_WARN_STREAM("sync is "<<sync);
  chdir(path.c_str());

  // fastsim config
  Settings settings(settings_name);
  boost::shared_ptr<Robot> robot = settings.robot();
  boost::shared_ptr<Map> map = settings.map();
  fastsim::display = settings.display();
  
  // bumpers
  ros::Publisher left_bumper = 
    n.advertise<std_msgs::Bool>("left_bumper", 10);
  ros::Publisher right_bumper = 
    n.advertise<std_msgs::Bool>("right_bumper", 10);

  // lasers
  ros::Publisher sensor_lasers;
  if (!robot->get_lasers().empty())
    sensor_lasers = n.advertise<std_msgs::Float32MultiArray>("lasers", 10);
  ros::Publisher laser_scan;
  if (!robot->get_laser_scanners().empty())
    laser_scan = n.advertise<sensor_msgs::LaserScan>("laser_scan", 1);

  // radars (-> goals)
  ros::Publisher sensor_radars;
  if (!robot->get_radars().empty())
    sensor_radars = n.advertise<std_msgs::Int16MultiArray>("radars", 10);

  // odometry
  ros::Publisher odom = n.advertise<nav_msgs::Odometry>("odom", 10);

  ros::Publisher obj_pose = n.advertise<nav_msgs::Odometry>("obj_pose", 10);

  ros::Publisher p_sound = n.advertise<std_msgs::Float32>("sound_back", 10);
  ros::Publisher p_speed_left = n.advertise<std_msgs::Float32>("speed_left_back", 10);
  ros::Publisher p_speed_right = n.advertise<std_msgs::Float32>("speed_right_back", 10);

  ros::Publisher p_dist_obj = n.advertise<std_msgs::Float32>("dist_obj", 10);
  ros::Publisher p_dist_obj_old = n.advertise<std_msgs::Float32>("dist_obj_old", 10);

  tf::TransformBroadcaster tf;



  // clock
  // remember to set the /use_sim_time Parameter to true in launch files
  ros::Publisher clock = n.advertise<rosgraph_msgs::Clock>("clock", 10);

  // inputs
  ros::Subscriber speed_left = 
    n.subscribe("speed_left", 10, speed_left_cb);
  ros::Subscriber speed_right = 
    n.subscribe("speed_right", 10, speed_right_cb);

  // For the experiment on curiosity
  ros::Subscriber sound_ = n.subscribe("sound", 10, sound_cb);

  // init the window (should we make this easy to de-activate?).
  //  Display d(map, *robot);
  // services
  ros::ServiceServer service_teleport = 
    n.advertiseService("teleport", teleport_cb);
  ros::ServiceServer service_display = 
    n.advertiseService("display", display_cb);
  
  // init the window
  boost::shared_ptr<Display> d;
  
  ros::Rate loop_rate(30);
  float sim_dt = 1.0 / 30.0f;
  ros::Time sim_time = ros::Time::now();

  // For the experiment on curiosity
  int old_sound=0;
  fastsim::Posture old_obj_pos,obj_pos;
  fastsim::Posture old_robot_pos=robot->get_pos();
  int last_action=0;
  if (map->get_illuminated_switches().size()>0) {
    obj_pos.set_x(map->get_illuminated_switches()[0]->get_x());
    obj_pos.set_y(map->get_illuminated_switches()[0]->get_y());
    old_obj_pos=obj_pos;
  }

  int k = 0;
  while (ros::ok()) {
    if (fastsim::teleport)
      {
	std::cout<<"teleporting to:" 
		 << teleport_x
		 << "," << teleport_y <<","
		 << "," << teleport_theta << std::endl;
	fastsim::teleport = false;
	robot->set_pos(Posture(teleport_x, teleport_y, teleport_theta));
	robot->move(0, 0, map);
	k = 0;
      }

    if (!sync || (new_speed_left && new_speed_right))
      {
	++k;
	robot->move(fastsim::sp_left, fastsim::sp_right, map);
	new_speed_left = false;
	new_speed_right = false;
    }

    // bumpers
    std_msgs::Bool msg_left_bumper, msg_right_bumper;
    msg_left_bumper.data = robot->get_left_bumper();
    msg_right_bumper.data = robot->get_right_bumper();
    left_bumper.publish(msg_left_bumper);
    right_bumper.publish(msg_right_bumper);

    // other publishers
    publish_lasers(sensor_lasers, robot);
    publish_laser_scan(laser_scan, tf, robot, sim_time);
    publish_radars(sensor_radars, robot);
    publish_odometry(odom, tf, robot, sim_time, sim_dt);
    for(int i=0;i<map->get_illuminated_switches().size();i++) 
      publish_odometry_obj(obj_pose, map->get_illuminated_switches()[i], sim_time, sim_dt);


    // and the clock
    rosgraph_msgs::Clock msg_clock;
    msg_clock.clock = sim_time;
    clock.publish(msg_clock);
    
    ros::spinOnce();
    
    if (fastsim::display)
      {
	if (!d)
	  d = boost::make_shared<Display>(map, *robot);
	d->update();
      }

        
    // explorer behavior
    /*float s1 = 1, s2 = 1;
    if (robot->get_left_bumper() 
	|| robot->get_right_bumper()
	|| (robot->get_vx() == 0 && robot->get_vy() == 0 && robot->get_va() == 0))
      {s2 = -1; s1 = 1; }
    
    robot->move(s1, s2, map);
    */
    robot->move(fastsim::sp_left, fastsim::sp_right, map);

    // For the experiment on curiosity
    if (map->get_illuminated_switches().size()>0) {
      if (fastsim::sound<0.34) {
	//ROS_WARN_STREAM("RANDOM MOVE (sound="<<fastsim::sound<<")"<<std::endl);
	// random move 
	map->get_illuminated_switches()[0]->set_pos(10+rand()%((int)floor(map->get_real_w())-20),10+rand()%((int)floor(map->get_real_h())-20));
	last_action=0;
      }
      else if (fastsim::sound<0.67) {
	//ROS_WARN_STREAM("NO MOVE (sound="<<fastsim::sound<<")"<<std::endl);
	if (last_action==0)
	  map->get_illuminated_switches()[0]->set_pos(robot->get_pos().get_x(),robot->get_pos().get_y());
	last_action=1;
      }
      else {
	//ROS_WARN_STREAM("JUMP ON THE ROBOT (sound="<<fastsim::sound<<")"<<std::endl);
	// jumps onto the robot
	map->get_illuminated_switches()[0]->set_pos(robot->get_pos().get_x(),robot->get_pos().get_y());
	last_action=2;
      }
    
    
    
      std_msgs::Float32 msg_p_sound, msg_p_speed_right, msg_p_speed_left;
      msg_p_sound.data = fastsim::sound; 
      msg_p_speed_right.data = fastsim::sp_right;
      msg_p_speed_left.data = fastsim::sp_left;
      p_sound.publish(msg_p_sound);
      p_speed_left.publish(msg_p_speed_left);
      p_speed_right.publish(msg_p_speed_right);
    
      if (fabs(fastsim::sound-old_sound)>0.01) {
	
	old_obj_pos=obj_pos;
      }
      
      obj_pos.set_x(map->get_illuminated_switches()[0]->get_x());
      obj_pos.set_y(map->get_illuminated_switches()[0]->get_y());
    
      std_msgs::Float32 msg_dist_obj,msg_dist_obj_old;
      msg_dist_obj.data=obj_pos.dist_to(robot->get_pos());
      p_dist_obj.publish(msg_dist_obj);
    
      msg_dist_obj_old.data=old_obj_pos.dist_to(old_robot_pos);
      p_dist_obj_old.publish(msg_dist_obj_old);
    
    
      old_robot_pos=robot->get_pos();
      
      old_sound=fastsim::sound;
    }

    loop_rate.sleep();
    sim_time = ros::Time::now();//sim_time + ros::Duration(sim_dt);
  }
  
  return 0;
}
