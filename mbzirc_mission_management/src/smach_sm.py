#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import mavros
import mavros_msgs
import os
import sys

import math
import cv2
import numpy as np

from std_msgs.msg import Float32, Header, Float64
from sensor_msgs.msg import NavSatFix, Range
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
from mavros import command, mission
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, State, WaypointReached, ParamValue 
from mavros_msgs.srv import CommandLong, SetMode, WaypointPush, CommandHome, WaypointClear, ParamSet
from smach import CBState
from pymavlink import mavutil

from waypoint_mp2mavros import create_waypoint_list
# from RecognitionThread import RecognitionThread,CameraError
from position_memory_class import Position_Memory
from ConfigParser import SafeConfigParser

def setGuidedMode():
   rospy.wait_for_service('/mavros/set_mode')
   global isModeGuided
   while mode != 'GUIDED':			
      try:
         flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
         isModeGuided = flightModeService(custom_mode='GUIDED') 
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setStabilizeMode():
   rospy.wait_for_service('/mavros/set_mode')
   global isModeStabilize
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeStabilize = flightModeService(custom_mode='STABILIZE') 
       print isModeStabilize
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. Check that GPS is enabled", e

def setAutoMode():
   rospy.wait_for_service('/mavros/set_mode')
   while mode != 'AUTO':			
      try:
         flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
         isModeAuto = flightModeService(custom_mode='AUTO') 
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setRTLMode():
   rospy.wait_for_service('/mavros/set_mode')
   while mode != 'RTL':			
      try:
         flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
         isModeRTL = flightModeService(custom_mode='RTL') 
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setArm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s", e
       
def setDisarm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(False)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s", e

def setTakeoffMode(TOheight):
   rospy.wait_for_service('/mavros/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       takeoffService(altitude = TOheight, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       time.sleep(0.2)
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s", e

def setLandMode():
   rospy.wait_for_service('/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       isLanding = landService()
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land ", e

def setHome():
   rospy.wait_for_service('/mavros/cmd/set_home')
   try:
       homeGPS_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
       homeAlt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
       rospy.sleep(1)
       setHomeService = rospy.ServiceProxy('/mavros/cmd/set_home', mavros_msgs.srv.CommandHome)
       setHomeService()
       rospy.sleep(0.5)
   except rospy.ServiceException, e:
       print "Service set home call failed: %s", e

def WPs_Mission():
   rospy.wait_for_service('/mavros/mission/push')
   path = os.path.join(os.path.abspath(os.path.dirname(__file__))+"/", "wp_file")
#    rospy.loginfo (os.path.abspath(os.path.dirname(__file__)))
   waypoint_list = create_waypoint_list(path)
   with open(path,'r') as file_handler:
        lines = file_handler.readlines()
   lines.pop(1)
   lines.insert(1, '0\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t'+str(lat)+'\t'+str(lon)+'\t'+str(alt)+'\t1\n')
   global total_waypoints
   total_waypoints = len(lines)-2 
   with open(path,'w') as file_handler:
        file_handler.writelines(lines)
   global isMissionLoaded
   try:
       MissionService = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
       isMissionLoaded = MissionService(0, waypoint_list)
   except rospy.ServiceException, e:
       print "Service call failed: %s", e

def Clear_Mission():
   rospy.wait_for_service('/mavros/mission/clear')
   try:
       ClearService = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
       ClearService()
   except rospy.ServiceException, e:
       print "Service call failed: %s", e 

def Param_Set(height, auto_velocity):
   #rospy.wait_for_service('/mavros/param/set')
   try:
       Parameter_Set = rospy.ServiceProxy('/mavros/param/set',  mavros_msgs.srv.ParamSet)
       mySpeed = ParamValue()
       mySpeed.real = auto_velocity
       Parameter_Set('WPNAV_SPEED', mySpeed)
       myRtlAlt = ParamValue()
       myRtlAlt.real = height*100
       Parameter_Set('RTL_ALT', myRtlAlt)
   except rospy.ServiceException, e:
       print "Service call failed: %s", e

def pose_callback(data):
    global alt
    alt = data.pose.position.z 

def vision_callback(data):
   global locked
   global err_x_pix
   global err_y_pix
   global target_dx
   global target_dy
   global target_dist_normal
   global target_yaw
   global theta_err


def manuale_callback(data):
    global mode
    mode = data.mode

def compass_callback(data):
    global compass
    compass = data.data

def WP_reached_callback(data):
    global CurrentWaypoint
    CurrentWaypoint=data.wp_seq

def home_callback(data):
    global lat
    global lon
    lat = data.latitude 
    lon = data.longitude

def cmd_calibrate_pressure():
   rospy.wait_for_service('/mavros/cmd/command')
   global cal_pres
   try:
       cal_pres_Service = rospy.ServiceProxy('/mavros/cmd/command', mavros_msgs.srv.CommandLong)
       cal_pres = cal_pres_Service(0, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0)
   except rospy.ServiceException, e:
       print "Service calibration failed: %s", e

def cmd_condition_yaw(yaw):
   rospy.wait_for_service('/mavros/cmd/command')
   try:
       condition_yaw_Service = rospy.ServiceProxy('/mavros/cmd/command', mavros_msgs.srv.CommandLong)
       condition_yaw = condition_yaw_Service(0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, 0, 1, 0, 0, 0, 0)
   except rospy.ServiceException, e:
       print "condition yaw failed: %s", e

def add_angles(ang1, ang2):
   ang = ang1+ang2
   if ang > 360:
     ang -= 360
   elif ang < 0: 
     ang += 360
   else:
     ang = ang
   return ang
 
def saturate(value, minimum, maximum, value_zero):
   if value > maximum:
     value = maximum
   elif value < minimum:
     value = minimum
   elif value < value_zero and value > -value_zero:
     value = 0
   else:
     value = value
   return value

def saturation_velZ(value, alt, Hmin, Hmax, err_y, Hmax85, Hmin115):
  if alt > Hmin115 and alt < Hmax85 and ((err_y > 0) or (err_y <= -0.1)):
     value = value
  elif Hmin < alt <= Hmin115 and err_y >= 0:
     value = -0.3*err_y*abs(alt-Hmin)
  elif Hmax85 <= alt < Hmax and err_y < -0.1:
     value = -0.3*err_y*abs(alt-Hmax)  
  elif Hmax85 <= alt < Hmax and err_y >= 0:
       value = value 
  elif Hmin < alt <= Hmin115 and err_y < -0.1: 
       value = value
  elif alt <= Hmin:
       value = 1
  else:
     value = 0 
  if -0.1 < value < 0.1:
     value = 0 
  return value

'''def saturation_velZ(value, alt, Hmin, Hmax, err_y):
  if alt > (1.15*Hmin) and alt < (0.85*Hmax) and abs(err_y) > 0.1:
     value = value
  elif (1.05*Hmin) < alt <= (1.15*Hmin) and err_y > 0.1:
     value = -0.3*err_y*abs(alt-(1.05*Hmin))
  elif (0.85*Hmax) <= alt < (0.95*Hmax) and err_y < -0.1:
     value = -0.3*err_y*abs(alt-(0.95*Hmax))  
  elif (0.85*Hmax) <= alt < (0.95*Hmax) and err_y > 0.1:
       value = value 
  elif (1.05*Hmin) < alt <= (1.15*Hmin) and err_y < -0.1: 
       value = value
  else:
     value = 0 
  return value'''

def yaw_alignment(err_x_pix, errx_min_pix, psi_dot):
   if err_x_pix >= errx_min_pix:
      vel_yaw = -psi_dot
   elif err_x_pix <= -errx_min_pix:
      vel_yaw = psi_dot 
   else:
      vel_yaw = 0
   return vel_yaw

def get_bearing(location_1, location_2):
   dlat = location_2[0] - location_1[0]
   dlon = location_2[1] - location_1[1]
   bearing = 180/math.pi*math.atan2(dlon, dlat)
   if bearing < 0:
    bearing +=360
   return bearing

def get_distance_meters(alocation_1, alocation_2):
   dlat = alocation_2[0] - alocation_1[0]
   dlon = alocation_2[1]- alocation_1[1]
   return math.sqrt((dlat*dlat)+(dlon*dlon))*1.113195e5

def send_ned_velocity(v_x, v_y, v_z):
   set_velocity = PositionTarget()
   set_velocity.coordinate_frame = 1    # local NED  
   set_velocity.velocity.x = v_y        # verificare che le componenti siano invertite
   set_velocity.velocity.y = v_x
   set_velocity.velocity.z = v_z 
   set_velocity.type_mask = 4039 
   setpoint_velocity_pub.publish(set_velocity)
   time.sleep(0.5)

def set_velocity_body(v_x, v_y, v_z, rate):
   set_velocity = PositionTarget()
   set_velocity.coordinate_frame = 8    # BODY FRAME
   set_velocity.velocity.x = v_y        # verificare che le componenti siano invertite
   set_velocity.velocity.y = v_x
   set_velocity.velocity.z = v_z
   set_velocity.yaw_rate = rate 
   set_velocity.type_mask = 1479 
   setpoint_velocity_pub.publish(set_velocity)
   time.sleep(0.5)

def set_target_velocity(v_x, v_y, v_z, phi, theta, psi):
   set_angular_velocity = TwistStamped()
   set_angular_velocity.twist.linear.x = v_x 
   set_angular_velocity.twist.linear.y = v_y   
   set_angular_velocity.twist.linear.z = v_z  
   set_angular_velocity.twist.angular.x = theta  
   set_angular_velocity.twist.angular.y = phi 
   set_angular_velocity.twist.angular.z = psi 
   setpoint_angular_velocity_pub.publish(set_angular_velocity)
   time.sleep(0.5)

def target_location(target_dist_normal, target_dx, target_dy, bearing, my_latit, my_longit, my_altit):
   alt_min = 1    
   alt_max = 15
   target_dist = math.sqrt((target_dist_normal*target_dist_normal)+(target_dx*target_dx))
   d_latit = target_dist*math.cos((bearing)*math.pi/180)/1.113195e5
   d_longit = target_dist*math.sin((bearing)*math.pi/180)/1.113195e5
   target_latit = my_latit + d_latit
   target_longit = my_longit + d_longit
   target_altit = my_altit - target_dy 
   target_altit = saturate(target_altit, alt_min, alt_max, 0)
   target_loc = [target_latit, target_longit, target_altit]
   return target_loc

@smach.cb_interface(input_keys=['height','auto_vel', 'yaw_behavior'], output_keys=[], outcomes=['finished','failed'])
def loadMission_cb(user_data):
    rospy.loginfo('mode STABILIZE')
    setStabilizeMode()
    rospy.loginfo('Set Home location to current location')
    rospy.loginfo('Load Mission')
    position_memory.wipe_memory()
    Clear_Mission()
    Param_Set(user_data.height, user_data.auto_vel)
    time.sleep(0.5)
    # Calibrazioni
    cmd_calibrate_pressure()
    time.sleep(2)
    setArm()
    setHome()
    WPs_Mission()
    time.sleep(2)
    setDisarm()
    time.sleep(4)
    # print 'start recording'
    # rec_and_show.start_recorder()
    if 'isModeStabilize' and 'isMissionLoaded' and 'cal_press':
         return 'finished'
    else:
         return 'failed'

@smach.cb_interface(input_keys=['height'], output_keys=[], outcomes=['finished','failed'])
def takeoff_cb(user_data):
    rospy.loginfo('mode GUIDED')
    setGuidedMode()
    rospy.loginfo('Arm')
    setArm()
    rospy.loginfo('Taking Off')
    takeoff_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
    rospy.sleep(0.5)
    alt_start = alt
    setTakeoffMode(user_data.height)
    start_takeoff_time=time.time()
    while mode == 'GUIDED' and alt < (user_data.height*0.85):
           t = time.time()-start_takeoff_time
           if t >=10 and alt == alt_start:
              break
    if mode == 'ALT_HOLD':
        Clear_Mission()
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill mavros')
        sis.stop()
    elif alt <= (user_data.height*1.15) and alt > (user_data.height*0.85) and mode == 'GUIDED':
         return 'finished'
    else:
         return 'failed'

@smach.cb_interface(input_keys=[], output_keys=['target_loc','target_dx','target_dy','errX_pix','errY_pix'], outcomes=['finished','reaching','failed'])
def auto_cb(user_data):
	rospy.loginfo('Mission Searching')
	setAutoMode() 
	locked = 0
	deltaTime = 9999
	waypoint_topic = rospy.Subscriber('mavros/mission/reached', WaypointReached, WP_reached_callback)
	position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
	Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
	compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)

   vision_sub=rospy.Subscriber('distance_finder/target_pos',message_type,vision_callback)
   
	rospy.sleep(0.5)
	while deltaTime > 30:
		if mode == 'ALT_HOLD' or locked == 1:
			global CurrentWaypoint
			CurrentWaypoint = 0 
			break
		try:
			waypointMsg = rospy.wait_for_message('mavros/mission/reached', WaypointReached,0.1)
			timeReachedWaypoint = waypointMsg.header.stamp.secs 
			deltaTime = rospy.Time.now().secs-timeReachedWaypoint

			(locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker() 
			if locked == 1:
				bearing = add_angles(compass, target_yaw)
				target_loc = target_location(target_dist_normal, target_dx, target_dy, bearing, lat, lon, alt)
				err_dx = target_dx
				err_dy = target_dy
				errX_pix = err_x_pix
				errY_pix = err_y_pix
				print '1Check balloon list'
				if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
					print '1Balloon already reached'
					locked = 0
		except rospy.exceptions.ROSException:
			deltaTime = 9999
			locked = 0
	print CurrentWaypoint
	print total_waypoints
	while CurrentWaypoint < total_waypoints and mode == 'AUTO' and locked == 0:
		(locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker()
		if locked == 1:
			bearing = add_angles(compass, target_yaw)
			target_loc = target_location(target_dist_normal, target_dx, target_dy, bearing, lat, lon, alt)
			err_dx = target_dx
			err_dy = target_dy
			errX_pix = err_x_pix
			errY_pix = err_y_pix
			print 'Check balloon list'
			print position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2])
			if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
				print 'Balloon already reached'
				locked = 0
			else:
				break 
	if mode == 'ALT_HOLD':
		Clear_Mission()
		print 'Stop VideoGet and VideoShow' 
		# rec_and_show.stop()	
		rospy.signal_shutdown('Quit for Alt_hold')
		os.system('rosnode kill mavros')
		sis.stop()
	elif CurrentWaypoint == total_waypoints:
		Clear_Mission()
		return 'finished'
	elif locked == 1:
		user_data.target_loc = target_loc
		user_data.target_dx = err_dx
		user_data.target_dy = err_dy
		user_data.errX_pix = errX_pix
		user_data.errY_pix = errY_pix
		return 'reaching'
	else:
		return 'failed'

@smach.cb_interface(input_keys=['Hmax85','Hmin115','target_loc','target_dx','target_dy','errX_pix','errY_pix','kerr_velx','dist_min','vel_x_min','vel_x_max','v_zero','kerr_vely','vel_y_min','vel_y_max','yaw_rateMax','yaw_rateMin','errx_max_pix','errx_min_pix','target_loc','erry_min','k_z','alt_min','alt_max'], output_keys=['target_loc','target_dx','target_dy','errX_pix','errY_pix'], outcomes=['finished','GPS','failed'])
def reaching_cb(user_data):
    rospy.loginfo('mode GUIDED')
    setGuidedMode()
    rospy.loginfo('Balloon found')
    target_loc = user_data.target_loc
    err_dx = user_data.target_dx
    err_dy = user_data.target_dy
    errX_pix = user_data.errX_pix 
    errY_pix = user_data.errY_pix 
    rospy.loginfo('Mission Reaching')
    (locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, psi_err, theta_err) = rec_and_show.ballonchecker() 
    position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
    Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
    rospy.sleep(0.5)
    if locked == 1:
        # Calcolo componenti
        vel_x = user_data.kerr_velx*(target_dist_normal-user_data.dist_min)
        vel_x = saturate(vel_x, user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
        vel_y = user_data.kerr_vely*target_dx
        vel_y = saturate(vel_y, user_data.vel_y_min, user_data.vel_y_max, user_data.v_zero)
        vel_z = -user_data.k_z*(target_dy)
        vel_z = saturation_velZ(vel_z, alt, user_data.alt_min, user_data.alt_max, target_dy, user_data.Hmax85, user_data.Hmin115)
        psi_dot = (((user_data.yaw_rateMax-user_data.yaw_rateMin)/(user_data.errx_max_pix-user_data.errx_min_pix))*(abs(err_x_pix)-user_data.errx_min_pix))+user_data.yaw_rateMin
        vel_yaw = yaw_alignment(err_x_pix, user_data.errx_min_pix, psi_dot)
        print "Vx=%.2f m/sec" % (vel_y),"; Vy=%.2f m/sec" % (vel_x),"; Vz=%.2f m/sec" % (vel_z),"; yaw_rate=%.2f rad/s" % (vel_yaw)
        # Invio Comando
        set_velocity_body(vel_x, vel_y, vel_z, vel_yaw)
    while locked == 1 and (vel_x > 0 or vel_z != 0) and mode == 'GUIDED':
        vel_x = user_data.kerr_velx*(target_dist_normal-user_data.dist_min)
        vel_x = saturate(vel_x, user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
        vel_y = user_data.kerr_vely*target_dx
        vel_y = saturate(vel_y, user_data.vel_y_min, user_data.vel_y_max, user_data.v_zero)
        vel_z = -user_data.k_z*(target_dy)
        vel_z = saturation_velZ(vel_z, alt, user_data.alt_min, user_data.alt_max, target_dy, user_data.Hmax85, user_data.Hmin115)
        psi_dot = (((user_data.yaw_rateMax-user_data.yaw_rateMin)/(user_data.errx_max_pix-user_data.errx_min_pix))*(abs(err_x_pix)-user_data.errx_min_pix))+user_data.yaw_rateMin
        vel_yaw = yaw_alignment(err_x_pix, user_data.errx_min_pix, psi_dot)
        print "Vx=%.2f m/sec" % (vel_y),"; Vy=%.2f m/sec" % (vel_x),"; Vz=%.2f m/sec" % (vel_z),"; yaw_rate=%.2f rad/s\r" % (vel_yaw),
        sys.stdout.flush()
        set_velocity_body(vel_x, vel_y, vel_z, vel_yaw)
        (locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker()
        if locked == 0:
            break
        else:
            bearing = add_angles(compass, target_yaw)
            target_loc = target_location(target_dist_normal, target_dx, target_dy, bearing, lat, lon, alt)
            err_dx = target_dx
            err_dy = target_dy
            errX_pix = err_x_pix
            errY_pix = err_y_pix
    if mode == 'ALT_HOLD':
        Clear_Mission()
        print 'Stop VideoGet and VideoShow' 
      #   rec_and_show.stop()	
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill mavros')
        sis.stop()
    elif locked == 1 and vel_x <= 0 and vel_z == 0: 
        print 'Balloon reached!'  
        return 'finished'
    elif locked == 0:
        user_data.target_loc = target_loc
        print target_loc
        return 'GPS'
    else:
        return 'failed'

@smach.cb_interface(input_keys=['kerr_velx','dist_min','vel_x_min','vel_x_max','v_zero','kerr_vely','vel_y_min','vel_y_max','target_loc'], output_keys=[], outcomes=['finished','reaching','failed'])
def reachingGPS_cb(user_data):
	rospy.loginfo('Mission Reaching GPS')
	(locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err) = rec_and_show.ballonchecker() 
	position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
	Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
	rospy.sleep(0.5)
	my_latit = lat
	my_longit = lon
	my_altit = alt
	my_loc = [my_latit, my_longit, my_altit]
	target_loc = user_data.target_loc
	if locked == 0:
		target_myloc_bearing = get_bearing(my_loc,target_loc)
		cmd_condition_yaw(target_myloc_bearing)
		rospy.sleep(0.5)
		dist_target_myloc = get_distance_meters(my_loc,target_loc)
		V = user_data.kerr_velx*(dist_target_myloc-user_data.dist_min)
		V = saturate(V,user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
		V_North = V*math.cos(target_myloc_bearing*math.pi/180)
		V_East = V*math.sin(target_myloc_bearing*math.pi/180)
		send_ned_velocity(V_North, V_East, 0)
		while V > 0 and mode == 'GUIDED':
			my_loc = [lat, lon, alt]
			target_myloc_bearing = get_bearing(my_loc,target_loc)
			cmd_condition_yaw(target_myloc_bearing)
			rospy.sleep(0.5)				
			dist_target_myloc = get_distance_meters(my_loc,target_loc)
			V = user_data.kerr_velx*(dist_target_myloc-user_data.dist_min)
			V = saturate(V,user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
			V_North = V*math.cos(target_myloc_bearing*math.pi/180)
			V_East = V*math.sin(target_myloc_bearing*math.pi/180)
			send_ned_velocity(V_North, V_East, 0)
			(locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err) = rec_and_show.ballonchecker()
			if locked == 1:
				break 
	if mode == 'ALT_HOLD':
		Clear_Mission()
		print 'Stop VideoGet and VideoShow' 
		rec_and_show.stop()	
		rospy.signal_shutdown('Quit for Alt_hold')
		os.system('rosnode kill mavros')
		sis.stop()
	elif locked == 1 and mode == 'GUIDED':
		return 'reaching'
	elif V <= 0 and mode == 'GUIDED' and locked == 0: 
		print 'GPS reached! Balloon not found'
		return 'finished'
	else:
		return 'failed'

@smach.cb_interface(input_keys=['Hmax85','Hmin115','k_action','dist_min','vel_x_min','vel_x_max','v_zero','kerr_vely','vel_y_min','vel_y_max','delta_alt','yaw_rateMax','yaw_rateMin','errx_max_pix','errx_min_pix','erry_min','k_z','alt_min','alt_max'], output_keys=[], outcomes=['finished','failed'])
def action_cb(user_data):
	rospy.loginfo('Mission Action')
	position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
	Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
	compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
	(locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker()
	rospy.sleep(0.5)       
	time_ini = rospy.Time.now().secs
	print 'Action: count'
	while (rospy.Time.now().secs-time_ini) < config.getint('GAPL','action_time') and mode == 'GUIDED':
		(locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker()
		if locked == 1:
			vel_x = 1.1*user_data.k_action*(target_dist_normal-user_data.dist_min)
			vel_x = saturate(vel_x, user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
			vel_y = user_data.kerr_vely*target_dx
			vel_y = saturate(vel_y, user_data.vel_y_min, user_data.vel_y_max, user_data.v_zero)
			vel_z = -user_data.k_z*(target_dy)
			vel_z = saturation_velZ(vel_z, alt, user_data.alt_min, user_data.alt_max, target_dy, user_data.Hmax85, user_data.Hmin115)
			psi_dot = (((user_data.yaw_rateMax-user_data.yaw_rateMin)/(user_data.errx_max_pix-user_data.errx_min_pix))*(abs(err_x_pix)-user_data.errx_min_pix))+user_data.yaw_rateMin
			vel_yaw = yaw_alignment(err_x_pix, user_data.errx_min_pix, psi_dot)
			print "Vx=%.2f m/sec" % (vel_x),"; Vy=%.2f m/sec" % (vel_y),"; Vz=%.2f m/sec" % (vel_z),"; yaw_rate=%.2f rad/s\r" % (vel_yaw),
			sys.stdout.flush()
			set_velocity_body(vel_x, vel_y, vel_z, vel_yaw)
			bearing = add_angles(compass, target_yaw)
			target_loc = target_location(target_dist_normal, target_dx, target_dy, bearing, lat, lon, alt)  
	if locked == 1:
		balloon_position = target_loc
		position_memory.balloon_reached(balloon_position[0], balloon_position[1], balloon_position[2])
		print 'Balloon added to the list of reached'
	my_latit = lat
	my_longit = lon
	my_new_altit = alt + user_data.delta_alt
	set_position = GlobalPositionTarget()
	set_position.coordinate_frame = 6  
	set_position.latitude = my_latit
	set_position.longitude = my_longit
	set_position.altitude = my_new_altit
	set_position.type_mask = 4088 
	setpoint_position_pub.publish(set_position)
	time.sleep(0.5)
	print 'Changing Altitude'
	while alt < my_new_altit*0.85 and mode == 'GUIDED':
		continue
	if mode == 'ALT_HOLD':
		Clear_Mission()
		print 'Stop VideoGet and VideoShow' 
		rec_and_show.stop()	
		rospy.signal_shutdown('Quit for Alt_hold')
		os.system('rosnode kill mavros')
		sis.stop()
	elif mode == 'GUIDED' and alt >= my_new_altit*0.85:
		print 'New Altitude Reached'
		return 'finished'
	else:
		return 'failed'

@smach.cb_interface(input_keys=['target_dx','target_dy','errX_pix','errY_pix','err_threshold','Zrate','Yawrate'], output_keys=['target_loc'], outcomes=['finished','auto','failed'])
def yawing_cb(user_data):
	rospy.loginfo('Looking for target')
	(locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err) = rec_and_show.ballonchecker()
	position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
	Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
	compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback) 
	alt_start = alt
	compass_start = compass
	alt_target = alt_start-user_data.target_dy
	rospy.sleep(0.5) 
	print compass 
	print alt
	print alt_target
	# Decisione manovra	
	print "target_dx=%.2f m" % (user_data.target_dx),"; target_dy=%.2f m" % (user_data.target_dy)
	if user_data.target_dx >= 0:
		sign_yaw = -1
	elif user_data.target_dx < 0:
		sign_yaw = +1	
	if user_data.target_dy >= 0:
		sign_z = -1
	elif user_data.target_dy < 0:
		sign_z = +1	
	print "sign_yaw=%.2f " % (sign_yaw),"; sign_z=%.2f " % (sign_z)
	errX_pixRel = float(abs(user_data.errX_pix))/(1920/2)
	errY_pixRel = float(abs(user_data.errY_pix))/(1080/2)
	print "errX_pixRel=%.2f " % (errX_pixRel),"; errY_pixRel=%.2f " % (errY_pixRel)
	if abs(errY_pixRel-errX_pixRel) >= user_data.err_threshold:
		if errY_pixRel > errX_pixRel:
			Zrate = user_data.Zrate
			Yawrate = 0
		elif errY_pixRel < errX_pixRel:
			Zrate = 0
			Yawrate = user_data.Yawrate
	else:
		Zrate = user_data.Zrate
		Yawrate = user_data.Yawrate
	print "vel_z=%.2f " % (Zrate),"; yaw_rate=%.2f " % (Yawrate)
	# Comando
	set_target_velocity(0, 0, (sign_z*Zrate), 0, 0, (sign_yaw*Yawrate))
	while ((abs(alt-alt_start) <= 0.50) and ((180-abs(abs(compass-compass_start)-180)) <= 90)) and mode == 'GUIDED':
		(locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = rec_and_show.ballonchecker()
		if locked == 1:
			bearing = add_angles(compass, target_yaw)
			target_loc = target_location(target_dist_normal, target_dx, target_dy, bearing, lat, lon, alt)
			print 'Check balloon list'
			if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
				print 'Balloon already reached'
				locked = 0
			else:
				set_target_velocity(0, 0, 0, 0, 0, 0)
				rospy.sleep(0.5)
				break
	print compass
	print alt
	if mode == 'ALT_HOLD':
		Clear_Mission()
		print 'Stop VideoGet and VideoShow' 
		rec_and_show.stop()	
		rospy.signal_shutdown('Quit for Alt_hold')
		os.system('rosnode kill mavros')
		sis.stop()
	elif locked == 1 and mode == 'GUIDED':
		print target_loc
		user_data.target_loc = target_loc
		return 'finished'
	elif locked == 0 and mode == 'GUIDED':
		return 'auto'
	else:
		return 'failed'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def RTL_cb(user_data):
    rospy.loginfo('RTL')
    setRTLMode()
    rtl_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
    rospy.sleep(0.5)
    while mode == 'RTL' and alt >= 1:
       continue
    if mode == 'ALT_HOLD':
        Clear_Mission()
        print 'Stop VideoGet and VideoShow' 
        rec_and_show.stop()	
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill mavros')
        sis.stop()
    if mode == 'RTL' and alt < 1:
        rospy.sleep(0.5)
        print 'Mission finished!'
        print 'Stop VideoGet and VideoShow' 
        rec_and_show.stop()
        rospy.signal_shutdown('Quit')
        return 'finished'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def land_cb(user_data):
    rospy.loginfo('Landing')
    land_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback) 
    setLandMode()
    rospy.sleep(0.5)
    while mode == 'LAND' and alt >= 1:
       continue
    if mode == 'ALT_HOLD':
        print 'Stop VideoGet and VideoShow' 
        rec_and_show.stop()	
        Clear_Mission()
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill mavros')
        sis.stop()
    if mode == 'LAND' and alt < 1:
        rospy.sleep(0.5)
        rec_and_show.stop()
        rospy.signal_shutdown('Quit')
        return 'finished'

 
if __name__ == '__main__':
    path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
    config = SafeConfigParser()
    config.read(path)
  															   
    rospy.init_node('drone01_state_machine', anonymous=True)
    rospy.wait_for_service('/mavros/param/set')
    setpoint_position_pub = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
    setpoint_velocity_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    manuale_topic = rospy.Subscriber('mavros/state', State, manuale_callback)	
    setpoint_angular_velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) 																	 

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])
	   
    # Dati
    sm.userdata.height = config.getfloat('GAPL','takeoff_altitude')  # quota di take-off e rtl
    sm.userdata.auto_vel = config.getfloat('GAPL','auto_vel')        # vel nella missione AUTO [cm/s]
    sm.userdata.delta_alt = config.getint('GAPL','delta_alt')        # altitude delta for elimination the target from camera view after ACTION
    sm.userdata.k_action = config.getfloat('GAPL','k_action')        # control gain for speed error during action
    sm.userdata.vel_x_max = config.getfloat('GAPL','vel_x_max')      # max speed along x_body [m/s]
    sm.userdata.vel_x_min = config.getfloat('GAPL','vel_x_min')      # max speed along x_body [m/s]
    sm.userdata.vel_y_max = config.getfloat('GAPL','vel_y_max')      # max speed along y_body [m/s]
    sm.userdata.vel_y_min = config.getfloat('GAPL','vel_y_min')      # max speed along y_body [m/s]
    sm.userdata.v_zero = config.getfloat('GAPL','v_zero')            # experimental speed value
    dist_max = config.getint('GAPL','dist_max')                      # maximum distance for positive target recognition
    sm.userdata.dist_min = config.getfloat('GAPL','dist_min')        # minimum distance from balloon during action
    err_x_max = config.getfloat('recognition_thread','sensor_width')/(2*config.getfloat('recognition_thread','focal_length'))*dist_max   # maximum x error for positive target recognition 
    sm.userdata.kerr_velx = sm.userdata.vel_x_max/dist_max           # control gain for speed error along x_body [1/s]
    sm.userdata.kerr_vely = sm.userdata.vel_y_max/err_x_max          # control gain for speed error along y_body [1/s]
    sm.userdata.erry_min = config.getfloat('GAPL','erry_min')        # minimum distance between the centre of the picture and the relative position of the drone    
    erry_max = config.getfloat('GAPL','erry_max')                    # maximum distance between the centre of the picture and the relative position of the drone 
    Vzmax = config.getfloat('GAPL','vel_z_max')                      # max speed along z_body [m/s]
    sm.userdata.k_z = 1.4*Vzmax/(erry_max-sm.userdata.erry_min)          # control gain for speed error along z_body
    sm.userdata.yaw_rateMin = config.getfloat('GAPL','yaw_rateMin')            # Yaw Rate Min for the orienting phase 
    sm.userdata.yaw_rateMax = config.getfloat('GAPL','yaw_rateMin')*4          # Yaw Rate Max for the orienting phase   
    sm.userdata.alt_max = config.getfloat('GAPL','alt_max')          # max altitude [m]
    sm.userdata.alt_min = config.getfloat('GAPL','alt_min')          # min altitude [m]
    sm.userdata.errx_min_pix = config.getint('video_get','height')/10     # minimum distance between the centre of the picture and the heading of the drone in pixel
    sm.userdata.errx_max_pix = config.getint('video_get','width')/2       # maximum distance between the centre of the picture and the heading of the drone in pixel
    sm.userdata.err_threshold = config.getfloat('GAPL','erry_min')   # max threshold to uncouple the searching aroud the edges of the camera picture 
    sm.userdata.Hmax85 = 0.85*config.getfloat('GAPL','alt_max')          
    sm.userdata.Hmin115 = 1.15*config.getfloat('GAPL','alt_min') 
    sm.userdata.Zrate = 0.25               # Z velocity for the yawing phase 
    sm.userdata.Yawrate = math.pi/4	       # Yaw Rate for the yawing phase 

    #  Start for RecognitionThread
    # try:
    #     rec_and_show = RecognitionThread(0,config.getboolean('GAPL','show_flag'), config.getboolean('GAPL','streaming_flag'),config.getboolean('GAPL','recording_flag'))
    # except:
    #     print 'camera 1 ERROR'
    #     exit(1)
    position_memory = Position_Memory()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('drone01_server', sm, '/SM_DRONE01')
    sis.start()
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MISSION', CBState(loadMission_cb),
                               transitions={'finished': 'TAKEOFF', 'failed':'LAND'})
        smach.StateMachine.add('TAKEOFF', CBState(takeoff_cb),
                               transitions={'finished': 'MISSION_SEARCHING', 'failed':'LAND'})
        smach.StateMachine.add('MISSION_SEARCHING', CBState(auto_cb),
                               transitions={'finished': 'RTL', 'reaching': 'MISSION_REACHING', 'failed':'RTL'},
                               remapping={'target_dx': 'target_dx','target_dy': 'target_dy','target_loc': 'target_loc','errX_pix': 'errX_pix','errY_pix': 'errY_pix'})
        smach.StateMachine.add('MISSION_REACHING', CBState(reaching_cb),
                               transitions={'finished': 'MISSION_ACTION', 'GPS': 'MISSION_REACHINGgps', 'failed':'RTL'},
                               remapping={'target_dx': 'target_dx','target_dy': 'target_dy','target_loc': 'target_loc','errX_pix': 'errX_pix','errY_pix': 'errY_pix'})
        smach.StateMachine.add('MISSION_REACHINGgps', CBState(reachingGPS_cb),
                               transitions={'finished': 'MISSION_SEARCHING','reaching': 'MISSION_REACHING','failed':'RTL'},
                               remapping={'target_loc': 'target_loc'})
        smach.StateMachine.add('MISSION_ACTION', CBState(action_cb),
                               transitions={'finished': 'MISSION_SEARCHING', 'failed':'RTL'})
        smach.StateMachine.add('RTL', CBState(RTL_cb),
                              transitions={'finished': 'outcome'})
        smach.StateMachine.add('LAND', CBState(land_cb),
                              transitions={'finished': 'outcome'})

    # Execute SMACH plan
    outcome = sm.execute()
    
    rospy.spin() 
    sis.stop()
