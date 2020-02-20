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


from std_msgs.msg import Float32, Header, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
from mavros import command, mission
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, State, WaypointReached, ParamValue
from mavros_msgs.srv import CommandLong, SetMode, WaypointPush, CommandHome, WaypointClear, ParamSet
from smach import CBState
from pymavlink import mavutil

from waypoint_mp2mavros import create_waypoint_list
from position_memory_class import Position_Memory
from ConfigParser import SafeConfigParser
from distance_finder.msg import ObjPosVec, ObjPos

def ServoParam_Set(servo_function):
   rospy.wait_for_service(mavros_name+'/param/set')
   try:
       Parameter_Set = rospy.ServiceProxy(mavros_name+'/param/set', mavros_msgs.srv.ParamSet)
       servo = ParamValue()
       servo.real = servo_function
       Parameter_Set('SERVO9_FUNCTION', servo)
   except rospy.ServiceException, e:
       print "Service call failed: %s", e

def CallbackLidar(data):
    global distLidar
    distLidar = data.data

def closeGripper(number): #1850 to open, 870 to close
    servo_Service = rospy.ServiceProxy(mavros_name +'/cmd/command', mavros_msgs.srv.CommandLong)
    rospy.Subscriber('/gripper_lidar', Float32, CallbackLidar)
    starttimeGripper = rospy.Time.now().secs
    while rospy.Time.now().secs - starttimeGripper < 3 :
        if distLidar < (12) :
            servo_activation = servo_Service(0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, number, 0, 0, 0, 0, 0)

def setGuidedMode():
   rospy.wait_for_service(mavros_name+'/set_mode')
   global isModeGuided
   while mode != 'GUIDED':
      try:
         flightModeService = rospy.ServiceProxy(mavros_name+'/set_mode', mavros_msgs.srv.SetMode)
         isModeGuided = flightModeService(custom_mode='GUIDED')
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setStabilizeMode():
   rospy.wait_for_service(mavros_name+'/set_mode')
   global isModeStabilize
   try:
       flightModeService = rospy.ServiceProxy(mavros_name+'/set_mode', mavros_msgs.srv.SetMode)
       isModeStabilize = flightModeService(custom_mode='STABILIZE')
       print isModeStabilize
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. Check that GPS is enabled", e

def setAutoMode():
   rospy.wait_for_service(mavros_name+'/set_mode')
   while mode != 'AUTO':
      try:
         flightModeService = rospy.ServiceProxy(mavros_name+'/set_mode', mavros_msgs.srv.SetMode)
         isModeAuto = flightModeService(custom_mode='AUTO')
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setRTLMode():
   rospy.wait_for_service(mavros_name+'/set_mode')
   while mode != 'RTL':
      try:
         flightModeService = rospy.ServiceProxy(mavros_name+'/set_mode', mavros_msgs.srv.SetMode)
         isModeRTL = flightModeService(custom_mode='RTL')
      except rospy.ServiceException, e:
         print "service set_mode call failed: %s. Check that GPS is enabled", e

def setArm():
   rospy.wait_for_service(mavros_name+'/cmd/arming')
   try:
       armService = rospy.ServiceProxy(mavros_name+'/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s", e

def setDisarm():
   rospy.wait_for_service(mavros_name+'/cmd/arming')
   try:
       armService = rospy.ServiceProxy(mavros_name+'/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(False)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s", e

def setTakeoffMode(TOheight):
   rospy.wait_for_service(mavros_name+'/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy(mavros_name+'/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       takeoffService(altitude = TOheight, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       time.sleep(0.2)
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s", e

def setLandMode():
   rospy.wait_for_service(mavros_name+'/cmd/land')
   try:
       landService = rospy.ServiceProxy(mavros_name+'/cmd/land', mavros_msgs.srv.CommandTOL)
       isLanding = landService()
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land ", e

def setHome():
   rospy.wait_for_service(mavros_name+'/cmd/set_home')
   try:
       setHomeService = rospy.ServiceProxy(mavros_name+'/cmd/set_home', mavros_msgs.srv.CommandHome)
       setHomeService()
       rospy.sleep(0.5)
   except rospy.ServiceException, e:
       print "Service set home call failed: %s", e

def WPs_Mission():
   rospy.wait_for_service(mavros_name+'/mission/push')
   path = os.path.join(os.path.abspath(os.path.dirname(__file__))+"/", "wp_file")
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
       MissionService = rospy.ServiceProxy(mavros_name+'/mission/push', WaypointPush)
       isMissionLoaded = MissionService(0, waypoint_list)
   except rospy.ServiceException, e:
       print "Service call failed: %s", e

def Clear_Mission():
   rospy.wait_for_service(mavros_name+'/mission/clear')
   try:
       ClearService = rospy.ServiceProxy(mavros_name+'/mission/clear', WaypointClear)
       ClearService()
   except rospy.ServiceException, e:
       print "Service call failed: %s", e

'''def Param_Set(height, auto_velocity):
   try:
       Parameter_Set = rospy.ServiceProxy('/mavros/param/set',  mavros_msgs.srv.ParamSet)
       mySpeed = ParamValue()
       mySpeed.real = auto_velocity
       Parameter_Set('WPNAV_SPEED', mySpeed)
       myRtlAlt = ParamValue()
       myRtlAlt.real = height*100
       Parameter_Set('RTL_ALT', myRtlAlt)
   except rospy.ServiceException, e:
       print "Service call failed: %s", e '''

def pose_callback(data):
    global alt
    alt = data.pose.position.z

def manuale_callback(data):
    global mode
    mode = data.mode

def balloon_callback(data):
    global vision_data
    vision_data = data #[data.header,data.targets_pos]

def balloonchecker():
    global vision_data
    if vision_data:
        if (rospy.Time.now().secs - vision_data.header.stamp.secs)<3:
            locked = 0
            err_x_pix = 0
            err_y_pix = 0
            err_x_m = 0
            err_y_m = 0
            dist = 9999
            res_w = 0
            res_h = 0
            for image_object in vision_data.targets_pos:
                if image_object.obj_class == 'primary_target' and image_object.dist < dist:
                    locked = 1
                    err_x_pix = image_object.err_x_pix
                    err_y_pix = image_object.err_y_pix
                    err_x_m = image_object.err_x_m
                    err_y_m = image_object.err_y_m
                    dist = image_object.dist
                    res_w = image_object.res_w
                    res_h = image_object.res_h
            return locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h
        else:
            vision_data = None
            return 0,0,0,0,0,0,0,0
    else:
        return 0,0,0,0,0,0,0,0

def dir_pid_callback(data):
    global dir_pid_data
    dir_pid_data=data

def dir_pid_return():
      vel_x = dir_pid_data.velocity.x
    #   vel_x = saturate(vel_x, user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
      vel_y = dir_pid_data.velocity.y
    #   vel_y = saturate(vel_y, user_data.vel_y_min, user_data.vel_y_max, user_data.v_zero)
      vel_z = dir_pid_data.velocity.z
    #   vel_z = saturation_velZ(vel_z, alt, user_data.alt_min, user_data.alt_max, err_y_m, user_data.Hmax85, user_data.Hmin115)
    #   psi_dot = (((user_data.yaw_rateMax-user_data.yaw_rateMin)/((res_w/2)-(res_h/10)))*(abs(err_x_pix)-(res_h/10)))+user_data.yaw_rateMin
      vel_yaw = dir_pid_data.yaw_rate



      return vel_y, vel_x,vel_z, vel_yaw

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
   rospy.wait_for_service(mavros_name+'/cmd/command')
   global cal_pres
   try:
       cal_pres_Service = rospy.ServiceProxy(mavros_name+'/cmd/command', mavros_msgs.srv.CommandLong)
       cal_pres = cal_pres_Service(0, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 1, 0, 0, 0, 0)
   except rospy.ServiceException, e:
       print "Service calibration failed: %s", e

def cmd_condition_yaw(yaw):
   rospy.wait_for_service(mavros_name+'/cmd/command')
   try:
       condition_yaw_Service = rospy.ServiceProxy(mavros_name+'/cmd/command', mavros_msgs.srv.CommandLong)
       condition_yaw = condition_yaw_Service(0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, 0, 1, 0, 0, 0, 0)
   except rospy.ServiceException, e:
       print "condition yaw failed: %s", e

def add_angles(ang1, ang2):
   return (ang1+ang2)%360

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
  return value

def get_bearing(location_1, location_2):
   dlat = location_2[0] - location_1[0]
   dlon = location_2[1] - location_1[1]
   return (math.degrees(math.atan2(dlon, dlat)))%360

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
   set_velocity.coordinate_frame = 8    # body NED
   set_velocity.velocity.x = v_y        # verificare che le componenti siano invertite
   set_velocity.velocity.y = v_x
   set_velocity.velocity.z = v_z
   set_velocity.yaw_rate = rate
   set_velocity.type_mask = 1479
   setpoint_velocity_pub.publish(set_velocity)
   #time.sleep(0.5)

def set_velocity_body_command(set_vel):
   setpoint_velocity_pub.publish(set_vel)
   #time.sleep(0.5)

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

def target_location(target_dist_normal, target_dx, err_y_m, bearing, my_latit, my_longit, my_altit):
   alt_min = 1
   alt_max = 5
   target_dist = math.sqrt((target_dist_normal*target_dist_normal)+(target_dx*target_dx))
   d_latit = target_dist*math.cos((bearing)*math.pi/180)/1.113195e5
   d_longit = target_dist*math.sin((bearing)*math.pi/180)/1.113195e5
   target_latit = my_latit + d_latit
   target_longit = my_longit + d_longit
   target_altit = my_altit - err_y_m
   target_altit = saturate(target_altit, alt_min, alt_max, 0)
   target_loc = [target_latit, target_longit, target_altit]
   return target_loc

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished','failed'])
def loadMission_cb(user_data):
    rospy.loginfo('mode STABILIZE')
    setStabilizeMode()
    rospy.loginfo('Set Home location to current location')
    rospy.loginfo('Load Mission')
    position_memory.wipe_memory()
    Clear_Mission()
    ServoParam_Set(0)
    # Calibrazioni
    cmd_calibrate_pressure()
    time.sleep(2)
    setArm()
    setHome()
    WPs_Mission()
    time.sleep(2)
    setDisarm()
    time.sleep(4)
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
    alt_start = alt
    setTakeoffMode(user_data.height)
    start_takeoff_time=time.time()
    while mode == 'GUIDED' and alt < (user_data.height*0.85):
           t = time.time()-start_takeoff_time
           if t >=10 and alt == alt_start:
              break
    if mode == 'ALT_HOLD':
        Clear_Mission()
        ServoParam_Set(58) #set servo to manual

        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill '+ mavros_name)
        sis.stop()
    elif alt <= (user_data.height*1.15) and alt > (user_data.height*0.85) and mode == 'GUIDED':
         return 'finished'
    else:
         return 'failed'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished','reaching','failed'])
def auto_cb(user_data):
    rospy.loginfo('Mission Searching')
    setAutoMode()
    locked = 0
    deltaTime = 9999
    #waypoint_topic = rospy.Subscriber('mavros/mission/reached', WaypointReached, WP_reached_callback)
    #position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
    #Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    #compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
    #rospy.sleep(0.5)
    while deltaTime > 30:
        if mode == 'ALT_HOLD' or locked == 1:
            global CurrentWaypoint
            CurrentWaypoint = 0
            break
        try:
            waypointMsg = rospy.wait_for_message(mavros_name+'/mission/reached', WaypointReached,0.1)
            timeReachedWaypoint = waypointMsg.header.stamp.secs
            deltaTime = rospy.Time.now().secs-timeReachedWaypoint

        except rospy.exceptions.ROSException:
            deltaTime = 9999
            pass
        (locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, res_w, res_h) = balloonchecker()
    # print CurrentWaypoint
    # print total_waypoints

    # giro sul primo waypoint
    setGuidedMode()
    t = time.time()
    elapsed=0
    print("Setting time to 0")
    while(elapsed<6):
        elapsed = time.time() - t
        print(elapsed)
        set_velocity_body(0,0,0,1)
        (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
        if (locked==1):
            break
            print("Second 180")
            print("Sleeping")
            print( "Waypoint reached")
    setAutoMode()

    previous_wp=CurrentWaypoint
    while CurrentWaypoint < total_waypoints and mode == 'AUTO' and locked == 0:
        (locked, err_x_pix, err_y_pix, target_dx, target_dy, target_dist_normal, target_yaw, theta_err) = balloonchecker()

        if previous_wp != CurrentWaypoint:

            setGuidedMode()
            t = time.time()
            elapsed=0
            print("Setting time to 0")
            while(elapsed<6):
                elapsed = time.time() - t
                print(elapsed)
                set_velocity_body(0,0,0,1)
                (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
                if (locked==1):
                    break
                print("Second 180")
                print("Sleeping")
                print( "Waypoint reached")
            setAutoMode()
            previous_wp=previous_wp+1



            # if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
            #     print 'Balloon already reached'
            #     locked = 0
            # else:
            #     break
    if mode == 'ALT_HOLD':
        Clear_Mission()
        print 'Stop VideoGet and VideoShow'
        # rec_and_show.stop()
        ServoParam_Set(58) #set servo to manual
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill '+ mavros_name)
        sis.stop()
    elif CurrentWaypoint == total_waypoints:
        Clear_Mission()
        return 'finished'
    elif locked == 1:
        return 'reaching'
    else:
        return 'failed'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['failed','searching'])
def reaching_cb(user_data):
    rospy.loginfo('mode GUIDED')
    setGuidedMode()
    rospy.loginfo('Balloon found')

    rospy.loginfo('Mission Reaching')
    (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
    #position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
    #Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    #compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
    #rospy.sleep(0.5)
    if locked == 1:
        vel_x,vel_y,vel_z,yaw_rate=dir_pid_return()
        set_velocity_body(vel_x,vel_y,vel_z,yaw_rate)

    while locked == 1 and mode == 'GUIDED':
        last_pos = [err_x_pix, err_y_pix, dist]
        vel_x,vel_y,vel_z,yaw_rate=dir_pid_return()
        set_velocity_body(vel_x,vel_y,vel_z,yaw_rate)
        (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
        last_time_locked = rospy.get_time()

        if distLidar<12: 
            #CHIUSURA:1850, APERTURA: 870
            servo_activation = servo_Service(0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 1850, 0, 0, 0, 0, 0) #chiudi
            rospy.loginfo('gripper chiuso causa lidar')
            #VERIFICARE SE HO LA PALLA O NO
            rospy.sleep(0.5)
            if distLidar<15:balloonchecker
                rospy.loginfo('palloncino CATTURATO')
                return 'failed'
            else:
                rospy.loginfo('palloncino LOST')
                servo_activation = servo_Service(0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 870, 0, 0, 0, 0, 0) #apri


    if mode == 'ALT_HOLD': #switched to manual mode
        Clear_Mission()
        ServoParam_Set(58) #set servo to manual
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill '+ mavros_name)
        sis.stop()
    elif locked == 0: #short term recovery
        rospy.loginfo("short term recovery")
        last_velocity = PositionTarget()
        last_velocity = dir_pid_data
        vel_cmd=last_velocity
        while rospy.Time.from_sec(last_time_locked) < 2 and mode=='GUIDED':
            #gripper checking part
            if distLidar<12: 
                #CHIUSURA:1850, APERTURA: 870
                servo_activation = servo_Service(0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 1850, 0, 0, 0, 0, 0) #chiudi
                rospy.loginfo('str:trovato oggetto vicino')
                #VERIFICARE SE HO LA PALLA O NO
                rospy.sleep(0.5)
                if distLidar<15:balloonchecker
                    rospy.loginfo('str:palloncino CATTURATO')
                    return 'failed'
                else:
                    rospy.loginfo('str:palloncino LOST')
                    servo_activation = servo_Service(0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 870, 0, 0, 0, 0, 0) #apri
            #command decayng part        
            vel_cmd.velocity.x = last_velocity.velocity.x * (2 - rospy.Time.from_sec(last_time_locked))
            vel_cmd.velocity.y = last_velocity.velocity.y * (2 - rospy.Time.from_sec(last_time_locked))
            vel_cmd.velocity.z = last_velocity.velocity.z * (2 - rospy.Time.from_sec(last_time_locked))
            vel_cmd.yaw_rate = last_velocity.yaw_rate * (2 - rospy.Time.from_sec(last_time_locked))
            set_velocity_body_command(vel_cmd)
         return 'searching'
    else:
         return 'failed'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def RTL_cb(user_data):
    rospy.loginfo('RTL')
    setRTLMode()
    #rtl_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    #rospy.sleep(0.5)
    while mode == 'RTL' and alt >= 1:
       continue
    if mode == 'ALT_HOLD':
        Clear_Mission()
        ServoParam_Set(58) #set servo to manual
        rospy.signal_shutdown('Quit for Alt_hold')
        os.system('rosnode kill '+ mavros_name)
        sis.stop()
    if mode == 'RTL' and alt < 1:
        rospy.sleep(0.5)
        print 'Mission finished!'
        rospy.signal_shutdown('Quit')
        return 'finished'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def land_cb(user_data):
    rospy.loginfo('Landing')
    #land_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    setLandMode()
    rospy.sleep(0.5)
    while mode == 'LAND' and alt >= 1:
       continue
    if mode == 'ALT_HOLD':
        Clear_Mission()
        rospy.signal_shutdown('Quit for Alt_hold')
        ServoParam_Set(58) #set servo to manual

        os.system('rosnode kill '+ mavros_name)
        sis.stop()
    if mode == 'LAND' and alt < 1:
        rospy.sleep(0.5)
        rospy.signal_shutdown('Quit')
        return 'finished'


if __name__ == '__main__':
    path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
    config = SafeConfigParser()
    config.read(path)

    mavros_name = 'mavros'
    rospy.init_node('drone_state_machine', anonymous=True)
    #rospy.wait_for_service('/mavros/param/set')
    #subs
    manuale_topic = rospy.Subscriber(mavros_name+'/state', State, manuale_callback)
    balloon_topic = rospy.Subscriber('/distance_finder/target_pos', ObjPosVec, balloon_callback)
    waypoint_topic = rospy.Subscriber(mavros_name+'/mission/reached', WaypointReached, WP_reached_callback)
    compass_topic = rospy.Subscriber(mavros_name+'/global_position/compass_hdg', Float64, compass_callback)
    position_topic = rospy.Subscriber(mavros_name+'/global_position/global', NavSatFix, home_callback)
    Alt_topic = rospy.Subscriber(mavros_name+'/local_position/pose', PoseStamped, pose_callback)
    setpoint_velocity_sub= rospy.Subscriber(mavros_name+'/setpoint_raw/local2',PositionTarget, dir_pid_callback)
    #publs
    setpoint_position_pub = rospy.Publisher(mavros_name+'/setpoint_raw/global', GlobalPositionTarget, queue_size=2)
    setpoint_velocity_pub = rospy.Publisher(mavros_name+'/setpoint_raw/local', PositionTarget, queue_size=2)
    setpoint_angular_velocity_pub = rospy.Publisher(mavros_name+'/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.sleep(0.3)
    #gripper
    servo_Service = rospy.ServiceProxy(mavros_name +'/cmd/command', mavros_msgs.srv.CommandLong)
    rospy.Subscriber('/gripper_lidar', Float32, CallbackLidar)


    # gripper_topic= rospy.Subscriber('/gripper', Bool, gripper_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])

    # Dati
    sm.userdata.height = config.getfloat('GAPL','takeoff_altitude')  # quota di take-off e rtl

    #  Start for Position Memory
    position_memory = Position_Memory()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('drone_server', sm, '/SM_DRONE')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MISSION', CBState(loadMission_cb),
                               transitions={'finished': 'TAKEOFF', 'failed':'LAND'})
        smach.StateMachine.add('TAKEOFF', CBState(takeoff_cb),
                               transitions={'finished': 'MISSION_SEARCHING', 'failed':'LAND'})
        smach.StateMachine.add('MISSION_SEARCHING', CBState(auto_cb),
                               transitions={'finished': 'RTL', 'reaching': 'MISSION_REACHING', 'failed':'RTL'})
                            #    remapping={'target_dx': 'target_dx','target_dy': 'target_dy','target_loc': 'target_loc','errX_pix': 'errX_pix','errY_pix': 'errY_pix'})
        smach.StateMachine.add('MISSION_REACHING', CBState(reaching_cb),
                               transitions={'failed':'RTL', 'searching': 'MISSION_SEARCHING'})
                            #    remapping={'target_dx': 'target_dx','target_dy': 'target_dy','target_loc': 'target_loc','errX_pix': 'errX_pix','errY_pix': 'errY_pix'})
        # smach.StateMachine.add('MISSION_REACHINGgps', CBState(reachingGPS_cb),
        #                        transitions={'finished': 'MISSION_SEARCHING','reaching': 'MISSION_REACHING','failed':'RTL'},
                            #    remapping={'target_loc': 'target_loc'})
        # smach.StateMachine.add('MISSION_ACTION', CBState(action_cb),
                            #    transitions={'finished': 'MISSION_SEARCHING', 'failed':'RTL'})
        smach.StateMachine.add('RTL', CBState(RTL_cb),
                              transitions={'finished': 'outcome'})
        smach.StateMachine.add('LAND', CBState(land_cb),
                              transitions={'finished': 'outcome'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()




# @smach.cb_interface(input_keys=[], output_keys=['target_dx','target_dy','target_loc','errX_pix','errY_pix'], outcomes=['finished','reaching','failed'])
# def auto_cb(user_data):
#     rospy.loginfo('Mission Searching')
#     setAutoMode()
#     locked = 0
#     deltaTime = 9999
#     waypoint_topic = rospy.Subscriber('mavros/mission/reached', WaypointReached, WP_reached_callback)
#     position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
#     Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
#     compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
#     rospy.sleep(0.5)
#     while deltaTime > 30:
#         if mode == 'ALT_HOLD' or locked == 1:
#             global CurrentWaypoint
#             CurrentWaypoint = 0
#             break
#         try:
#             waypointMsg = rospy.wait_for_message('mavros/mission/reached', WaypointReached,0.1)
#             timeReachedWaypoint = waypointMsg.header.stamp.secs
#             deltaTime = rospy.Time.now().secs-timeReachedWaypoint
#             (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#             if locked == 1:
#                 target_yaw = math.degrees((err_x_m/dist))
#                 bearing = add_angles(compass, target_yaw)
#                 target_loc = target_location(dist, err_x_m, err_y_m, bearing, lat, lon, alt)
#                 err_dx = err_x_m
#                 err_dy = err_y_m
#                 errX_pix = err_x_pix
#                 errY_pix = err_y_pix
#                 print '1Check balloon list'
#                 if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
#                     print '1Balloon already reached'
#                     locked = 0
#         except rospy.exceptions.ROSException:
#             deltaTime = 9999
#             locked = 0
#     print CurrentWaypoint
#     print total_waypoints
#     while CurrentWaypoint < total_waypoints and mode == 'AUTO' and locked == 0:
#         (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#         if locked == 1:
#             target_yaw = math.degrees((err_x_m/dist))
#             bearing = add_angles(compass, target_yaw)
#             target_loc = target_location(dist, err_x_m, err_y_m, bearing, lat, lon, alt)
#             err_dx = err_x_m
#             err_dy = err_y_m
#             errX_pix = err_x_pix
#             errY_pix = err_y_pix
#             print 'Check balloon list'
#             print position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2])
#             if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
#                 print 'Balloon already reached'
#                 locked = 0
#             else:
#                 break
#     if mode == 'ALT_HOLD':
#         Clear_Mission()
#         rospy.signal_shutdown('Quit for Alt_hold')
#         os.system('rosnode kill mavros')
#         sis.stop()
#     elif CurrentWaypoint == total_waypoints:
#         Clear_Mission()
#         return 'finished'
#     elif locked == 1:
#         user_data.target_loc = target_loc
#         user_data.target_dx = err_dx
#         user_data.target_dy = err_dy
#         user_data.errX_pix = errX_pix
#         user_data.errY_pix = errY_pix
#         return 'reaching'
#     else:
#         return 'failed'
# @smach.cb_interface(input_keys=['kerr_velx','dist_min','vel_x_min','vel_x_max','v_zero','kerr_vely','vel_y_min','vel_y_max','target_loc'], output_keys=[], outcomes=['finished','reaching','failed'])
# def reachingGPS_cb(user_data):



# @smach.cb_interface(input_keys=['Hmax85','Hmin115','finalAlt','k_action','dist_min','vel_x_min','vel_x_max','v_zero','kerr_vely','vel_y_min','vel_y_max','delta_alt','yaw_rateMax','yaw_rateMin','erry_min','k_z','alt_min','alt_max'], output_keys=[], outcomes=['finished','failed'])
# def action_cb(user_data):
#     rospy.loginfo('Mission Action')
#     position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
#     Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
#     compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
#     (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#     rospy.sleep(0.5)
#     time_ini = rospy.Time.now().secs
#     print 'Action: count'
#     while (rospy.Time.now().secs-time_ini) < config.getint('GAPL','action_time') and mode == 'GUIDED':
#         (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#         if locked == 1:
#             vel_x = user_data.k_action*(dist-user_data.dist_min)
#             vel_x = saturate(vel_x, user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
#             vel_y = user_data.kerr_vely*err_x_m
#             vel_y = saturate(vel_y, user_data.vel_y_min, user_data.vel_y_max, user_data.v_zero)
#             vel_z = -user_data.k_z*err_y_m
#             vel_z = saturation_velZ(vel_z, alt, user_data.alt_min, user_data.alt_max, err_y_m, user_data.Hmax85, user_data.Hmin115)
#             psi_dot = (((user_data.yaw_rateMax-user_data.yaw_rateMin)/((res_w/2)-(res_h/10)))*(abs(err_x_pix)-(res_h/10)))+user_data.yaw_rateMin
#             vel_yaw = yaw_alignment(err_x_pix, res_h, psi_dot)
#             print "Vx=%.2f m/sec" % (vel_x),"; Vy=%.2f m/sec" % (vel_y),"; Vz=%.2f m/sec" % (vel_z),"; yaw_rate=%.2f rad/s\r" % (vel_yaw),
#             sys.stdout.flush()
#             set_velocity_body(vel_x, vel_y, vel_z, vel_yaw)
#             target_yaw = math.degrees((err_x_m/dist))
#             bearing = add_angles(compass, target_yaw)
#             target_loc = target_location(dist, err_x_m, err_y_m, bearing, lat, lon, alt)
#     if locked == 1:
#         balloon_position = target_loc
#         position_memory.balloon_reached(balloon_position[0], balloon_position[1], balloon_position[2])
#         print 'Balloon added to the list of reached'
#     my_latit = lat
#     my_longit = lon
#     #my_new_altit = alt + user_data.delta_alt
#     my_new_altit = user_data.finalAlt
#     set_position = GlobalPositionTarget()
#     set_position.coordinate_frame = 6
#     set_position.latitude = my_latit
#     set_position.longitude = my_longit
#     set_position.altitude = my_new_altit
#     set_position.type_mask = 4088
#     setpoint_position_pub.publish(set_position)
#     time.sleep(0.5)
#     print 'Changing Altitude'
#     while alt < my_new_altit*0.85 and mode == 'GUIDED':
#         continue
#     if mode == 'ALT_HOLD':
#         Clear_Mission()
#         rospy.signal_shutdown('Quit for Alt_hold')
#         os.system('rosnode kill mavros')
#         sis.stop()
#     elif mode == 'GUIDED' and alt >= my_new_altit*0.85:
#         print 'New Altitude Reached'
#         return 'finished'
#     else:
#         return 'failed'




# @smach.cb_interface(input_keys=['target_dx','target_dy','errX_pix','errY_pix','err_threshold','Zrate','Yawrate'], output_keys=['target_loc'], outcomes=['finished','auto','failed'])
# def yawing_cb(user_data):
#     rospy.loginfo('Looking for target')
#     (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#     position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
#     Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
#     compass_topic = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback)
#     alt_start = alt
#     compass_start = compass
#     alt_target = alt_start-user_data.target_dy
#     rospy.sleep(0.5)
#     # Decisione manovra
#     print "target_dx=%.2f m" % (user_data.target_dx),"; target_dy=%.2f m" % (user_data.target_dy)
#     if user_data.target_dx >= 0:
#         sign_yaw = -1
#     elif user_data.target_dx < 0:
#         sign_yaw = +1
#     if user_data.target_dy >= 0:
#         sign_z = -1
#     elif user_data.target_dy < 0:
#         sign_z = +1
#     print "sign_yaw=%.2f " % (sign_yaw),"; sign_z=%.2f " % (sign_z)
#     errX_pixRel = float(abs(user_data.errX_pix))/(1920/2)
#     errY_pixRel = float(abs(user_data.errY_pix))/(1080/2)
#     print "errX_pixRel=%.2f " % (errX_pixRel),"; errY_pixRel=%.2f " % (errY_pixRel)
#     if abs(errY_pixRel-errX_pixRel) >= user_data.err_threshold:
#         if errY_pixRel > errX_pixRel:
#             Zrate = user_data.Zrate
#             Yawrate = 0
#         elif errY_pixRel < errX_pixRel:
#             Zrate = 0
#             Yawrate = user_data.Yawrate
#     else:
#         Zrate = user_data.Zrate
#         Yawrate = user_data.Yawrate
#     print "vel_z=%.2f " % (Zrate),"; yaw_rate=%.2f " % (Yawrate)
#     # Comando
#     set_target_velocity(0, 0, (sign_z*Zrate), 0, 0, (sign_yaw*Yawrate))
#     while ((abs(alt-alt_start) <= 0.50) and ((180-abs(abs(compass-compass_start)-180)) <= 90)) and mode == 'GUIDED':
#         (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#         if locked == 1:
#             target_yaw = math.degrees((err_x_m/dist))
#             bearing = add_angles(compass, target_yaw)
#             target_loc = target_location(dist, err_x_m, err_y_m, bearing, lat, lon, alt)
#             print 'Check balloon list'
#             if position_memory.check_ballon_already_reached(target_loc[0], target_loc[1], target_loc[2]):
#                 print 'Balloon already reached'
#                 locked = 0
#             else:
#                 set_target_velocity(0, 0, 0, 0, 0, 0)
#                 rospy.sleep(0.5)
#                 break
#     if mode == 'ALT_HOLD':
#         Clear_Mission()
#         rospy.signal_shutdown('Quit for Alt_hold')
#         os.system('rosnode kill mavros')
#         sis.stop()
#     elif locked == 1 and mode == 'GUIDED':
#         user_data.target_loc = target_loc
#         return 'finished'
#     elif locked == 0 and mode == 'GUIDED':
#         return 'auto'
#     else:
#         return 'failed'



#  rospy.loginfo('Mission Reaching GPS')
#     (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#     position_topic = rospy.Subscriber('/mavros/global_position/global', NavSatFix, home_callback)
#     Alt_topic = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
#     rospy.sleep(0.5)
#     my_latit = lat
#     my_longit = lon
#     my_altit = alt
#     my_loc = [my_latit, my_longit, my_altit]
#     target_loc = user_data.target_loc
#     if locked == 0:
#         target_myloc_bearing = get_bearing(my_loc,target_loc)
#         cmd_condition_yaw(target_myloc_bearing)
#         rospy.sleep(0.5)
#         dist_target_myloc = get_distance_meters(my_loc,target_loc)
#         V = user_data.kerr_velx*(dist_target_myloc-user_data.dist_min)
#         V = saturate(V,user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
#         V_North = V*math.cos(math.radians(target_myloc_bearing))
#         V_East = V*math.sin(math.radians(target_myloc_bearing))
#         send_ned_velocity(V_North, V_East, 0)
#         while V > 0 and mode == 'GUIDED':
#             my_loc = [lat, lon, alt]
#             target_myloc_bearing = get_bearing(my_loc,target_loc)
#             cmd_condition_yaw(target_myloc_bearing)
#             rospy.sleep(0.5)
#             dist_target_myloc = get_distance_meters(my_loc,target_loc)
#             V = user_data.kerr_velx*(dist_target_myloc-user_data.dist_min)
#             V = saturate(V,user_data.vel_x_min, user_data.vel_x_max, user_data.v_zero)
#             V_North = V*math.cos(math.radians(target_myloc_bearing))
#             V_East = V*math.sin(math.radians(target_myloc_bearing))
#             send_ned_velocity(V_North, V_East, 0)
#             (locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, res_w ,res_h) = balloonchecker()
#             if locked == 1:
#                 break
#     if mode == 'ALT_HOLD':
#         Clear_Mission()
#         rospy.signal_shutdown('Quit for Alt_hold')
#         os.system('rosnode kill mavros')
#         sis.stop()
#     elif locked == 1 and mode == 'GUIDED':
#         return 'reaching'
#     elif V <= 0 and mode == 'GUIDED' and locked == 0:
#         print 'GPS reached! Balloon not found'
#         return 'finished'
#     else:
#         return 'failed'
