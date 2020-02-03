#!/usr/bin/env python

import math
import rospy
import smach
import smach_ros
import time
import threading

from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from mbzirc_mission_management.msg import Task
from time import sleep

sm = smach.StateMachine(outcomes=['DONE'])
mavros_state = State()
mavros_odometry = Odometry()
guided = False

sub_task_pub = rospy.Publisher('sub_task', Task, queue_size=1, latch=True)
task_id_pub = rospy.Publisher('/smach/task_id', String, queue_size = 1)

next_task_task_type = ""
next_task_reference = []

reset_requested = False


class SmInitialisation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['initialised']
                             )

    def execute(self, userdata):
        sub_task_msg = Task()
        sub_task_msg.task_type = "IDLE"
        sub_task_msg.reference = []
        sub_task_pub.publish(sub_task_msg)
        return 'initialised'

class GuidedModeSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['guided_requested']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Check if GPS is enabled"%e
        return 'guided_requested'


class ArmSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['arm_requested']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arm call failed: %s"%e
        return 'arm_requested'


class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['take_off_requested']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude = 3, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e
        return 'take_off_requested'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['land_requested']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "service land call failed: %s. The vehicle cannot land "%e
        return 'land_requested'

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_requested']
                             )

    def execute(self, userdata):
        task_id = String()
        task_id.data = "SEARCH"
        task_id_pub.publish(task_id)
        return 'search_requested'

class ChaseRQ(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['chase_requested']
                             )

    def execute(self, userdata):
        task_id = String()
        task_id.data = "CHASE"
        task_id_pub = rospy.Publisher('/smach/task_id', String, queue_size = 1)
        task_id_pub.publish(task_id)
        return 'chase_requested'


class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['aborted']
                             )

    def execute(self, userdata):
        return 'aborted'




def task_cb(ud, msg):
    global sm
    sm.userdata.next_task_task_type = msg.task_type
    sm.userdata.next_task_reference = msg.reference
    return False

def MavrosGuidedCallback(ud, msg):
    global sm
    sm.userdata.guided = msg.guided
    print "Guided: %r"%sm.userdata.guided
    # return sm.userdata.guided
    if msg.guided:
        return False
    else:
        return True

def MavrosArmedCallback(ud, msg):
    global sm
    sm.userdata.armed = msg.armed
    if msg.armed or rospy.is_shutdown():
        return False
    else:
        return True

def MavrosTakeOffCallback(ud, msg):
    global sm
    sm.userdata.height = msg.pose.pose.position.z
    if sm.userdata.height > 2.8  or rospy.is_shutdown():
        return False
    else:
        return True

def MavrosLandCallback(ud, msg):
    global sm
    sm.userdata.height = msg.pose.pose.position.z
    if sm.userdata.height < 0.1  or rospy.is_shutdown():
        return False
    else:
        return True

def SearchCallback(ud, msg):
    global sm
    sm.userdata.targetlocked = False;
    sm.userdata.targetlocked = msg.data
    if sm.userdata.targetlocked  or rospy.is_shutdown():
        return False
    else:
        return True

def ChaseCallback(ud, msg):
    global sm
    sm.userdata.targetlocked = False;
    sm.userdata.targetlocked = msg.data
    if sm.userdata.targetlocked :
        return True
    else:
        return False


def switching_condition_cb( msg):
    global reset_requested
    if msg.data == "SWITCH":
        return False
    elif msg.data == "RESET":
        rospy.logwarn("RESET is requested")
        reset_requested = True
        return False
    else:
        rospy.logwarn("Switching condition UNKNOWN: RESET is requested")
        rospy.logwarn("Switching condition is")
        rospy.logwarn(msg.data)
        reset_requested = True
        return False


def main():
    global sm

    rospy.init_node("task_manager")


    sm.set_initial_state(['INITIALISATION'])

    with sm:

        sm.add('INITIALISATION', SmInitialisation(),
                         transitions = {'initialised':'IDLE'}
                         )

        sm.add('IDLE', smach_ros.MonitorState("/task", Task, task_cb),
                               transitions={'invalid':'GUIDED_MODE_RQ',
                                            'valid':'IDLE',
                                            'preempted':'IDLE'})


        sm.add('GUIDED_MODE_RQ',GuidedModeSwitch(),
                            transitions = {'guided_requested' : 'GUIDED_MODE_CHECK'}
                            )


        sm.add('GUIDED_MODE_CHECK', smach_ros.MonitorState("/mavros/state", State, MavrosGuidedCallback),
                                    transitions={'invalid':'ARM_RQ',
                                                 'valid':'GUIDED_MODE_CHECK',
                                                 'preempted':'GUIDED_MODE_CHECK'})

        sm.add('ARM_RQ',ArmSwitch(),
                            transitions = {'arm_requested' : 'ARM_CHECK'}
                            )


        sm.add('ARM_CHECK', smach_ros.MonitorState("/mavros/state", State, MavrosArmedCallback),
                                    transitions={'invalid':'TAKEOFF_RQ',
                                                 'valid':'ARM_CHECK',
                                                 'preempted':'ARM_CHECK'})

        sm.add('TAKEOFF_RQ',TakeOff(),
                            transitions = {'take_off_requested' : 'TAKEOFF_CHECK'}
                            )


        sm.add('TAKEOFF_CHECK', smach_ros.MonitorState("/mavros/local_position/odom", Odometry, MavrosTakeOffCallback),
                                    transitions={'invalid':'SEARCH_RQ',
                                                 'valid':'TAKEOFF_CHECK',
                                                 'preempted':'TAKEOFF_CHECK'})

        sm.add('LAND_RQ',Land(),
                            transitions = {'land_requested' : 'LAND_CHECK'}
                            )


        sm.add('LAND_CHECK', smach_ros.MonitorState("/mavros/local_position/odom", Odometry, MavrosLandCallback),
                                    transitions={'invalid':'IDLE',
                                                 'valid':'LAND_CHECK',
                                                 'preempted':'LAND_CHECK'})


        sm.add('SEARCH_RQ',Search(),
                            transitions = {'search_requested' : 'SEARCHING'}
                            )


        sm.add('SEARCHING', smach_ros.MonitorState("/target_found", Bool, SearchCallback),
                                    transitions={'invalid':'CHASE_RQ',
                                                 'valid':'SEARCHING',
                                                 'preempted':'SEARCHING'})

        sm.add('CHASE_RQ',ChaseRQ(),
                            transitions = {'chase_requested' : 'CHASING'}
                            )

        sm.add('CHASING', smach_ros.MonitorState("/target_found", Bool, ChaseCallback),
                                    transitions={'invalid':'SEARCH_RQ',
                                                 'valid':'CHASING',
                                                 'preempted':'CHASING'})

        # sm.add('CATCH',Abort(),
        #                     transitions = {'aborted' : 'SEARCHING'}
        #                     )



    #sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    #sis.start()
    sm.execute()
    rospy.spin()
    #sis.stop()

if __name__=="__main__":
    main()
