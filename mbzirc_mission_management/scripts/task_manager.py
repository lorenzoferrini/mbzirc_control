#!/usr/bin/env python

import math

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import String


sm = smach.StateMachine(outcomes=['DONE'])


sub_task_pub = rospy.Publisher('sub_task', Task, queue_size=1, latch=True)

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
                             outcomes=['guided']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
       return 'guided'
   except rospy.ServiceException, e:
       print "service set_mode call failed: %s. Check if GPS is enabled"%e


class ArmSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['guided']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
       return 'armed'
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e


class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['guided']
                             )

    def execute(self, userdata):
        rospy.wait_for_service('/mavros/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       return 'taken_off'
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s"%e



def task_cb(ud, msg):
    global sm
    sm.userdata.next_task_task_type = msg.task_type
    sm.userdata.next_task_reference = msg.reference
    return False

def switching_condition_cb(ud, msg):
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
                               transitions={'invalid':'GUIDED_MODE',
                                            'valid':'IDLE',
                                            'preempted':'IDLE'})


        sm.add('GUIDED_MODE',GuidedModeSwitch(),
                            transitions = {'guided' : 'ARM'}
                            )

        sm.add('ARM',ArmSwitch(),
                            transitions = {'armed' : 'TAKEOFF'}
                            )

        sm.add('TAKEOFF',TakeOff(),
                            transitions = {'taken_off' : 'SEARCH'}
                            )


    #sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    #sis.start()
    sm.execute()
    rospy.spin()
    #sis.stop()

if __name__=="__main__":
    main()
