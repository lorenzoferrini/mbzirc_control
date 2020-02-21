#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from distance_finder.msg import ObjPosVec
from scipy.spatial.transform import Rotation as R

import tf
import collections

drone_to_ball_in_drone_frame=PointStamped()

def publish_ball_position():
    global drone_to_ball_in_drone_frame
    
    (trans,rot) = listener.lookupTransform('/ball', '/map', rospy.Time.now())
    print(trans,rot)
    drone_to_ball_in_drone_frame.header.stamp = rospy.Time.now()
    drone_to_ball_in_drone_frame.point.x=1
    drone_to_ball_in_drone_frame.point.z=1
    drone_to_ball_in_drone_frame.point.y=1
    
    
def ball_broadcaster(data):
    
    y=-data.targets_pos[0].err_x_m
    z=-data.targets_pos[0].err_y_m
    x=data.targets_pos[0].dist

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'/ball',"/drone_pos")
    

def drone_broadcaster(data):
    # print(data)
    # global absolute_ball_pos_in_absolute_frame
    # rospy.loginfo("averaged_ball_publisher")
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.position.x, data.pose.position.y, data.pose.position.z),(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w),rospy.Time.now(),'/drone_pos',"map")





def translator():
    # pub_ball_absolute_pos_in_abs_frame = rospy.Publisher('/trajectory_fitting/target_pos', PointStamped, queue_size=100)
    
    sub_pos_from_vision=rospy.Subscriber("/distance_finder/target_pos", ObjPosVec, ball_broadcaster)
    
    sub_drone_pos_from_mavros=rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, drone_broadcaster)
    
    pub_abs_ball_pos_in_abs_frame=rospy.Publisher("/trajectory_fitting/target_pos",PointStamped, queue_size=100)
    

    rate = rospy.Rate(2) # 2hz
    

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Ciao2    ")
            publish_ball_position()

            # rospy.loginfo("BOH")
            # absolute_ball_pos_in_absolute_frame=absolute_ball_in_absolute_frame()
            # pub_ball_absolute_pos_in_abs_frame.publish(absolute_ball_pos_in_absolute_frame)
            # abs_ball_publisher()
            # print (absolute_ball_pos_in_absolute_frame)
        except:
            pass
        rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('translator', anonymous=True)
        translator()
    except rospy.ROSInterruptException:
        pass
    
    
    



