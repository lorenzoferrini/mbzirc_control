#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from distance_finder.msg import ObjPosVec
from scipy.spatial.transform import Rotation as R

import tf
import collections

# # FRAMES
# # 1) DRONE FRAME
# # 2) ABSOLUTE FRAME

# # POSITIONS

# # 1) DRONE POSITION IN FIXED FRAME
# drone_pose_in_absolute_frame=PoseStamped()

# # 2) RELATIVE BALL POSITION IN DRONE FRAME
# drone_to_ball_in_drone_frame=PointStamped()

# # 3) RELATIVE BALL POSITION IN ABSOLUTE FRAME
# drone_to_ball_in_absolute_frame=ObjPosVec()

# # 4+) ABSOLUTE BALL POSITION IN ABSOLUTE FRAME
# absolute_ball_pos_in_absolute_frame=PointStamped()

# # 5) AVERAGED BALL POSITION
# # averaged_drone_to_ball_in_drone_frame

# drone_to_ball_in_drone_frame_stack= collections.deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)


# # VISION MSG TO 


# def absolute_ball_in_absolute_frame(data):

#     global drone_to_ball_in_absolute_frame
#     global drone_to_ball_in_drone_frame

#     global drone_pose_in_absolute_frame
    
#     global absolute_ball_pos_in_absolute_frame
    
#     """ 
#     input:
     
#     data:  drone_pos (PoseStamped)

#     output:
    
#     absolute_ball_pos_in_abs_frame
    
#     """
    
#     drone_pose_in_absolute_frame=data
    
#     # Rotazione ottenuta dai quaternioni della pose del drone, da terna drone a terna globale
#     r = R.from_quat([drone_pose_in_absolute_frame.pose.orientation.x, drone_pose_in_absolute_frame.pose.orientation.y, drone_pose_in_absolute_frame.pose.orientation.z, drone_pose_in_absolute_frame.pose.orientation.w])
#     r=r.inv()
    
#     # Posizione del palloncino
#     np_drone_to_ball_in_drone_frame=np.array([drone_to_ball_in_drone_frame.point.x,drone_to_ball_in_drone_frame.point.y,drone_to_ball_in_drone_frame.point.z])
    
#     # Posizione del drone
#     np_drone_pose_in_absolute_frame=np.array([drone_pose_in_absolute_frame.pose.position.x,drone_pose_in_absolute_frame.pose.position.y,drone_pose_in_absolute_frame.pose.position.z])

#     # Posizione relativa drone palloncino nel frame globale
#     np_drone_to_ball_in_absolute_frame=r.apply(np_drone_to_ball_in_drone_frame)
    
#     # Posizione del palloncino assoluta nel frame globale
#     np_ball_position_in_global_frame=np_drone_to_ball_in_absolute_frame+np_drone_pose_in_absolute_frame
    
#     absolute_ball_pos_in_absolute_frame.header.stamp = rospy.Time.now()
#     absolute_ball_pos_in_absolute_frame.point.x=np_ball_position_in_global_frame[0]
#     absolute_ball_pos_in_absolute_frame.point.y=np_ball_position_in_global_frame[1]
#     absolute_ball_pos_in_absolute_frame.point.z=np_ball_position_in_global_frame[2]
    
#     # rospy.loginfo("abs_pose")


# def from_vision_msg_to_drone_pose_in_global_frame(data):
    
#     global drone_to_ball_in_drone_frame
#     global averaged_drone_to_ball_in_drone_frame
#     global drone_to_ball_in_drone_frame_stack
    
#     """ 
#     input:
    
#     data:  drone_to_ball_in_drone_frame (ObjPosVec)
    
#     output:
    
#     global drone_to_ball_in_drone_frame  (PointStamped)
    
#     """

#     drone_to_ball_in_drone_frame.header.stamp = rospy.Time.now()
#     drone_to_ball_in_drone_frame.point.x=data.targets_pos[0].err_x_m
#     drone_to_ball_in_drone_frame.point.z=-data.targets_pos[0].err_y_m
#     drone_to_ball_in_drone_frame.point.y=data.targets_pos[0].dist
    
#     # drone_to_ball_in_drone_frame_stack.popleft()
#     # drone_to_ball_in_drone_frame_stack.append(drone_to_ball_in_drone_frame.point.z)
    
#     # print(drone_to_ball_in_drone_frame.point.z)
        
#     # averaged_drone_to_ball_in_drone_frame=reject_outliers(drone_to_ball_in_drone_frame_stack)
    
#     # rospy.loginfo("from_vision_to_pos")

    
def ball_broadcaster(data):
    
    y=-data.targets_pos[0].err_x_m
    z=-data.targets_pos[0].err_y_m
    x=data.targets_pos[0].dist

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'/ball',"/drone_pos")
    

def drone_broadcaster(data):
    print(data)
    # global absolute_ball_pos_in_absolute_frame
    # rospy.loginfo("averaged_ball_publisher")
    br = tf.TransformBroadcaster()
    br.sendTransform((data.pose.position.x, data.pose.position.y, data.pose.position.z),(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w),rospy.Time.now(),'/drone_pos',"map")





def translator():
    # pub_ball_absolute_pos_in_abs_frame = rospy.Publisher('/trajectory_fitting/target_pos', PointStamped, queue_size=100)
    
    sub_pos_from_vision=rospy.Subscriber("/distance_finder/target_pos", ObjPosVec, ball_broadcaster)
    
    sub_drone_pos_from_mavros=rospy.Subscriber("/zed/pose", PoseStamped, drone_broadcaster)
    
    
    

    rate = rospy.Rate(2) # 2hz
    

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Ciao")
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
    
    
    



