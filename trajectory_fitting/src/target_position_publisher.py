#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from distance_finder.msg import ObjPosVec
from scipy.spatial.transform import Rotation as R

import tf
import collections

# FRAMES
# 1) DRONE FRAME
# 2) ABSOLUTE FRAME

# POSITIONS

# 1) DRONE POSITION IN FIXED FRAME
drone_pose_in_absolute_frame=PoseStamped()

# 2) RELATIVE BALL POSITION IN DRONE FRAME
drone_to_ball_in_drone_frame=PointStamped()

# 3) RELATIVE BALL POSITION IN ABSOLUTE FRAME
drone_to_ball_in_absolute_frame=ObjPosVec()

# 4+) ABSOLUTE BALL POSITION IN ABSOLUTE FRAME
absolute_ball_pos_in_absolute_frame=PointStamped()

# 5) AVERAGED BALL POSITION
# averaged_drone_to_ball_in_drone_frame

drone_to_ball_in_drone_frame_stack= collections.deque([0,0,0,0,0,0,0,0,0,0],maxlen=10)


# VISION MSG TO 


def absolute_ball_in_absolute_frame(data):

    global drone_to_ball_in_absolute_frame
    global drone_to_ball_in_drone_frame

    global drone_pose_in_absolute_frame
    
    global absolute_ball_pos_in_absolute_frame
    
    """ 
    input:
     
    data:  drone_pos (PoseStamped)

    output:
    
    absolute_ball_pos_in_abs_frame
    
    """
    
    drone_pose_in_absolute_frame=data
    
    # Rotazione ottenuta dai quaternioni della pose del drone, da terna drone a terna globale
    r = R.from_quat([drone_pose_in_absolute_frame.pose.orientation.x, drone_pose_in_absolute_frame.pose.orientation.y, drone_pose_in_absolute_frame.pose.orientation.z, drone_pose_in_absolute_frame.pose.orientation.w])
    r=r.inv()
    
    # Posizione del palloncino
    np_drone_to_ball_in_drone_frame=np.array([drone_to_ball_in_drone_frame.point.x,drone_to_ball_in_drone_frame.point.y,drone_to_ball_in_drone_frame.point.z])
    
    # Posizione del drone
    np_drone_pose_in_absolute_frame=np.array([drone_pose_in_absolute_frame.pose.position.x,drone_pose_in_absolute_frame.pose.position.y,drone_pose_in_absolute_frame.pose.position.z])

    # Posizione relativa drone palloncino nel frame globale
    np_drone_to_ball_in_absolute_frame=r.apply(np_drone_to_ball_in_drone_frame)
    
    # Posizione del palloncino assoluta nel frame globale
    np_ball_position_in_global_frame=np_drone_to_ball_in_absolute_frame+np_drone_pose_in_absolute_frame
    
    absolute_ball_pos_in_absolute_frame.header.stamp = rospy.Time.now()
    absolute_ball_pos_in_absolute_frame.point.x=np_ball_position_in_global_frame[0]
    absolute_ball_pos_in_absolute_frame.point.y=np_ball_position_in_global_frame[1]
    absolute_ball_pos_in_absolute_frame.point.z=np_ball_position_in_global_frame[2]
    
    # rospy.loginfo("abs_pose")


def from_vision_msg_to_drone_pose_in_global_frame(data):
    
    global drone_to_ball_in_drone_frame
    global averaged_drone_to_ball_in_drone_frame
    global drone_to_ball_in_drone_frame_stack
    
    """ 
    input:
    
    data:  drone_to_ball_in_drone_frame (ObjPosVec)
    
    output:
    
    global drone_to_ball_in_drone_frame  (PointStamped)
    
    """

    drone_to_ball_in_drone_frame.header.stamp = rospy.Time.now()
    drone_to_ball_in_drone_frame.point.x=data.targets_pos[0].err_x_m
    drone_to_ball_in_drone_frame.point.z=-data.targets_pos[0].err_y_m
    drone_to_ball_in_drone_frame.point.y=data.targets_pos[0].dist
    
    # drone_to_ball_in_drone_frame_stack.popleft()
    # drone_to_ball_in_drone_frame_stack.append(drone_to_ball_in_drone_frame.point.z)
    
    # print(drone_to_ball_in_drone_frame.point.z)
        
    # averaged_drone_to_ball_in_drone_frame=reject_outliers(drone_to_ball_in_drone_frame_stack)
    
    rospy.loginfo("from_vision_to_pos")

    
def abs_ball_publisher_averaged():
    
    global averaged_drone_to_ball_in_drone_frame
    # rospy.loginfo("abs_ball_publisher")
    br = tf.TransformBroadcaster()
    br.sendTransform((averaged_drone_to_ball_in_drone_frame.point.x, averaged_drone_to_ball_in_drone_frame.point.y, absolute_ball_pos_in_absolute_frame.point.z),tf.transformations.quaternion_from_euler(0, 0, 90),rospy.Time.now(),'/abs_ball_averaged',"map")

def abs_ball_publisher():
    
    global absolute_ball_pos_in_absolute_frame
    # rospy.loginfo("averaged_ball_publisher")
    br = tf.TransformBroadcaster()
    br.sendTransform((absolute_ball_pos_in_absolute_frame.point.x, absolute_ball_pos_in_absolute_frame.point.y, averaged_drone_to_ball_in_drone_frame),tf.transformations.quaternion_from_euler(0, 0, 90),rospy.Time.now(),'/abs_ball',"map")


def reject_outliers(data):
    m = 2
    u = np.mean(data)
    s = np.std(data)
    filtered = [e for e in data if (u - m * s < e < u + m * s)]
    np.average(filtered)
    return filtered


def translator():
    pub_ball_absolute_pos_in_abs_frame = rospy.Publisher('/trajectory_fitting/target_pos2', PointStamped, queue_size=100)
    
    relative_ball = rospy.Publisher('/trajectory_fitting/target_pos', PointStamped, queue_size=100)

    
    sub_pos_from_vision=rospy.Subscriber("/distance_finder/target_pos", ObjPosVec, from_vision_msg_to_drone_pose_in_global_frame)
    
    # sub_drone_pos_from_mavros=rospy.Subscriber("/mavros/local_position/pose", PoseStamped, absolute_ball_in_absolute_frame)
    
    sub_zed_absolute_pose=rospy.Subscriber("/mavros/local_position/pose", PoseStamped, absolute_ball_in_absolute_frame)
    
    

    rate = rospy.Rate(10) # 10hz
    

    while not rospy.is_shutdown():
        try:
            # rospy.loginfo("BOH")
            pub_ball_absolute_pos_in_abs_frame.publish(absolute_ball_pos_in_absolute_frame)
            relative_ball.publish(drone_to_ball_in_drone_frame)
            rospy.loginfo("Sto pubblicando")
            
            abs_ball_publisher()
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
    
    
    
    

# def drone_pose2(data):
#     global drone_pose
#     global est_pos
#     global global_pos2
#     global target_found
#     drone_pose=data
#     r = R.from_quat([drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z, drone_pose.pose.orientation.w])
#     vector=np.array([est_pos.position.point.x,est_pos.position.point.y,est_pos.position.point.z])
#     vector3=np.array([drone_pose.pose.position.x,drone_pose.pose.position.y,drone_pose.pose.position.z])
#     # print('vector3 is : \n',vector3)
#     # print ('\n')
#     # print('vector is : \n',vector)
#     # # print('vector3 is : \n',vector3)

#     # vector=np.array([0,0,1])

#     vector2=r.apply(vector)+vector3
#     # print('vector2 is : \n',vector2)
#     # global_pos2.header.stamp =  est_pos.position.header.stamp
#     global_pos2.point.x=vector2[0]
#     global_pos2.point.y=vector2[1]
#     global_pos2.point.z=vector2[2]
#     # global_pos.distance=vector2[1]



