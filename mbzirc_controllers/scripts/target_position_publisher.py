#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from distance_finder.msg import ObjPosVec
from trajectory_fitting.msg import TargetPos
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import numpy as np

target_found= Bool()
est_pos  =PointStamped() 
global_pos=TargetPos()
global_pos2=PointStamped()
obj_pos=ObjPosVec()
drone_pose=PoseStamped()
abc=PointStamped()
target_found= False


def drone_pose(data):
    global drone_pose
    global est_pos
    global global_pos
    global target_found
    drone_pose=data
    r = R.from_quat([drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z, drone_pose.pose.orientation.w])
    vector=np.array([est_pos.point.x,est_pos.point.y,est_pos.point.z])
    vector3=np.array([drone_pose.pose.position.x,drone_pose.pose.position.y,drone_pose.pose.position.z])
    # print('vector3 is : \n',vector3)
    # print ('\n')
    # print('vector is : \n',vector)
    # # print('vector3 is : \n',vector3)

    # vector=np.array([0,0,1])
# Vector2= global_pose_target
# Vector = local_pose_target 
# Vector3= drone_pose_target
    r=r.inv()
    print("This is drone pose:",vector3)
    print("this is target_pose in global frame:", r.apply(vector))
    vector2=r.apply(vector)+vector3
    print('vector2 is : \n',vector2)
    global_pos.position.header.stamp =  est_pos.header.stamp
    global_pos.position.point.x=vector2[0]
    global_pos.position.point.y=vector2[1]
    global_pos.position.point.z=vector2[2]
    global_pos.distance=vector2[1]

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



def target_pose_in_body_frame(data):
    global target_found
    # target_found=data
    # est_pos.header.frame_id = "/tg"
    try:
        est_pos.header.stamp = rospy.Time.now()
        est_pos.point.x=data.targets_pos[0].err_x_m
        est_pos.point.z=-data.targets_pos[0].err_y_m
        est_pos.point.y=data.targets_pos[0].dist
        # est_pos.distance=data.targets_pos[0].dist

        # est_pos.position.header.stamp = rospy.Time.now()
        # est_pos.position.point.y=-data.targets_pos[0].err_x_m
        # est_pos.position.point.z=-data.targets_pos[0].err_y_m
        # est_pos.position.point.x=data.targets_pos[0].dist
        # est_pos.distance=data.targets_pos[0].dist


        target_found=True

    except:
        target_found=False
        pass



def translator():
    pub = rospy.Publisher('/trajectory_fitting/target_pos', PointStamped, queue_size=100)
    pub2 = rospy.Publisher('/target_found', Bool, queue_size=100)
    pub3= rospy.Publisher('/trajectory_fitting/target_pos2', TargetPos, queue_size=100)
    # pub4= rospy.Publisher('/trajectory_fitting/target_pos3', PointStamped, queue_size=100)
    # pub5= rospy.Publisher('/trajectory_fitting/target_pos4', PointStamped, queue_size=100)


    sub=rospy.Subscriber("/distance_finder/target_pos", ObjPosVec, target_pose_in_body_frame)
    sub=rospy.Subscriber("/mavros/local_position/pose", PoseStamped, drone_pose)
    # sub=rospy.Subscriber("/mavros/local_position/pose", PointStamped, drone_pose2)

    rospy.init_node('translator', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(est_pos)
        pub2.publish(target_found)
        if (target_found):
            pub3.publish(global_pos)
        #     pub4.publish(global_pos2)
        rate.sleep()

if __name__ == '__main__':
    try:
        translator()
    except rospy.ROSInterruptException:
        pass
