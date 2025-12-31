#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

current_pose = None

def callback(data):
    global current_pose
    current_pose = data.pose.pose

def get_yaw_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2] 

if __name__ == '__main__':
    rospy.init_node('waypoint_saver', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    
    print("-" * 40)
    print("Koodinatını istediğiniz nokta için ENTER'a basın.")
    print("-" * 40)

    while not rospy.is_shutdown():
        try:
            input("Kaydetmek için ENTER'a bas... (Çıkış: Ctrl+C)")
            
            if current_pose:
                x = current_pose.position.x
                y = current_pose.position.y
                yaw = get_yaw_from_pose(current_pose)
                
                print(f"\nMevcut konum->  - {{x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}}}")
                print("-" * 20)
            else:
                print("AMCL node kontrol ediniz..")
                
        except rospy.ROSInterruptException:
            pass