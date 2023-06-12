#!/usr/bin/env python
import rospy
from xarm_planner.srv import pose_plan
from xarm_planner.srv import exec_plan
from geometry_msgs.msg import Pose


#rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
# /xarm/move_line
# [x, y, z, 3.14, 0, 0] {max speed} {max acceleration}

main_pose = Pose()

def callback(msg):
    global main_pose
    main_pose.position.x = msg.position.x
    main_pose.position.y = msg.position.y
    main_pose.position.z = msg.position.z

if __name__ == '__main__':
    print("Xarm Controller Running")
    rospy.init_node("movement_controller")
    rospy.Subscriber("/xarm6_coordinates", Pose, callback)
    rate = rospy.Rate(100)

    main_pose.position.x = 0.3
    main_pose.position.y = 0.0
    main_pose.position.z = 0.2
    main_pose.orientation.x = 1.0
    main_pose.orientation.y = 0.0
    main_pose.orientation.z = 0.0
    main_pose.orientation.w = 0.0

    while not rospy.is_shutdown():
        rospy.wait_for_service('xarm_pose_plan')
        rospy.wait_for_service('xarm_exec_plan')
        planner = rospy.ServiceProxy('xarm_pose_plan', pose_plan)
        executor = rospy.ServiceProxy('xarm_exec_plan', exec_plan)

        print("Trying:\n")
        print("X: " + str(main_pose.position.x))
        print("Y: " + str(main_pose.position.y))
        print("Z: " + str(main_pose.position.z))

        try:
            planner.call(main_pose)
            executor.call(True)
        except:
            print("No posible")
    
        rate.sleep()