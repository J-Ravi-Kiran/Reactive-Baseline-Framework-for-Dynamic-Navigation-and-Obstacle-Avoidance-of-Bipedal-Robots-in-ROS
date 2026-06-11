#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

rospy.init_node("moving_block_controller")

rospy.wait_for_service("/gazebo/set_model_state")
set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

rate = rospy.Rate(50)
t0 = rospy.Time.now().to_sec()

while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec() - t0

    state = ModelState()
    state.model_name = "moving_block"
    state.reference_frame = "world"

    state.pose.position.x = 2.0 + 0.6 * math.sin(0.5 * t)
    state.pose.position.y = 0.0
    state.pose.position.z = 0.25

    state.pose.orientation.w = 1.0

    try:
        set_state(state)
    except rospy.ServiceException as e:
        rospy.logerr(e)

    rate.sleep()
