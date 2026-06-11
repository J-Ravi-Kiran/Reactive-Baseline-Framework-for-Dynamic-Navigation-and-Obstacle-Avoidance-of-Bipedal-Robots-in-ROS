#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

class Stage1Controller:
    def __init__(self):
        rospy.init_node("stage1_controller")

        self.robot_name = "biped"
        self.obstacle_name = "moving_block"

        self.state = "HOME"  # Initial state is HOME

        self.home = (0.0, 0.0)  # Home position
        self.wall_x = 3.5    # Wall location (adjust as needed)
        self.step = 0.03     # Step size for movement (adjustable)
        self.safe_dist = 0.6 # Minimum distance to obstacle (adjustable)
        self.sidestep_direction = 1.0  # 1 for right, -1 for left; alternate to prevent drift
        self.tolerance = 0.05  # Tolerance for when the robot is close to the home position

        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")

        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.rate = rospy.Rate(20)

    def distance(self, a, b):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def run(self):
        """Main control loop for robot navigation"""
        while not rospy.is_shutdown():
            robot = self.get_state(self.robot_name, "world")
            obs = self.get_state(self.obstacle_name, "world")

            rx, ry = robot.pose.position.x, robot.pose.position.y
            ox, oy = obs.pose.position.x, obs.pose.position.y

            rospy.loginfo("State: %s, Robot position: (%f, %f)", self.state, rx, ry)

            if self.state == "HOME":
                rospy.loginfo_once("STATE: GO_TO_WALL")
                self.state = "GO_TO_WALL"

            elif self.state == "GO_TO_WALL":
                if rx >= self.wall_x:  # Reached the wall
                    rospy.loginfo_once("Reached the wall. Returning home...")
                    self.state = "RETURN_HOME"
                else:
                    self.move(rx, ry, ox, oy, forward=True)

            elif self.state == "RETURN_HOME":
                if self.distance((rx, ry), self.home) < self.tolerance:  # Close enough to home
                    rospy.loginfo_once("Reporting back from reaching the origin!")
                    rospy.loginfo("Robot reached the starting point (origin).")
                    self.state = "DONE"
                else:
                    self.move(rx, ry, ox, oy, forward=False)

            elif self.state == "DONE":
                rospy.loginfo_once("TASK COMPLETED")
                break

            self.rate.sleep()

    def move(self, rx, ry, ox, oy, forward=True):
        """Move the robot with obstacle avoidance and wall diversion"""
        target = ModelState()
        target.model_name = self.robot_name
        target.reference_frame = "world"

        dist_to_obs = self.distance((rx, ry), (ox, oy))

        if dist_to_obs < self.safe_dist:
            rospy.loginfo_once("Obstacle detected, sidestepping.")
            # Sidestep in alternating direction to prevent constant drift
            target.pose.position.y = ry + self.sidestep_direction * 0.03
            target.pose.position.x = rx
            # Alternate direction for next time
            self.sidestep_direction *= -1.0
        else:
            if forward:
                # Move forward towards the wall, but check for wall proximity
                if rx >= self.wall_x - self.step:
                    # Close to wall, step back slightly and sidestep if needed
                    target.pose.position.x = rx - self.step
                    target.pose.position.y = ry + self.sidestep_direction * 0.03
                    self.sidestep_direction *= -1.0
                else:
                    target.pose.position.x = rx + self.step
                    target.pose.position.y = ry
            else:
                # Return home: move directly towards home position
                dx = self.home[0] - rx
                dy = self.home[1] - ry
                dist = self.distance(self.home, (rx, ry))
                if dist > self.step:
                    target.pose.position.x = rx + self.step * (dx / dist)
                    target.pose.position.y = ry + self.step * (dy / dist)
                else:
                    # Close enough, snap to home
                    target.pose.position.x = self.home[0]
                    target.pose.position.y = self.home[1]

        # Handle potential left wall (assuming there's one around x=0)
        if target.pose.position.x < 0.0 + self.step:
            # Divert if hitting left boundary/wall
            target.pose.position.x = 0.0 + self.step
            target.pose.position.y = ry + self.sidestep_direction * 0.03
            self.sidestep_direction *= -1.0

        target.pose.position.z = 0.45  # Ensure the robot's height is correct
        target.pose.orientation.w = 1.0

        self.set_state(target)

if __name__ == "__main__":
    controller = Stage1Controller()
    controller.run()
