#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from heron_msgs.msg import Drive
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import numpy as np

running = False
def status_callback(data):
    global running
    if data.data == False:
        running = False
    else:
        running = True

def test_goto_rl():
    global running
    rospy.init_node('test_goto_rl')

    # Publisher for the goal
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/rl_status', Bool, status_callback)

    goal = PoseStamped()
    goal.header.frame_id = "world"  # Assuming the goal is given in the map frame
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0  # Set goal orientation (w=1.0 means no rotation)

    # create a list with N different possitons distributed in a circle
    N = 36*2
    distances = [3,4,5,6,7,8,9]
    positions = []
    for dist in distances:
        for i in range(0, N):
            x = dist * np.cos(2 * np.pi * i / N)
            y = dist * np.sin(2 * np.pi * i / N)
            # If the position is within -45 and 45 degrees from the x axis add it to the list
            angle = np.arctan2(y, x) + 0.01
            if angle >= -np.pi/4 and angle <= np.pi/4:
                positions.append((x, y))

    rospy.sleep(1.0)
    for position in positions:

        rospy.sleep(0.5)
        while running:
            rospy.sleep(0.1)
        
        reset_world()
        rospy.loginfo("World reset successfully.")
        rospy.sleep(2.0)

        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        pub.publish(goal)
        rospy.loginfo("Published goal: %s" % goal.pose.position)
        rospy.sleep(0.5)
        
        while running:
            rospy.sleep(0.1)

    rospy.loginfo("Finished.")

if __name__ == '__main__':
    try:
        test_goto_rl()
    except rospy.ROSInterruptException:
        pass
