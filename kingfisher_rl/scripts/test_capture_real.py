#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from kingfisher_msgs.msg import Drive
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
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/rl_status', Bool, status_callback)

    goal = PoseStamped()
    goal.header.frame_id = "base_link"  # Assuming the goal is given in the map frame
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0  # Set goal orientation (w=1.0 means no rotation)

    # create a list with N different possitons distributed in a circle
    N = 8
    positions = []
    for dist in [10]:
        for i in range(0, N):
            x = dist * np.cos(2 * np.pi * i / N)
            y = dist * np.sin(2 * np.pi * i / N)
            positions.append((x, y))

    count = 1
    total = len(positions)
    rospy.sleep(1.0)
    for position in positions:
        print(f"Next goal [{position[0]:.2f}, {position[1]:.2f}]. {count}/{total}")
        user_input = input("press any key: s:skip q:quit\n")
        if user_input.lower() == 's':
            continue
        elif user_input.lower() == 'q':
            print("quit")
            break
        count += 1

        rospy.sleep(0.5)
        while running:
            rospy.sleep(0.1)
        
        rospy.sleep(2.0)

        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        pub.publish(goal)
        rospy.loginfo(f"Published goal: {goal.pose.position.x:.2f} {goal.pose.position.y:.2f}")
        rospy.sleep(0.5)
        
        while running:
            rospy.sleep(0.1)

    rospy.loginfo("Finished.")

if __name__ == '__main__':
    try:
        test_goto_rl()
    except rospy.ROSInterruptException:
        pass
