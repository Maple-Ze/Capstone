#!/usr/bin/env python
import rospy
import random
import actionlib
import time

from actionlib_msgs.msg import *                                
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolNav():
    def __init__(self):
        

        rospy.init_node('patrol_nav_node', anonymous=False)


        rospy.on_shutdown(self.shutdown)


        self.rest_time     = rospy.get_param("~rest_time", 5)
        self.keep_patrol   = rospy.get_param("~keep_patrol",   False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type   = rospy.get_param("~patrol_type", 0)
        self.patrol_loop   = rospy.get_param("~patrol_loop", 2)
        self.patrol_time   = rospy.get_param("~patrol_time", 5)


        self.locations = dict()  
        self.locations['one']   = Pose(Point(4.28564075035,  3.7944754537, 0.000), Quaternion(0.000, 0.000, -0.999187940384, 0.0402921802795))
        self.locations['two']   = Pose(Point(2.80089789076,  3.75163288353, 0.000), Quaternion(0.000, 0.000, -0.999421186545,  0.034018993002))
        self.locations['three'] = Pose(Point(4.2581847411,  7.83543252813, 0.000), Quaternion(0.000, 0.000, -0.999114356415, 0.0420773431252))
        self.locations['four']  = Pose(Point(4.15707936356, 6.85442485998, 0.000), Quaternion(0.000, 0.000, 0.997792352715, 0.0664109995628))
        self.locations['five']  = Pose(Point(0.200589099314, 7.76813161799, 0.000), Quaternion(0.000, 0.000,0.985441957522,0.170012200606))


        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
    
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        n_successes  = 0
        target_num   = 0
        location   = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        sequeue = ['one', 'two', 'three', 'four', 'five', 'six']
         
        rospy.loginfo("Starting position navigation ")
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            location = sequeue[target_num]
            rospy.loginfo("target_num_value:"+str(target_num)) 
            rospy.loginfo("Going to: " + str(location))
            time.sleep(1)
            self.send_goal(location)

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(90))
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
            else:
                state = self.move_base.get_state()

            target_num += 1
            
            if target_num > 3:
               target_num = 0
            

    def send_goal(self, locate):
        self.goal = MoveBaseGoal() 
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

if __name__ == '__main__':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")

