#!/usr/bin/env python


import rospy
import operator
from math import pow, sqrt
import time
from std_msgs.msg import Header
import threading
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from collections import OrderedDict
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from gerty.msg import robot_status



class retry_goals():

    def __init__(self):
        rospy.init_node('retry_goals')
        self.x_dest=0
        self.y_dest=0
        self.flago = 1
        self.x_pos = 0
        self.y_pos = 0
        self.x_o = 0
        self.y_o = 0
        self.flag = True
        self.i=0
        self.reach_goals = []
        #self.sub_2 = rospy.Subscriber('gazebo/model_states', ModelStates, self.check_tolerance)
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.check)
        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        # self.pub_2 = rospy.Publisher('get_robo_stat', robot_status, queue_size = 1)
        self.sub_3 = rospy.Subscriber('move_base/status', GoalStatusArray, self.stat_find)
        # self.sub_4= rospy.Subscriber('reached_goals',robot_status,self.reached_goals)
        self.sub_5=rospy.Subscriber('missed_goals', PointArray,self.reach_missed_goal)
        #self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.rate = rospy.Rate(10)
        self.stat = robot_status()

    def reach_missed_goal(self,missed_goals):
        self.reach_goals = missed_goals.goals

    def pub_to_goals(self):
        try:
            if self.i<len(self.goals):
                self.x_dest = self.reach_goals.goals[self.i].x
                self.y_dest = self.reach_goals.goals[self.i].y
                #print(self.x_dest,self.y_dest)
                self.dest_p = MoveBaseActionGoal()
                self.dest_p.goal.target_pose.header.frame_id = "map"
                self.dest_p.goal.target_pose.pose.position.x = self.x_dest
                self.dest_p.goal.target_pose.pose.position.y = self.y_dest
                self.dest_p.goal.target_pose.pose.orientation.w = 1
                self.pub.publish(self.dest_p)
            else:
                print('All goals have been reached')

        except Exception as e:
            print("goals not yet printed")
            pass

    def check(self,pos):
        self.x_pos = pos.pose.pose.position.x
        self.y_pos = pos.pose.pose.position.y
        if self.flago == 1:
            self.x_o = self.x_pos
            self.y_o = self.y_pos
            self.flago = 0
        else:
            pass

    def check_possibility(self):
      	 threading.Timer(30.0, self.check_possibility).start()
      	 x_n = self.x_pos
      	 y_n = self.y_pos
         #print(x_n,y_n)
      	 if abs(self.x_dest - self.x_pos) < 0.2 and abs(self.y_dest - self.y_pos) < 0.2:
          	 self.i += 1
             #self.pub_2.publish(self.stat)
          	 self.rate.sleep()
      	 else:
             dist = sqrt(pow((self.x_o - x_n),2)+pow((self.y_o - y_n),2))
             if dist < 0.2:
                 self.flag = True
                 self.i += 1
                 print("Waited for longer and skipping the goal {} {}".format(self.x_dest,self.y_dest))
                 self.rate.sleep()
             else:
                 self.flag = False
                 self.flago = 1

    def stat_find(self, state):
        try:
            if state.status_list[0].status == 3:
                self.i += 1
                rospy.sleep(1)
                self.rate.sleep()
            else:
                pass
        except Exception as e:
            pass


if __name__ == '__main__':
    run = retry_goals()
    run.pub_to_goals()
    run.check_possibility()
    rospy.spin()
