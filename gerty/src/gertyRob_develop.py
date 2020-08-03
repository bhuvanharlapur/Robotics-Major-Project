#!/usr/bin/env python

################Importing all required libraries and message types ##########################
import rospy
import time
import threading
from math import sqrt, pow
# from gerty.msg import GertyRoboAction, GertyRoboGoal, GertyRoboFeedback, GertyRoboResult
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header
from goal_publisher.msg import PointArray
from gerty.msg import robot_status
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan                               # Importing Laser datas
###############################################################################################


class Gerty_run():

    def __init__(self):
        ##################Initializing all variables###########################################
        self.x_dest = 0
        self.y_dest = 0
        self.x_pos = 0
        self.y_pos = 0
        self.first_rot = 1
        self.prev_state = 0
        self.flago = 1
        self.x_o = 0
        self.y_o = 0
        self.for_first_goal = 1

        ################## For the first rotation of the robot #################################
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel = Twist()
        time.sleep(2)
        if self.first_rot == 1:
            thread = threading.Thread(self.make_one_rotation())
            thread.start()
            thread.join()
        else:
            pass
        #######################################################################################
        ###############Subribing and publishing to required topics#############################
        # self.sub_2 = rospy.Subscriber('gazebo/model_states', ModelStates, self.check_tolerance)
        self.sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.check_tolerance)
        self.pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size = 1)
        self.pub_2 = rospy.Publisher('get_robo_stat', robot_status, queue_size = 1)
        self.sub = rospy.Subscriber('star_goals', PointArray, self.reach_goal)
        self.sub_3 = rospy.Subscriber('move_base/status', GoalStatusArray, self.stat_find)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.rate = rospy.Rate(10)
        self.stat = robot_status()
        ######################################################################################

    #############################Making first rotation for localization#######################
    def make_one_rotation(self):
        self.vel.angular.z = 0.8
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)
        time.sleep(15)
        self.vel.angular.z = 0.0
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)
        self.first_rot = 0
    #########################################################################################


    #################################### It publishes goal to the move_base #################
    def reach_goal(self, dest):
        try:
            time.sleep(1)
            self.x_dest = dest.goals[0].x
            self.y_dest = dest.goals[0].y
            self.dest_p = MoveBaseActionGoal()
            #print()
            self.dest_p.goal.target_pose.header.frame_id = "map"
            self.dest_p.goal.target_pose.pose.position.x = self.x_dest
            self.dest_p.goal.target_pose.pose.position.y = self.y_dest
            self.dest_p.goal.target_pose.pose.orientation.w = 1
            self.pub.publish(self.dest_p)
        except Exception as e:
            rospy.loginfo("Something went wrong or reached all goal points, Please wait!!!!")
            pass
        ####################################################################################


    #####It takes current position and saves location for continuous check of robot state###
    def check_tolerance(self,pos):
        self.x_pos = pos.pose.pose.position.x
        self.y_pos = pos.pose.pose.position.y
        if self.flago == 1:
            self.x_o = self.x_pos
            self.y_o = self.y_pos
            self.flago = 0
        else:
            pass
        # print(self.x_dest,self.x_pos,self.y_dest,self.y_pos)
    #######################################################################################


    ##################### Checks status of robot at particular interval of time ###########
    def check_possibility(self):
        threading.Timer(40.0, self.check_possibility).start()
        x_n = self.x_pos
        y_n = self.y_pos
        #print(x_n,y_n)
        if abs(self.x_dest - self.x_pos) < 0.2 and abs(self.y_dest - self.y_pos) < 0.2:
            # print("Reached goal {} {}".format(self.x_dest,self.y_dest))
            self.stat.robo_stat = "Reached"
            #print("Reached goal {} {}".format(self.x_dest,self.y_dest))
            #print("Reward Gained {}".format(self.dest.goals[0].reward))
            rospy.sleep(1)
            self.pub_2.publish(self.stat)
            self.rate.sleep()

        else:

            dist = sqrt(pow((self.x_o - x_n),2)+pow((self.y_o - y_n),2))
            if dist < 0.2:
                print("Waited for longer and skipping the goal {} {}".format(self.x_dest,self.y_dest))
                self.stat.robo_stat = "DifficultToReach"
                # rospy.sleep(1)
                self.pub_2.publish(self.stat)
                self.rate.sleep()

            else:
                self.flag = False
                self.stat.robo_stat = "Moving"
                self.pub_2.publish(self.stat)
                self.rate.sleep()
        self.flago = 1
    ########################################################################################

    ####################### Subscribes to status and publishes required status of robot ####
    def stat_find(self, state):
        #self.stat.robo_stat = "Rotation Done"
        try:
            if state.status_list[0].status == self.prev_state:
                self.stat.robo_stat = "Moving"
                self.pub_2.publish(self.stat)
                self.rate.sleep()
            elif state.status_list[0].status == 3:
                self.stat.robo_stat = "Reached"
                rospy.sleep(1)
                # while pub_2.get_num_connections() < 1:
                #     pass
                self.pub_2.publish(self.stat)
                #print("Reached goal {} {}".format(self.x_dest,self.y_dest))
                #print("Reward Gained {}".format(self.dest.goals[0].reward))
                # print("collect reward {}".format(self.reward))
                self.rate.sleep()
            elif state.status_list[0].status == 4:
                print("entered to recovery mode!!!")
                self.stat.robo_stat = "DifficultToReach"
                if (self.front > 0.5):
                    if abs(self.x_dest - self.x_pos) < 0.2 and abs(self.y_dest - self.y_pos) < 0.2:
                        self.stat.robo_stat = "Reached"
                    else:
                        pass
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.02
                elif (self.back > 0.5):
                    if abs(self.x_dest - self.x_pos) < 0.2 and abs(self.y_dest - self.y_pos) < 0.2:
                        self.stat.robo_stat = "Reached"
                    else:
                        pass
                    self.vel.linear.x = -0.1
                    self.vel.angular.z = -0.02
                elif (self.right > 0.2) or (self.left > 0.2):
                    if abs(self.x_dest - self.x_pos) < 0.2 and abs(self.y_dest - self.y_pos) < 0.2:
                        self.stat.robo_stat = "Reached"
                    else:
                        pass
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.4
                self.pub_vel.publish(self.vel)
                self.rate.sleep()
                rospy.sleep(5)
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.pub_vel.publish(self.vel)
                self.pub_2.publish(self.stat)
                self.rate.sleep()
            else:
                self.stat.robo_stat = "Moving"
                self.pub_2.publish(self.stat)
                self.rate.sleep()
            self.prev_state = state.status_list[0].status

        except Exception as e:
            pass

        if self.for_first_goal == 1:
            time.sleep(1)
            self.stat.robo_stat = "Rotated"
            self.pub_2.publish(self.stat)
            self.rate.sleep()
            self.for_first_goal = 0
        else:
            pass
    #######################################################################################


    ####################### Laser callback function ######################################
    def callback_laser(self, laser_data):
        laser = list(laser_data.ranges)
        self.front = min(min(laser[0:30]),min(laser[330:359]))
        self.left = min(laser[30:150])
        self.right = min(laser[210:330])
        self.back = min(laser[150:210])

    ########################################################################################

if __name__ == '__main__':
    rospy.init_node('GetryRun')
    run = Gerty_run()
    run.check_possibility()
    rospy.spin()
