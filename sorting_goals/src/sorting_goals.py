#!/usr/bin/env python

#########################Importing all required librabries and message types#################
import rospy
import operator
from math import pow, sqrt
import time
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from collections import OrderedDict
from gerty.msg import robot_status           # Importing Gazebo_msgs inorder to find positions and orientation
##############################################################################################


class goals_logic():

      def __init__(self):
          #####################Initialization #####################################
          rospy.init_node('goals_logic')
          self.x1 = self.y1 = 0
          self.x_reached = self.y_reached = []
          self.new_points=[]
          self.rew_new=[]
          self.dist_new=[]
          self.rew_dis_new=[]
          self.merged_new_list=[]
          self.new_points=[]
          self.rew_new=[]
          self.dist_new=[]
          self.rew_dis_new=[]
          self.i = True
          self.ind = 0
          self.temp_rew_dis=[]
          self.reached_goals_x = []
          self.reached_goals_y =[]
          self.flag=0
          self.reward=0
          self.length = 0
          self.number_of_points = 0
          self.reached_goals=0
          self.skipped_goals=0
          ########################################################################
          ###############Subscribing and publishing to topics####################
          self.sub_2 = rospy.Subscriber('gazebo/model_states', ModelStates, self.sub_pos)
          self.sub = rospy.Subscriber('/goals', PointArray, self.goals_sorting)
          self.pub1=rospy.Publisher('star_goals',PointArray,queue_size = 1)
          self.sub_2 = rospy.Subscriber('get_robo_stat', robot_status, self.pub_goals)
          self.rate = rospy.Rate(10)
          self.goal_reached=robot_status()
          self.missed_goals = PointArray()
          self.missed_goals.header = Header()
          ###########################################################################


      ###############Sorting according to the reward and distance ##################
      ###### If reward is more and distance is less, then it will assign to that ####
      ##############It is made only for first goal to be sorted######################
      def goals_sorting(self,goal_poi):
          self.dist = []
          self.rew = []
          self.rew_dis = []
          self.array_value=[]
          self.points = goal_poi.goals
          self.merged_list=[]
          self.length = len(goal_poi.goals)
          for i in range(len(self.points)):
              x_diff = self.x1 - self.points[i].x
              y_diff = self.y1 - self.points[i].y
              self.rew.append(self.points[i].reward)
              self.dist.append(sqrt(pow(x_diff,2)+pow(y_diff,2)))

          for i in range(len(self.points)):
              self.rew_dis.append(self.rew[i]/self.dist[i])

          self.merged_list=[(self.points[i], self.rew_dis[i]) for i in range(0, len(self.points))]
          self.merged_list.sort(key=operator.itemgetter(1),reverse=True)
          self.goal_points=[]
          self.goal_points=PointArray()
          self.goal_points.header = Header()
          self.goal_points.goals = [i[0] for i in self.merged_list]
          self.sub.unregister()
    ###################################################################################################

    #################### Similar as the above program, but it does the work for remaining points########
      def sort_again(self):
          self.temp_points = self.goal_points.goals
          for i in range(len(self.temp_points)):
              x_diff = self.x1 - self.temp_points[i].x
              y_diff = self.y1 - self.temp_points[i].y
              self.rew_new.append(self.temp_points[i].reward)
              self.dist_new.append(sqrt(pow(x_diff,2)+pow(y_diff,2)))

          for i in range(len(self.temp_points)):
              self.rew_dis_new.append(self.rew_new[i]/self.dist_new[i])

          self.merged_new_list=[(self.temp_points[i], self.rew_dis_new[i]) for i in range(0, len(self.temp_points))]

          self.merged_new_list.sort(key=operator.itemgetter(1),reverse=True)

          self.new_points=[]
          self.new_points=PointArray()
          self.new_points.header = Header()
          self.new_points.goals = [i[0] for i in self.merged_new_list]
          self.goal_points.goals=self.new_points.goals
      ############################################################################################################


      ##################### Takes present position of the robot##############################################
      def sub_pos(self,present_point):
          self.x1 = present_point.pose[1].position.x
          self.y1 = present_point.pose[1].position.y
      ######################################################################################################


      #################### Publishes sorted goals to the topic ###########################################
      def pub_goals(self, statt):
          status=''

          if self.flag==0:
             self.pub1.publish(self.goal_points)
             self.flag=1
          else:
             if statt.robo_stat == "Reached" or statt.robo_stat == "DifficultToReach":
                 status=statt.robo_stat
                 try:

                    self.reached_goals_x.append(self.goal_points.goals[0].x)
                    self.reached_goals_y.append(self.goal_points.goals[0].y)
                    self.reached_goals_x=list(OrderedDict.fromkeys(self.reached_goals_x))
                    self.reached_goals_y=list(OrderedDict.fromkeys(self.reached_goals_y))

                    for i in range(len(self.reached_goals_x)):
                        for j in range(len(self.goal_points.goals)):
                            if self.goal_points.goals[j].x == self.reached_goals_x[i]:
                                if status=='Reached':
                                  self.reached_goals= self.reached_goals + 1
                                  print("Reached goal {} {}".format(self.goal_points.goals[0].x,self.goal_points.goals[0].y))
                                  self.reward=self.reward+self.goal_points.goals[0].reward
                                  print("Cumulative Reward Gained {}".format(self.reward))
                                elif status== 'DifficultToReach':
                                  self.skipped_goals=self.skipped_goals+1
                                  print("Reward lost {}".format(self.goal_points.goals[0].reward))
                                if (self.reached_goals + self.skipped_goals) >= self.length:
                                  rospy.loginfo("Star-Force has Completed all goal points!!!!!!")
                                else:
                                  del self.goal_points.goals[j]
                                  self.sort_again()
                                  self.pub1.publish(self.new_points)

                            else:
                                pass
                 except:
                         pass
             else:
                  pass

        ##############################################################################################################

if __name__ == '__main__':
    goals_logic()
    rospy.spin()
