#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseArray, PolygonStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty
from movebase_sequence.srv import *

class MoveBaseSeq():
    def __init__(self):
        rospy.init_node('movebase_sequence')
        #changed this line to get_param every time so that any change in the parameter would be captured without needing to initialize a new object
        #self._repeating  = rospy.get_param("/movebase_sequence/is_repeating",True)
        self.poses_pub   = rospy.Publisher("/wayposes",PoseArray,queue_size=10)          
        #self.poly_pub    = rospy.Publisher("/poly_path",PolygonStamped,queue_size=10)          
        self.path_pub    = rospy.Publisher("/path",Path,queue_size=10)          
        _ = rospy.Subscriber("/wayposes",PoseArray,self.set_poses)          
        _ = rospy.Subscriber("/corner_pose",PoseStamped,self.add_pose)          
        _ = rospy.Service('/movebase_sequence/get_state', get_state, self.get_state) 
        _ = rospy.Service('/movebase_sequence/toggle_state', toggle_state, self.toggle_state) 
        _ = rospy.Service('/movebase_sequence/set_state', set_state, self.set_state) 
        _ = rospy.Service('/movebase_sequence/reset', reset, self.reset) 
        #_ = rospy.Subscriber("/movebase_sequence/state",Bool,self.set_state)#to switch the sending of the sequnce on or off          
        # = rospy.Subscriber("/movebase_sequence/reset",Empty,self.reset)#to reset the whole sequence          
      
        rospy.set_param("/movebase_sequence/abortion_behaviour","stop")
        rospy.set_param("/movebase_sequence/is_repeating",True)
      
        self.__state__ = True #indicates whether the seq. is running or stopped... edited by /movebase_seq_state topic, should be a service!
        self._i = 0 #how many goals in the list are done          
        self._sending = False #indicates whether there is a goal thats being served or not
        self._internal_call = False #this is to prevent infinite loop of calling self.add_pose() when adding poses through pose array

        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()

        #self.poly_path = PolygonStamped()
        #self.poly_path.header.frame_id = "map"
        #self.poly_path.header.stamp = rospy.Time.now()
        self.poses = PoseArray()
        self.poses.header.frame_id = "map"
        self.poses.header.stamp = rospy.Time.now()

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            #rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server, moving the base "+ ("one way trip" if rospy.get_param("/movebase_sequence/is_repeating",True) == False else "in a repetitive way"))
        rospy.loginfo("Waiting for goals..")


    def __del__(self):
       self.reset(resetRequest())
       print("movebase_sequence ended successfully!")

    def reset(self,request):
        try:
            self.__state__ = True 
            self._i = 0           
            #self._internal_call = False
            if self._sending: self.client.cancel_goal()
            self.path = Path()
            self.path.header.frame_id = "map"
            self.path.header.stamp = rospy.Time.now()
            #self.poly_path = PolygonStamped()
            #self.poly_path.header.frame_id = "map"
            #self.poly_path.header.stamp = rospy.Time.now()
            self.poses = PoseArray()
            self.poses.header.frame_id = "map"
            self.poses.header.stamp = rospy.Time.now()

            self._internal_call = True
            self.poses_pub.publish(self.poses)
            #self.poly_pub.publish(self.poly_path)
            self.path_pub.publish(self.path)

            rospy.loginfo("Sequence reset done!.. waiting for new goals..")
            return resetResponse(True)
        except:
            #this means a problem happened, may be imporved to add a warning or a log, or even handle the possible errors
            rospy.logerr("A problem occured during resetting!")
            return resetResponse(False)

    def set_state(self,request):
        if self.__state__ == request.state : 
           rospy.logwarn("the sent state is the same as the internal state! no change is applied.")
           return set_stateResponse(False)            
        self.__state__ = not self.__state__
        rospy.loginfo("set_state Success! State now is "+ ("operating" if self.__state__ else "paused."))
        if self.__state__ == False:
            if self._sending: self.client.cancel_goal()
        return set_stateResponse(True)            

    def toggle_state(self,request):
        try:
            self.__state__ = not self.__state__
            if self.__state__ == False:
                if self._sending: self.client.cancel_goal()
            rospy.loginfo("toggle_state Success! State now is "+ ("operating" if self.__state__ else "paused."))
            return toggle_stateResponse(True)            
        except:
            rospy.logerr("Something went wrong while toggling the state!")
            return toggle_stateResponse(False)            


    def get_state(self,request):
         return get_stateResponse("operating" if self.__state__ else "Paused") 

    def add_pose(self,msg):
        self.poses.poses.append(msg.pose)
        #self.poly_path.polygon.points.append(self.poses.poses[-1].position)       
        self.path.poses.append(msg)
        self.poses.header.stamp = rospy.Time.now()
        #self.poly_path.header.stamp = rospy.Time.now()
        self.path.header.stamp = rospy.Time.now()
        if self._internal_call: rospy.loginfo("Adding a goal from a pose array...")
        else: rospy.loginfo("Adding a new pose to the list.")
        #updating new visualization topics only if the call was a callback for the topic /corner_pose
        if not self._internal_call:
            self._internal_call = True
            self.poses_pub.publish(self.poses)
            #self.poly_pub.publish(self.poly_path)
            self.path_pub.publish(self.path)
        
    def set_poses(self,msg):
        if not self._internal_call:
            self._internal_call = True
            temp_posestamped = PoseStamped()
            temp_posestamped.header = self.poses.header
            for pose in msg.poses:
                temp_posestamped.pose = pose
                self.add_pose(temp_posestamped)
            self.poses_pub.publish(self.poses)
            #self.poly_pub.publish(self.poly_path)
            self.path_pub.publish(self.path)
            rospy.loginfo("Successfully added goal poses!")
        else:
            self._internal_call = False

    def check_newgoals(self):
        if not self.__state__: 
             return  #state is off, do not send goals!, it won't get here if __state__ is false but as a safety
        elif self._i <= len(self.poses.poses)-1:
          self.movebase_client()
          return
        else:  return  # cool, no new goals :)

        
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self._i)+" is now being processed by the Action Server...")


    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        pass

 
    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self._i)+" received a cancel request after it started executing, successfully cancelled!")

        elif status == 8:
            rospy.loginfo("Goal pose "+str(self._i)+" received a cancel request before it started executing, successfully cancelled!")

        elif status == 3:
            rospy.loginfo("Goal pose "+str(self._i)+" REACHED!") 
            self._set_next_goal()

        elif status == 4:
            behav = rospy.get_param("/movebase_sequence/abortion_behaviour","stop")
            if behav == "stop": self.__state__ = False
            elif not (behav == "continue"): rospy.logwarn("Param /movebase_sequence/abortion_behaviour  is neither 'stop' nor 'continue'! continue is assumed.")
            rospy.logerr ("Goal pose "+str(self._i)+" aborted," +("stopping sequence execution," if behav=='stop' else "continuing with next goals, ")+ "check any errors!")
            self._set_next_goal()

        elif status == 5:
            rospy.logerr("Goal pose "+str(self._i)+" has been rejected by the Action Server. moving to next goal.")
            self._set_next_goal()

        self._sending = False #ended dealing with the goal


    def _set_next_goal(self):
        self._i+=1
        if self._i <= len(self.poses.poses)-1: pass
        elif self._i > len(self.poses.poses)-1:
            if rospy.get_param("/movebase_sequence/is_repeating",True):
               self._i = 0
               rospy.loginfo("reached the end of the sequence successfully, repeating it again!")
            else:
                self._i = 0           
                rospy.loginfo("reached the end of the sequence successfully, waiting another sequence!")
                #clearing the poses seq., path, and visualisation topics   
                self.path = Path()
                self.path.header.frame_id = "map"
                self.path.header.stamp = rospy.Time.now()
                #self.poly_path = PolygonStamped()
                #self.poly_path.header.frame_id = "map"
                #self.poly_path.header.stamp = rospy.Time.now()
                self.poses = PoseArray()
                self.poses.header.frame_id = "map"
                self.poses.header.stamp = rospy.Time.now()

                self._internal_call = True
                self.poses_pub.publish(self.poses)
                #self.poly_pub.publish(self.poly_path)
                self.path_pub.publish(self.path)

    
    def movebase_client(self):
        if  self._sending or not self.__state__: return
        else:
          self._sending = True #don't send as it has already sent a goal thats being served
          goal = MoveBaseGoal()
          goal.target_pose.header.frame_id = "map"
          goal.target_pose.header.stamp = rospy.Time.now() 
          goal.target_pose.pose = self.poses.poses[self._i]
          rospy.loginfo("Sending goal pose "+str(self._i)+" to Action Server")
          #rospy.loginfo(str(self.poses.poses[self._i]))  
          self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)



if __name__ == '__main__':
    movebaseseq = MoveBaseSeq()
    while not rospy.is_shutdown():
      movebaseseq.check_newgoals()
      continue

