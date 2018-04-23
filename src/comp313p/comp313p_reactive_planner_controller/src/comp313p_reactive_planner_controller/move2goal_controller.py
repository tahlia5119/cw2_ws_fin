#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp313p_reactive_planner_controller.planned_path import PlannedPath
from comp313p_reactive_planner_controller.controller_base import ControllerBase
from comp313p_mapper.srv import *
import math
import angles
import time

# This sample controller works a fairly simple way. It figures out
# where the goal is. It first turns the robot until it's roughly in
# the correct direction and then keeps driving. It monitors the
# angular error and trims it as it goes.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
    
        ControllerBase.__init__(self, occupancyGrid)
        
        #Open file to insert performance data
        path = '/home/tahlia/dataP4.txt'
        f = open(path,'w')
        heading = 'Distance\tAngle\tTime'
        f.write(heading)
        f.close()
        
        #Initialising variables for measuring performance
        self.timeTaken = 0.0
        self.oldTime = 0.0
        self.angle = 0.0
        self.distanceTravelled = 0.0
        self.angleTurned = 0.0
        
        #PART 1: Getting the velocity of the robot (assuming that the drive speed refers to ONLY the linear velocity)
        #self.velocity = rospy.get_param('drive_speed',10.0)
        
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))


        # Flag to toggle the mapper state
        self.enableSettingMapperState = rospy.get_param('enable_change_mapper_state', True)
        rospy.loginfo('enableSettingMapperState=%d', self.enableSettingMapperState)
        self.mappingState = True

        # Get the service to switch the mapper on and off if required
        if self.enableSettingMapperState is True:
            rospy.loginfo('Waiting for change_mapper_state')
            rospy.wait_for_service('change_mapper_state')
            self.changeMapperStateService = rospy.ServiceProxy('change_mapper_state', ChangeMapperState)
            rospy.loginfo('Got the change_mapper_state service')
            
        #Get the time scale, define time taken
        self.timeScale = rospy.get_param('time_scale_factor',1)
   
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta
    
    def angleDeg(self, angle1, angle2):
        delta = angle2-angle1
        if delta > math.pi:
            ang = angle1+angle2
        elif delta < 0:
            ang = angle1 - angle2
        else:
            ang = angle2-angle1
            
        return ang 
  
    def driveToWaypoint(self, waypoint):
        
        #Get current ROS time
        self.oldTime = rospy.get_time()
        
        vel_msg = Twist()
        self.angle = 0.0
        
        #Get the current pose of the robot
        prevx = self.pose.x
        prevy = self.pose.y
        prevtheta = self.pose.theta

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))
        
        while (distanceError >= self.distanceErrorTolerance) & (not self.abortCurrentGoal) & (not rospy.is_shutdown()):
            
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                #vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * distanceError, 10.0))
                vel_msg.linear.x = 0.15
                #print("Velocity: {}".format(vel_msg.linear.x))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))


            #print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))

            # Toggle switching the mapping on and off, depending on
            # how fast the robot is turning. This has to happen first
            # to make sure the mapping is disabled before the new
            # twist message is sent.
            if self.enableSettingMapperState is True:
                if (self.mappingState is True) and (abs(vel_msg.angular.z) > math.radians(0.1)):
                    self.mappingState = False
                    self.changeMapperStateService(False)
                elif (self.mappingState is False) and (abs(vel_msg.angular.z) < math.radians(0.1)):
                    self.mappingState = True
                    self.changeMapperStateService(True)
            
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
                                                      
        self.distanceTravelled += sqrt((self.pose.x-prevx)**2+(self.pose.y-prevy)**2)
        self.angleTurned += abs(self.angleDeg(prevtheta, self.pose.theta))*180/math.pi
        self.timeTaken += rospy.get_time() - self.oldTime
        
        # Stopping our robot after the movement is over
        self.stopRobot()

        return (not self.abortCurrentGoal) & (not rospy.is_shutdown())

    def rotateToGoalOrientation(self, goalOrientation):
        
        #Get the current robot angle theta
        prevtheta = self.pose.theta
        
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        if self.enableSettingMapperState is True:
            self.mappingState = False
            self.changeMapperStateService(False)

        while (math.fabs(angleError) >= self.goalAngleErrorTolerance) & (not self.abortCurrentGoal) \
              & (not rospy.is_shutdown()):
            #print 'Angular Error: ' + str(angleError)

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()
            angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)
        
        #Determien the time taken by the robot, get the angle turned
        self.timeTaken += rospy.get_time()-self.oldTime
        self.angleTurned += abs(self.angleDeg(prevtheta, self.pose.theta))*180/math.pi
        
        # Stop movement once finished
        self.stopRobot()

        if self.enableSettingMapperState is True:
            self.mappingState = True
            self.changeMapperStateService(True)
            
        print("Distance Travelled: {}".format(self.distanceTravelled))
        print("Angle Turned: {}".format(self.angleTurned))
        print("Time taken: {}".format(self.timeTaken))
        
        #Write the performance data to the file opened when script initiated
        path = '/home/tahlia/data4.txt'
        f = open(path,'a+')

        fileData = '\n'+str(round(self.distanceTravelled,2)) + '\t' + str(round(self.angleTurned,2)) + '\t' + str(round(self.timeTaken,2))
        f.write(fileData)
        f.close()
        
        #Reset variables for next goal
        self.distanceTravelled = 0.0
        self.angleTurned = 0.0
        self.timeTaken = 0.0

        return (not self.abortCurrentGoal) & (not rospy.is_shutdown())
