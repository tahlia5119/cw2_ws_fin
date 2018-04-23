import rospy

from explorer_node_base import ExplorerNodeBase
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
import time
import math
from collections import defaultdict

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []
        
        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

    def updateFrontiers(self):
        pass

    def chooseNewDestination(self):
        
        #Creating a dictionary to store the frontier cell locations and their distance from the robot
        candidates = defaultdict(list)
        
        #These are the variables in which the desired cell location coordiantes will be stored
        xReturn = 0
        yReturn = 0
        
        #Get the current pose of the robot
        xpos = self.pose.x
        ypos = self.pose.y
        
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True
                    
                    #Get the euclidean distance of the frontier cell from the roboto
                    distance = sqrt((x-xpos)**2+(y-ypos)**2)
                    
                    #Check if the cell has been blacklisted
                    for k in range(0, len(self.blackList)):
                        if self.blackList[k] == candidate:
                            candidateGood = False
                            break
                    #Add any viable candidate cells' coordiantes and distance to dictionary
                    if candidateGood is True:
                        candidates['x'].append(x)
                        candidates['y'].append(y)
                        candidates['dist'].append(distance)
        #If the dictionary isn't empty, find the maximum distance and the corresponding x and y coordinates            
        if candidates:
            ind = candidates['dist'].index(max(candidates['dist']))
            xReturn = candidates['x'][ind]
            yReturn = candidates['y'][ind]
            candidate = (xReturn, yReturn) 
            return True, candidate
            
        return False, None
        
        
    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            self.blackList.append(goal)
            
    #Addinf following code copied from controller_base so that the pose of the robot can be accessed
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose
    
