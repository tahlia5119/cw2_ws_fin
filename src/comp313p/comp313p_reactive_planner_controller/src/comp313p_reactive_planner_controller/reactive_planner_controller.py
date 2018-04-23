# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot mtion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp313p_mapper.msg import *

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        #self.sg = SearchGrid()

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()
        # rospy.loginfo("Cell [96,30] is: {}".format(self.occupancyGrid.getCell(96,30)))
        
        for i in range(0,len(self.currentPlannedPath.waypoints)):
            
            cell = self.currentPlannedPath.waypoints[i].coords
            label = self.currentPlannedPath.waypoints[i].label
            
            if self.occupancyGrid.getCell(cell[0],cell[1])==1:
                rospy.loginfo("CELL [{},{}] IS OBSTRUCTED".format(cell[0],cell[1]))
                self.controller.stopDrivingToCurrentGoal()
        pass
        
    
    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        goalReached = False
        
        while (goalReached is False) & (rospy.is_shutdown() is False):

            # Set the start conditions to the current position of the robot
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)
            self.gridUpdateLock.release()

            # If we can't reach the goal, give up and return
            if pathToGoalFound is False:
                rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                              goalCellCoords[0], goalCellCoords[1])
                return False
            
            # Extract the path
            self.currentPlannedPath = self.planner.extractPathToGoal()

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

        return goalReached
            
            
