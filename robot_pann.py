#!/usr/bin/env python
"""
Obodroid Private Workshop 2021
Intro to Navigation Stack
Robot Node
- Get Signal from the referee and begin the Competition
- Sending move_base goal to Start
- Sending custom goal to custom behavior
- Stop the move_base goal

Author : Theppasith N. <theppasith.n@obodroid.com>
Date : 16-Mar-2021
"""
##############################################################################
# Imports
##############################################################################
import rospy
from std_msgs.msg import Empty, Bool
from modules.mission import RobotMission
from modules.move_base_interface import MoveBaseInterface 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


##############################################################################
# Class
##############################################################################

laser_mid = None
laser_top = None
twist = Twist()

class Robot(object):

    """
    Robot Class
    """
    def __init__(self):
        # move_base communication module
        self.move_base_interface = MoveBaseInterface(self.move_base_finish)

        ################### CODE HERE ###################
        self.robot_mission = RobotMission()
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # self.clear_map = rospy.ServiceProxy('/move_base/clear_costmap',Empty)
    	self.vel = rospy.Publisher('/cmd_vel',Twist)
        #################################################

        # Referee Signal Subscription
        self.start_signal_sub = rospy.Subscriber(
            '/start',
            Empty,
            self.on_referee_start_callback
        )

        self.finish_signal_sub = rospy.Subscriber(
            '/finish',
            Bool,
            self.on_referee_finish_callback
        )

        self.reset_signal_sub = rospy.Subscriber(
            '/reset',
            Empty,
            self.on_referee_reset_callback
        )

        # Prompt
        self.log("Initialization Completed ! - Waiting for Start Signal")

    def on_referee_start_callback(self, _):
        """
        On Start Signal Received !
        """
        self.log("Receiving START Signal !")
        #################################################
        ################### CODE HERE ###################
        #################################################
        # Setup the Mission Queue
        self.robot_mission.setup()
        # Pop the Frontmost Mission out
        mission, mission_type = self.robot_mission.get_mission()
        # Start That Mission
        self.do_mission(mission, mission_type)
        #################################################
        #################################################
        #################################################

    def on_referee_finish_callback(self, _):
        """
        On Finish Signal Received !
        - Referee will send finish signal to the contestants
        when robot is on the finish line
        - Robot must Stop after receiving the message
        """
        self.log("Receiving FINISH Signal !")
        self.move_base_interface.stop()

    def on_referee_reset_callback(self, _):
        """
        On Reset Signal Received !
        """
        pass

    def move_base_finish(self, result):
        """
        When Move Base Finish
        """
        self.log(result)
        #################################################
        ################### CODE HERE ###################
        #################################################
        # Mission is Success
        is_success = result is "SUCCEEDED"

        # Check if Misssion is Success
        if is_success:
            mission, mission_type = self.robot_mission.get_mission()
            # If we can Get Mission = Do Mission
            if mission:
                self.do_mission(mission, mission_type)
            else:
                # No Mission Left
                self.log("FINISH !! ")
                self.move_base_interface.stop()

        # Mission is not Success
        else:
            self.move_base_interface.stop()
        #################################################
        #################################################
        #################################################

    def do_mission(self, mission, mission_type):
        """
        Proxy to Send Mission to The place
        with corresponding type
        """
        self.log("Do Mission Type : {}".format(mission_type))

        if mission_type == "move_base":
            self.move_base_interface.send_goal(mission)
        elif mission_type == "wait_obstacle":
            self.log("Wait for obstacle")
            self.wait_obstacle()
            self.log("Obstacle passed")
            self.move_base_interface.send_goal(mission)
	elif mission_type == "fly":
	    self.fly()
        else:
            self.log(
                "mission_type[{}] is not defined in do_mission".format(
                    mission_type
                )
            )
    def fly(self):
	global twist
	twist.linear.x = 1000000
	twist.linear.y =0
	twist.linear.z =0
	twist.angular.x =0
	twist.angular.y = 0
	twist.angular.z=0
	self.vel.publish(twist)




    def wait_obstacle(self):
     	#not found
        while(laser_mid>5):
            print("(1)laser mid",laser_mid)
            pass

	#found in front
        while(True):
            print("laser top", laser_top)
            print("(2)laser mid", laser_mid)
	    if(laser_top<7):
	 	print("(3)laser mid",laser_mid)
	 	print("laser top", laser_top)
	 	break
            pass		
	
	#print("not found in front")
	#not found on top	
	    #while(laser_top>6):
	    #	print("laser mid",laser_mid)
	    #	print("laser top", laser_top)
	    #	pass
	print("found on top")
        rospy.sleep(0.5)

    def laser_callback(self, msg):
	print(msg.ranges[100])
	global laser_mid, laser_top
	laser_top = msg.ranges[100]
	laser_mid = msg.ranges[320]


    def clearMap(self):
        clear_map()

    def log(self, msg):
        """
        Logging
        """
        rospy.loginfo(
            "[robot] : {}".format(msg)
        )
    


    
    

##############################################################################
# Main
##############################################################################
if __name__ == '__main__':
    rospy.init_node("robot_node")
    ROBOT = Robot()
    # rospy.Timer(rospy.Duration(3), ROBOT.clearMap())
    # rospy.Timer(rospy.Duration(3), ROBOT.laser_callback)
    rospy.spin()
