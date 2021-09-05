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
import rosparam
from std_msgs.msg import Empty, Bool
from modules.mission import RobotMission
from modules.move_base_interface import MoveBaseInterface 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.msg import Config



##############################################################################
# Class
##############################################################################

laser_mid = None
laser_top = None
laser_bottom = None
twist = Twist()
twist2 = Twist()
#new_config = [[{'DWAPlannerROS': {'max_vel_theta': 3.14, 'goal_distance_bias': 20.0, 'publish_cost_grid_pc': True, 'min_vel_x': 0, 'min_vel_theta': 0.3, 'min_vel_y': 0.0, 'yaw_goal_tolerance': 0.01, 'forward_point_distance': 0.325, 'publish_traj_pc': True, 'acc_lim_x': 2.5, 'acc_lim_y': 0.0, 'min_vel_trans': 0.08, 'sim_time': 1.5, 'max_scaling_factor': 0.2, 'vy_samples': 0, 'stop_time_buffer': 0.2, 'oscillation_reset_dist': 0.05, 'vth_samples': 40, 'acc_lim_theta': 3.2, 'path_distance_bias': 32.0, 'max_vel_trans': 3.0, 'xy_goal_tolerance': 0.01, 'occdist_scale': 0.02, 'vx_samples': 20, 'scaling_speed': 0.25, 'max_vel_x': 1.5, 'max_vel_y': 0.0, 'latch_xy_goal_tolerance': False, 'controller_frequency': 10.0}}, '/']]
new_config = [[{'DWAPlannerROS': {'max_vel_theta': 3.14, 'goal_distance_bias': 20.0, 'publish_cost_grid_pc': True, 'min_vel_x': 0, 'min_vel_theta': 0.3, 'min_vel_y': 0.0, 'yaw_goal_tolerance': 0.01, 'forward_point_distance': 0.325, 'publish_traj_pc': True, 'acc_lim_x': 2.5, 'acc_lim_y': 0.0, 'min_vel_trans': 0.08, 'sim_time': 1.5, 'max_scaling_factor': 0.2, 'vy_samples': 0, 'stop_time_buffer': 0.2, 'oscillation_reset_dist': 0.05, 'vth_samples': 40, 'acc_lim_theta': 3.2, 'path_distance_bias': 32.0, 'max_vel_trans': 20, 'xy_goal_tolerance': 0.01, 'occdist_scale': 0.02, 'vx_samples': 20, 'scaling_speed': 0.25, 'max_vel_x': 0.1, 'max_vel_y': 0.0, 'latch_xy_goal_tolerance': False, 'controller_frequency': 10.0}}, '/']]
cfg_temp = Config()

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
        self.vel = rospy.Publisher('/cmd_vel', Twist)
        self.param = rospy.Subscriber('/move_base/DWAPlannerROS/parameter_updates', Config, self.param_callback)
        self.param = rospy.Publisher('/move_base/DWAPlannerROS/parameter_updates', Config)
        # self.clear_map = rospy.ServiceProxy('/move_base/clear_costmap',Empty)
    
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

        global twist2
        twist2.linear.x=0
        twist2.linear.y =0
        twist2.linear.z = 0
        twist2.angular.x = 0
        twist2.angular.y = 0
        twist2.angular.z = 0
        self.vel.publish(twist2)

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
        elif mission_type == "precise_pos":
            self.precise_pos()
            self.move_base_interface.send_goal(mission)
        else:
            self.log(
                "mission_type[{}] is not defined in do_mission".format(
                    mission_type
                )
            )
    def fly(self):
        #self.vel.publish('{linear: {x: 100000000, y: 0, z: 0}', 'angular: {x: 0, y: 0,z: 0}}')	
        global twist
        print("Time to fly!!!!")
        twist.linear.x=1.5 #velocity limit
        twist.linear.y =0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.vel.publish(twist)

    def precise_pos(self):
        global cfg_temp
        # rospy.set_param('move_base/DWAPlannerROS',new_config)
        cfg_temp.doubles[12].value = 0.01
        cfg_temp.doubles[13].value = 0.01
        
        self.param.publish(cfg_temp)

    def wait_obstacle(self):
     	#not found
        while(laser_mid>5):
            print("laser mid",laser_mid)
            pass

	#found in front
        while(True):
            print("laser mid", laser_mid)
            print("laser top", laser_top)
            if(laser_top<7):
                print("laser mid",laser_mid)
                print("laser top", laser_top)
                break
            pass		
	
	#while(True):
        #    print("laser bot", laser_bottom)
	#    if(laser_bottom<6):
	#	print("laser mid",laser_mid)
	#	print("laser bottom", laser_bottom)
	#	break
        #    pass

        # print("found on top")
        
        rospy.sleep(0.5)

    def laser_callback(self, msg):
        # print(msg.ranges[100])
        global laser_mid, laser_top, laser_bottom
        laser_top = msg.ranges[100]
        laser_mid = msg.ranges[320]
        laser_bottom = msg.ranges[540]

    def param_callback(self,msg):
        global cfg_temp
        cfg_temp = msg
        print(msg)




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
