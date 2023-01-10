# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import math
import matplotlib.pyplot as plt
from RRT import RRT
from pid_controller import PID

class RobotControl:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')
        self.idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.robot_object = self.sim.getObject("/youBot")
        self.world_frame = self.sim.handle_world
        # self.get_circle_obstacles =

        # get all racks from Coppeliasim scene
        self.racks = []
        for i in range(27):
            self.racks.append(self.sim.getObject(f'/rack[{i}]'))

        # initialize wheel joints
        self.front_left_wheel_joint = self.sim.getObject('./rollingJoint_fl')
        self.rear_left_wheel_joint = self.sim.getObject('./rollingJoint_rl')
        self.rear_right_wheel_joint = self.sim.getObject('./rollingJoint_rr')
        self.front_right_wheel_joint = self.sim.getObject('./rollingJoint_fr')

        # initialize robot arm joints
        self.joint_0 = self.sim.getObject('./youBotArmJoint0')
        self.joint_1 = self.sim.getObject('./youBotArmJoint1')
        self.joint_2 = self.sim.getObject('./youBotArmJoint2')
        self.joint_3 = self.sim.getObject('./youBotArmJoint3')
        self.joint_4 = self.sim.getObject('./youBotArmJoint4')
        self.gripper_joint_1 = self.sim.getObject('./youBotGripperJoint1')
        self.gripper_joint_2 = self.sim.getObject('./youBotGripperJoint2')

    def get_circle_obstacles(self):
        obstacle_list = []
        radius = 0.16711
        for rack in self.racks:
            x_rack, y_rack, _ = self.getObjectPosition(rack)
            y_start = y_rack + 1.055
            for i in range(11):
                obstacle = (x_rack, (y_start - 0.211*i), radius)
                obstacle_list.append(obstacle)
        return obstacle_list

    def setMovement(self, yvel, xvel, yawrate):
        self.sim.setJointTargetVelocity(self.front_left_wheel_joint, -yvel - xvel - yawrate)
        self.sim.setJointTargetVelocity(self.rear_left_wheel_joint, -yvel + xvel - yawrate)
        self.sim.setJointTargetVelocity(self.rear_right_wheel_joint, -yvel - xvel + yawrate)
        self.sim.setJointTargetVelocity(self.front_right_wheel_joint, -yvel + xvel + yawrate)

    def getObjectPosition(self, object):
        return self.sim.getObjectPosition(object, self.world_frame)

    def getObjectOrientation(self, object):
        return self.sim.getObjectOrientation(object, self.world_frame)

    def setObjectPosition(self, object, position: list):
        return self.sim.setObjectPosition(object, self.world_frame, position)

    def setObjectOrientation(self, object, orientation: list):
        return self.sim.setObjectOrientation(object, self.world_frame, orientation)

    def getRobotPosition(self):
        return self.getObjectPosition(self.robot_object)

    def getRobotOrientation(self):
        return self.getObjectOrientation(self.robot_object)

    def setRobotToStartPosition(self):
        x = 0.0
        y = 0.0
        z = self.getObjectPosition(self.robot_object)[2]
        position = [x, y, z]
        orientation = self.getObjectOrientation(self.robot_object)
        return self.setObjectPosition(self.robot_object, [x, y, z]) and self.setObjectOrientation(self.robot_object, orientation)

    def startSimulation(self):
        # When simulation is not running, ZMQ message handling could be a bit
        # slow, since the idle loop runs at 8 Hz by default. So let's make
        # sure that the idle loop runs at full speed for this program:
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        return self.sim.startSimulation()

    def stopSimulation(self):
        # stop simulation in CoppeliaSim
        self.sim.stopSimulation()
        # Restore the original idle loop frequency:
        return self.sim.setInt32Param(self.sim.intparam_idle_fps, self.idle_fps)

    def run(self):
        print('Program started')

        #start simulation in CoppeliaSim
        self.startSimulation()
        # self.setRobotToStartPosition()
        # print(self.sim.getJointPosition(self.join_1))
        # self.sim.setJointPosition(self.joint_1, 0)
        # time.sleep(5)
        # while (t := self.sim.getSimulationTime()) < 15:
        #     # self.setMovement(5, 0, 0)
        #     self.sim.setJointTargetVelocity(self.joint_1, 0.01)
        #     self.client.step()
        #


        self.stopSimulation()
        print('Program ended')

if __name__ == '__main__':
    # ====Search Path with RRT====
    # obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), #These should be filled with the obstacle in our sim
    #                 (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    # # Set Initial parameters
    # rrt = RRT(
    #     start=[0, 0],
    #     goal=[gx, gy],
    #     rand_area=[-2, 15],
    #     obstacle_list=obstacleList,
    #     # play_area=[0, 10, 0, 14]
    #     robot_radius=0.8
    # )
    # path = rrt.planning(animation=show_animation) #this path is the reference trajectory for the PID
    # if path is None:
    #     print("Cannot find path")
    # else:
    #     print("found path!!")

    # pid = PID(path)
    # pid.control()
    # robot_velocities = pid.velocity_path #array of vel inputs for robot
    # robot_steering = pid.steering_path #array of steering inputs for robot


    control = RobotControl()
    control.run()