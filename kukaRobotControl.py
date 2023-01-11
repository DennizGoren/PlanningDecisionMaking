import time
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import math
import matplotlib.pyplot as plt
from RRT import RRT
from RRTStar import RRTStar
from pid_controller import PID
import time

class RobotControl:
    def __init__(self):
        self.start = [3, 11]
        self.goal = [17, 4]
        self.robot_radius = 0.8
        self.client = RemoteAPIClient()
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')
        self.idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.robot_object = self.sim.getObject("/youBot")
        self.world_frame = self.sim.handle_world
        # self.get_circle_obstacles = self.get_circle_obstacles()

        # get all racks from Coppeliasim scene
        self.racks = []
        i = 0
        while True:
            try:
                self.racks.append(self.sim.getObject(f'/rack[{i}]'))
                i += 1
            except Exception:
                break

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

    def rrt_path(self, RRT_star=True):
        if RRT_star:
            algo = RRTStar(
                start=self.start,
                goal=self.goal,
                max_iter=10000,
                rand_area=[0, 15],
                obstacle_list=self.get_circle_obstacles(),
                expand_dis=10,
                search_until_max_iter= True,
                robot_radius=self.robot_radius)

        else:
            algo = RRT(
                start=self.start,
                goal=self.goal,
                rand_area=[0, 15],
                obstacle_list=self.get_circle_obstacles(),
                play_area=[0, 20, 0, 20],
                robot_radius=self.robot_radius)
        tic = time.perf_counter()
        path = algo.planning(animation=False)  # this path is the reference trajectory for the PID
        toc = time.perf_counter()
        print(f"Downloaded the tutorial in {toc - tic:0.4f} seconds")

        algo.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.0001)  # Need for Mac
        plt.show()

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
        return path

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
        print(self.rrt_path())
        # while (t := self.sim.getSimulationTime()) < 15:
        #     # self.setMovement(5, 0, 0)
        #     self.sim.setJointTargetVelocity(self.joint_1, 0.01)
        #     self.client.step()
        #
        self.stopSimulation()
        print('Program ended')

if __name__ == '__main__':

    # pid = PID(path)
    # pid.control()
    # robot_velocities = pid.velocity_path #array of vel inputs for robot
    # robot_steering = pid.steering_path #array of steering inputs for robot

    control = RobotControl()
    control.run()