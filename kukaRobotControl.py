import time
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import math
import matplotlib.pyplot as plt
from RRT import RRT
from RRTStar import RRTStar
from pid_controller import PID
import time
from tqdm import tqdm
from scipy import interpolate


class RobotControl:
    def __init__(self):
        self.start = [0, 0]
        self.goal = [15, 11]
        self.robot_radius = 0.8
        self.client = RemoteAPIClient()
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')
        self.idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.robot_object = self.sim.getObject("/LineTracer")
        self.world_frame = self.sim.handle_world

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
        self.left_wheel_joint = self.sim.getObject('./DynamicLeftJoint')
        self.right_wheel_joint = self.sim.getObject('./DynamicRightJoint')

    def get_circle_obstacles(self):
        obstacle_list = []
        radius = 0.16711
        for i, rack in enumerate(self.racks):
            x_rack, y_rack, _ = self.getObjectPosition(rack)
            theta = self.getObjectOrientation(rack)[2]
            y_start = y_rack + 1.055 * np.cos(theta)
            x_start = x_rack + 1.055 * np.sin(theta)
            for i in range(11):
                obstacle = (x_rack + 0.211 * np.sin(theta) * i, (y_start - 0.211 * np.cos(theta) * i), radius)
                obstacle_list.append(obstacle)
        return obstacle_list

    def rrt_path(self, RRT_star=True):
        if RRT_star:
            algo = RRTStar(
                start=self.start,
                goal=self.goal,
                max_iter=600,
                rand_area=[-10, 20],
                obstacle_list=self.get_circle_obstacles(),
                expand_dis=3,
                search_until_max_iter=False,
                robot_radius=self.robot_radius)

        else:
            algo = RRT(
                start=self.start,
                goal=self.goal,
                rand_area=[-10, 20],
                obstacle_list=self.get_circle_obstacles(),
                play_area=[0, 20, 0, 20],
                robot_radius=self.robot_radius)
        tic = time.perf_counter()
        path = algo.planning(animation=False)  # this path is the reference trajectory for the PID
        toc = time.perf_counter()
        print(f"Downloaded the tutorial in {toc - tic:0.4f} seconds")

        # print(path)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
            # Path smoothing
            # maxIter = 2000
            # smoothedPath = algo.path_smoothing(path, maxIter, self.get_circle_obstacles())
            algo.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            # plt.plot([x for (x, y) in smoothedPath], [y for (x, y) in smoothedPath], '-c')
            plt.grid(True)
            #
            # numpy_path = np.asarray(path, dtype=np.float32)
            # x, y = zip(*numpy_path)
            # x = np.r_[x, x[0]]
            # y = np.r_[y, y[0]]
            # f, u = interpolate.splprep([x, y], s=0, per=True)
            # #create interpolated lists of points
            # xint, yint = interpolate.splev(np.linspace(0, 1, 100), f)
            # plt.scatter(x, y)
            # # plt.plot(xint, yint)
            # plt.pause(0.0001)  # Need for Mac
            plt.show()

        return path

    def setMovement(self, velocity, yawrate):
        robot_scaling_factor = self.sim.getObjectSizeFactor(self.robot_object)
        R = 0.027 * robot_scaling_factor
        h = 0.119 * robot_scaling_factor
        self.sim.setJointTargetVelocity(self.right_wheel_joint, (velocity + yawrate * (h / 2) / R))
        self.sim.setJointTargetVelocity(self.left_wheel_joint, (velocity - yawrate * (h / 2) / R))

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
        return self.setObjectPosition(self.robot_object, [x, y, z]) and self.setObjectOrientation(self.robot_object,
                                                                                                  orientation)

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
        # start simulation in CoppeliaSim
        self.startSimulation()
        # print(self.getObjectOrientation(self.racks[0]))
        # for i, rack in enumerate(self.racks):
        #     print(self.getObjectOrientation(rack))
        self.rrt_path()
        # while (t := self.sim.getSimulationTime()) < 15:
        #     self.setMovement()
        #     self.client.step()
        self.stopSimulation()
        print('Program ended')


if __name__ == '__main__':
    # pid = PID(path)
    # pid.control()
    # robot_velocities = pid.velocity_path #array of vel inputs for robot
    # robot_steering = pid.steering_path #array of steering inputs for robot

    control = RobotControl()
    control.run()