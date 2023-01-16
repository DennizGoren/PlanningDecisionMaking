import time
from zmqRemoteApi import RemoteAPIClient
import numpy as np
import math
import matplotlib.pyplot as plt
from RRT import RRT
from RRTStar import RRTStar
import time
import pickle

class RobotControl:
    '''
    Class for the control of the robot in the simulation environment. 
    '''

    # Initialize the connection with the simulation environment and retrieve all necessary parameters. 
    def __init__(self):
        self.robot_radius = 0.8
        self.client = RemoteAPIClient()
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')
        self.idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.robot_object = self.sim.getObject("/LineTracer/Body")
        self.goal_object = self.sim.getObject("/indoorPlant")
        self.world_frame = self.sim.handle_world
        start_pos = self.getObjectPosition(self.robot_object)
        self.start = [start_pos[0], start_pos[1]]
        goal_pos = self.getObjectPosition(self.goal_object)
        self.goal = [goal_pos[0], goal_pos[1]]

        # Retrieve all rack obstacles from the CoppeliaSim scene.
        self.racks = []
        i = 0
        while True:
            try:
                self.racks.append(self.sim.getObject(f'/rack[{i}]'))
                i += 1
            except Exception:
                break

        # Initialize the wheel joints.
        self.left_wheel_joint = self.sim.getObject('./DynamicLeftJoint')
        self.right_wheel_joint = self.sim.getObject('./DynamicRightJoint')

    # Function that converts obstacles into arrays of circles, used by the path algorithms. 
    def get_circle_obstacles(self):
        obstacle_list = []
        for i, rack in enumerate(self.racks):
            radius = 0.16711 * self.sim.getObjectSizeFactor(rack)
            x_rack, y_rack, _ = self.getObjectPosition(rack)
            theta = self.getObjectOrientation(rack)[2]
            y_start = y_rack + 1.055 * np.cos(theta)
            x_start = x_rack - 1.055 * np.sin(theta)
            for i in range(11):
                obstacle = (x_start + 0.211 * np.sin(theta) * i, (y_start - 0.211 * np.cos(theta) * i), radius)
                obstacle_list.append(obstacle)
        return obstacle_list

    # Path finding algorithms. 
    def rrt_path(self, RRT_star=False):
        if RRT_star:
            algo = RRTStar(
                start=self.start,
                goal=self.goal,
                max_iter=1500,
                rand_area=[-15, 15],
                obstacle_list=self.get_circle_obstacles(),
                expand_dis=8,
                search_until_max_iter=True,
                robot_radius=self.robot_radius)

        else:
            algo = RRT(
                start=self.start,
                goal=self.goal,
                max_iter=4000,
                rand_area=[-15, 15],
                obstacle_list=self.get_circle_obstacles(),
                play_area=[-15, 15, -15, 15],
                robot_radius=self.robot_radius)
        
        # Display the time it takes the algorithm to find a path. 
        tic = time.perf_counter()
        # Set animation to True if a live animation is wanted. 
        path = algo.planning(animation=False)  
        toc = time.perf_counter()
        print(f"Path found in {toc - tic:0.4f} seconds")

        if path is None:
            print("Couldn't find path")
        else:
            print("Found path!!")
            algo.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.show()

        return path

    # Convert and set the velocity for the wheels of the robot. 
    def setMovement(self, velocity, yawrate):
        robot_scaling_factor = self.sim.getObjectSizeFactor(self.robot_object)
        R = 0.027 * robot_scaling_factor
        h = 0.119 * robot_scaling_factor
        
        self.sim.setJointTargetVelocity(self.right_wheel_joint, ((velocity + yawrate * (h / 2)) / R))
        self.sim.setJointTargetVelocity(self.left_wheel_joint, ((velocity - yawrate * (h / 2)) / R))

    def getObjectPosition(self, object):
        return self.sim.getObjectPosition(object, self.world_frame)

    def getObjectOrientation(self, object):
        return self.sim.getObjectOrientation(object, -1)

    def getObjectVelocity(self, object):
        return self.sim.getObjectVelocity(object, self.world_frame)

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

    # Save the path as a pickle file so it can be used multiple times. 
    def savePath(self):
        with open('foundPath.pkl', 'wb') as f:
            pickle.dump(self.rrt_path(), f)

    def run(self):
        print('Program started')
        # Start simulation in CoppeliaSim.
        self.startSimulation()

        with open('foundPath.pkl', 'rb') as f:
            path = np.array(pickle.load(f))[::-1]

        # PD control loop.
        for desired_pos in path:
            remaining_length = math.sqrt((desired_pos[0] - self.getObjectPosition(self.robot_object)[0]) ** 2 + 
            (desired_pos[1] - self.getObjectPosition(self.robot_object)[1]) ** 2)
            threshold = 0.2

            # PD starting values.
            dt = 0.05
            prev_error_steer = 0
            prev_error_vel = 0

            while remaining_length > threshold:
                # Distance left between the robot and it's desired position.
                remaining_length = math.sqrt((desired_pos[0] - self.getObjectPosition(self.robot_object)[0]) ** 2 + 
                                    (desired_pos[1] - self.getObjectPosition(self.robot_object)[1]) ** 2)
                # Retrieve current information from the simulation environment.
                current_heading = self.getObjectOrientation(self.robot_object)[2]
                current_pos = np.array(self.getObjectPosition(self.robot_object))[:2]
                # Calculate the desired heading and the heading error. 
                desired_heading = math.atan2(desired_pos[1] - current_pos[1], desired_pos[0] - current_pos[0])
                heading_error = desired_heading - current_heading
                heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                
                # Calculate the control output for the steering angle using a PD controller
                Kp = 0.7
                Kd = 0.002

                steering_derivative = (heading_error - prev_error_steer) / dt
                steering = Kp * heading_error + Kd * steering_derivative
                prev_error_vel = heading_error
                steering = math.atan2(math.sin(steering), math.cos(steering))

                # Calculate the control output for the velocity using a PD controller
                Kp = 0.1
                Kd = 0.01

                velocity_derivative = (remaining_length - prev_error_vel) / dt
                velocity = Kp * remaining_length + Kd * velocity_derivative
                # velocity = velocity * (np.pi - abs(heading_error))/np.pi
                prev_error_vel = remaining_length
                
                # Make the robot stops until it's heading error is small enough. 
                if np.abs(heading_error) > np.pi / 32:
                    velocity = 0.0

                # Send the required velocity and steering input to the robot. 
                self.setMovement(velocity, steering)
                self.client.step()

        self.stopSimulation()
        print('Program ended')


if __name__ == '__main__':
    control = RobotControl()
    # Uncomment the following line if you want to search for a new path.
    # control.savePath() 
    control.run()