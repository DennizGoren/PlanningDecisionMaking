# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time
from zmqRemoteApi import RemoteAPIClient


class RobotControl:
    def __init__(self):
        self.sim = RemoteAPIClient().getObject('sim')
        self.front_left_wheel_joint = self.sim.getObject('./rollingJoint_fl')
        self.rear_left_wheel_joint = self.sim.getObject('./rollingJoint_rl')
        self.rear_right_wheel_joint = self.sim.getObject('./rollingJoint_rr')
        self.front_right_wheel_joint = self.sim.getObject('./rollingJoint_fr')

    def setMovement(self, Yvel, Xvel, Yawvel):
        self.sim.setJointTargetVelocity(self.front_left_wheel_joint, -Yvel - Xvel - Yawvel)
        self.sim.setJointTargetVelocity(self.rear_left_wheel_joint, -Yvel + Xvel - Yawvel)
        self.sim.setJointTargetVelocity(self.rear_right_wheel_joint, -Yvel - Xvel + Yawvel)
        self.sim.setJointTargetVelocity(self.front_right_wheel_joint, -Yvel + Xvel + Yawvel)


    def run(self):
        print('Program started')
        # When simulation is not running, ZMQ message handling could be a bit
        # slow, since the idle loop runs at 8 Hz by default. So let's make
        # sure that the idle loop runs at full speed for this program:
        defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        #start simulation in CoppeliaSim
        self.sim.startSimulation()
        # move robot
        self.setMovement(Yvel=1, Xvel=1, Yawvel=0)
        time.sleep(10)

        # stop simulation in CoppeliaSim
        self.sim.stopSimulation()
        # Restore the original idle loop frequency:
        self.sim.setInt32Param(self.sim.intparam_idle_fps, defaultIdleFps)
        print('Program ended')

if __name__ == '__main__':
    control = RobotControl()
    control.run()