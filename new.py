import mujoco
from gymnasium import Env

class BlimpEnv(Env):


    def __init__(self, modelName : str = 'sano.xml' ):
        self.m = mujoco.MjModel.from_xml_path(modelName)
        self.d = mujoco.MjData(m)
        pass

    def calcReward(self):
        """
        Calculate Reward using the observation from the sensors
        """
        sensor_imu = self.d.sensor('body_gyro').data.copy() # this will give you angular
                                                            # accelerations
        join_vel = self.d.sensor('hinge_y_v').data.copy()
        join_pos = self.d.sensor('hinge_y').data.copy()
        # compute the reward function
        pass
        

    """
    TODO Override the ActType to reflect our action space [m1,m2,m3,m4,m5,m6] 
    Action: ActType is a tuple of ObsType, SupportsFloat, bool, bool, dict[str, Any]
    """
    def step(self, action: ActType):
        actuator1 = self.d.actuator('prop_joint')
        actuator2 = self.d.actuator('prop_joint2')
        actuator3 = self.d.actuator('prop_joint3')
        actuator4 = self.d.actuator('prop_joint4')
        actuator5 = self.d.actuator('prop_joint5')
        actuator6 = self.d.actuator('prop_joint6')

        actuator1.ctrl = [action[0]]
        actuator2.ctrl = [action[1]]
        actuator3.ctrl = [action[2]]
        actuator4.ctrl = [action[3]]
        actuator5.ctrl = [action[4]]
        actuator6.ctrl = [action[5]]
        mujoco.mj_step(self.m, self.d)
        
        """
        Termination Logic:
               If angular acceleration is above lets say 1 then we terminate.
        """
        # TODO return observations, reward, terminated or not, and some aux
        # Look at the doc for more information.
        pass


    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        pass

    def render():
        pass

    def close():
        pass

