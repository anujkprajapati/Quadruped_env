import pybullet as p
import numpy as np
import os

class Robot:
    def __init__(self,client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'custom_robot.urdf')
        self.robot = p.loadURDF(fileName=f_name, basePosition=[0,0,0.205], physicsClientId=client)
        self.jointArray = [1,2,3,5,6,7,10,11,12,14,15,16]
        # self.end_eff = [4,8,13,17]
        
    def get_ids(self):
        return self.client, self.robot
    
    def apply_action(self, action):
        #action is h,k,l,a,b for each leg
        #use IK to calculate joint angles for each leg: target = list of 12 floats
        # i=0
        # target1 = p.calculateInverseKinematics(self.robot, self.end_eff[i],)
        # target = np.full(12,0)
        p.setJointMotorControlArray(self.robot, self.jointArray, controlMode=p.POSITION_CONTROL,
            targetPositions=action, physicsClientId=self.client)
        
        

        
    def get_obs(self):
        status = p.getJointStates(self.robot, self.jointArray,physicsClientId=self.client)
        linear,_ = p.getBaseVelocity(self.robot, self.client)
        pos,ori = p.getBasePositionAndOrientation(self.robot, self.client)
        obs = [t[0] for t in status]
        h1 = p.getLinkState(self.robot,4, self.client)[0]
        h2 = p.getLinkState(self.robot,8, self.client)[0]
        h3 = p.getLinkState(self.robot,13, self.client)[0]
        h4 = p.getLinkState(self.robot,17, self.client)[0]

        return h1,h2,h3,h4,pos[2],pos[0],pos[1],ori, linear[0], obs