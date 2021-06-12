import gym
import numpy as np
import pybullet as p
from quadruped.resources.robot import Robot
from quadruped.resources.plane import Plane
from quadruped.resources.trajectory import Path


class QuadrupedEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.full(12,-0.5),
            low = np.array([-0.5,-0.5,-0.5,-0.5]),
            high=np.full(12,0.5))
        self.observation_space = gym.spaces.box.Box(
            low=np.full(12,-0.5),
            high=np.full(12, 0.5)
            )
        self.np_random, _ = gym.utils.seeding.np_random()
        self.client = p.connect(p.GUI)
        # self.client =p.connect(p.DIRECT)


        #self.robot = None
        self.done = False
        self.reset()

    def step(self, action, n_step):
        #swing for left(1,3) or right(2,4)
        count = 50
        left = (n_step/count)%2

        # feed action, note observation
        self.robot.apply_action(action)
        p.stepSimulation()
        #print(action)
        h1,h2,h3,h4,posz, posx, posy,ori, velx, status = self.robot.get_obs()

        # trajectory
        # cgz = 0.12+0.05*np.cos(n_step/1000)
        h1x, h2x, h3x, h4x, h1z, h2z, h3z, h4z = Path.get_location(left,n_step,count)
        
        # centre of gravity: area b/w h1, h4,posbase
        if left==1:
            area = posx*(h1[1]-h4[1]) + h1[0]*(h4[1]-posy) + h4[0]*(posy-h1[1])
        else:
            area = posx*(h2[1]-h3[1]) + h2[0]*(h3[1]-posy) + h3[0]*(posy-h2[1])
        

        r_ori = ori[1]**2 + ori[2]**2 

        # compute reward
        error_x = (h1x-h1[0])**2+(h2x-h2[0])**2+(h3x-h3[0])**2+(h4x-h4[0])**2
        error_z = (h1z-h1[2])**2+(h2z-h2[2])**2+(h3z-h3[2])**2+(h4z-h4[2])**2
        # rpos = (posz-cgz)**2
        rvel = velx if velx !=0 else -10*n_step   #to avoid getting stuck
        rstep = n_step   # length of time
        # rheight = 0 if posz >= 0.15 else -5
        reward = 10*rstep + 10*rvel - r_ori -1000*area**2 - 100*error_x - 20*error_z
        print(f'rewards {rstep}   {rvel}   {1000*area**2} {10*error_z}  {10*error_x}') #only for debugging

        # check if done
        if posz <= 0.15 or n_step>=10000 or r_ori>=0.04:
            self.done = True
            print(f'no steps {n_step}') #only for debugging
        
            
        return status, reward, self.done, dict()
        

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        Plane(self.client)
        self.robot = Robot(self.client)
        
        self.done = False
        return np.full(12,0)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)
        
        
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]