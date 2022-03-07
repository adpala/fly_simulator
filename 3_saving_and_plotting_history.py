import numpy as np
import matplotlib.pyplot as plt
from unbounded_flies import *

class Simple_Seeking_Fly(Fly):
    def update(self,targets=[]):

        self.iframe += 1
        
        close_targets = self.perceive_targets(targets)

        # calculate forces
        self.acceleration = np.array([0,0],dtype=np.float32)
        if len(close_targets) > 0:
            self.acceleration += self.seek_closest(close_targets)
        elif self.iframe % 5 == 0:
            self.acceleration += self.random_move()
        self.acceleration += self.get_friction()

        # update vels
        pre_velocity = self.velocity.copy()
        self.velocity = self.velocity + self.acceleration*self.DELTA_T
        self.velocity = self.limit_vector(self.velocity,self.max_trans_speed)
        self.rotation_speed = self.rotation_constant*(self.heading_to_angle(self.velocity)-self.angle)
        self.rotation_speed = np.clip(self.rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # update position
        self.position = self.position + 0.5*(self.velocity+pre_velocity)*self.DELTA_T
        self.position = self.wrap_position(self.position,self.WORLD_SIZE)

        # update angle
        self.angle = self.angle + self.rotation_speed*self.DELTA_T
        self.angle = self.wrap_angle(self.angle)

class Simple_Target_Fly(Fly):

    def __init__(self,params):
        Fly.__init__(self,params)
        self.color = 'g'
        self.perception_radius = 0.1
        self.identity = 'female'
        self.target_preference = 'none'

    def update(self,targets=[]):

        self.iframe += 1

        # calculate forces
        if self.iframe % 200 == 0:
            self.angle = 2*np.pi*np.random.random_sample()
            self.velocity = self.angle_to_heading(self.angle)*self.max_trans_speed

        # update position
        self.position = self.position + self.velocity*self.DELTA_T
        self.position = self.wrap_position(self.position,self.WORLD_SIZE)

class Simple_Fleeing_Fly(Fly):

    def __init__(self,params):
        Fly.__init__(self,params)
        self.color = 'm'
        self.identity = 'female'
        self.target_preference = 'male'
        self.max_trans_speed = 6
        self.friction_constant = 0.5
        self.perception_radius = 6

    def update(self,targets=[]):

        self.iframe += 1

        close_targets = self.perceive_targets(targets)

        # calculate forces
        self.acceleration = np.array([0,0],dtype=np.float32)
        if len(close_targets) > 0:
            self.acceleration -= self.seek_closest(close_targets)
        self.acceleration += self.random_move()
        self.acceleration += self.get_friction()
        
        # update vels
        pre_velocity = self.velocity.copy()
        self.velocity = self.velocity + self.acceleration*self.DELTA_T
        self.velocity = self.limit_vector(self.velocity,self.max_trans_speed)
        self.rotation_speed = self.rotation_constant*(self.heading_to_angle(self.velocity)-self.angle)
        self.rotation_speed = np.clip(self.rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # update position
        self.position = self.position + 0.5*(self.velocity+pre_velocity)*self.DELTA_T
        self.position = self.wrap_position(self.position,self.WORLD_SIZE)

        # update angle
        self.angle = self.angle + self.rotation_speed*self.DELTA_T
        self.angle = self.wrap_angle(self.angle)


WORLD_SIZE = 100
DELTA_T = 0.1
NFRAMES = 1000

flies = init_random(myclass=Simple_Seeking_Fly,n=1,vel_scale=2,WORLD_SIZE=WORLD_SIZE, DELTA_T=DELTA_T)
target_flies = init_random(myclass=Simple_Fleeing_Fly,n=1,vel_scale=2,WORLD_SIZE=WORLD_SIZE, DELTA_T=DELTA_T)
flies.extend(target_flies)
nflies = len(flies)

history_positions = np.zeros((NFRAMES,nflies,2))
for kk in range(NFRAMES):
    for ii in range(nflies):
        flies[ii].update([flies[jj] for jj in range(nflies) if jj != ii])
        history_positions[kk,ii] = flies[ii].position


plt.figure()
plt.xlim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
plt.ylim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
plt.axhline(y=-WORLD_SIZE,color='k',linestyle='--')
plt.axhline(y=WORLD_SIZE,color='k',linestyle='--')
plt.axvline(x=-WORLD_SIZE,color='k',linestyle='--')
plt.axvline(x=WORLD_SIZE,color='k',linestyle='--')
for ii in range(nflies):
    plt.plot(history_positions[:,ii,0],history_positions[:,ii,1],color=flies[ii].color)

if nflies == 2:
    plt.figure()
    pair_distance = np.linalg.norm(history_positions[:,0,:]-history_positions[:,1,:], axis=1)
    plt.plot(np.arange(NFRAMES)*DELTA_T,pair_distance)

plt.show()