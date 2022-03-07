import numpy as np
import matplotlib.pyplot as plt
from bounded_flies import *

class Simple_Seeking_Fly(Fly):
    def __init__(self,params):
        Fly.__init__(self,params)
        self.max_trans_speed = 4

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
        self.velocity = self.bound_collision()
        self.velocity = self.limit_vector(self.velocity,self.max_trans_speed)
        self.rotation_speed = self.rotation_constant*(self.heading_to_angle(self.velocity)-self.angle)
        self.rotation_speed = np.clip(self.rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # update position
        self.position = self.position + 0.5*(self.velocity+pre_velocity)*self.DELTA_T

        # update angle
        self.angle = self.angle + self.rotation_speed*self.DELTA_T
        self.angle = self.wrap_angle(self.angle)


class Simple_Fleeing_Fly(Fly):

    def __init__(self,params):
        Fly.__init__(self,params)
        self.color = 'm'
        self.identity = 'female'
        self.target_preference = 'male'
        self.max_trans_speed = 5
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
        self.velocity = self.bound_collision()
        self.velocity = self.limit_vector(self.velocity,self.max_trans_speed)
        self.rotation_speed = self.rotation_constant*(self.heading_to_angle(self.velocity)-self.angle)
        self.rotation_speed = np.clip(self.rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # update position
        self.position = self.position + 0.5*(self.velocity+pre_velocity)*self.DELTA_T

        # update angle
        self.angle = self.angle + self.rotation_speed*self.DELTA_T
        self.angle = self.wrap_angle(self.angle)

class Simple_Fixed_Fly(Fly):

    def __init__(self,params):
        Fly.__init__(self,params)
        self.color = 'm'
        self.identity = 'female'
        self.target_preference = 'male'

    def update(self,targets=[]):
        pass

# TESTING DIFFERENT STARTING DISTANCE TO FIXED TARGET
WORLD_SIZE = 100
DELTA_T = 0.1
NFRAMES = 500

n = 10
xs = np.linspace(-WORLD_SIZE+10,0,n)
ys = np.zeros((n,))
vxs = np.zeros((n,))
vys = np.zeros((n,))
angles = np.zeros((n,))
flies = []
for x, y, vx, vy, angle in zip(xs,ys,vxs,vys,angles):
    flies.append(Simple_Seeking_Fly({'position': (x,y), 'velocity': (vx,vy), 'angle':angle, 'WORLD_SIZE':WORLD_SIZE, 'DELTA_T':DELTA_T}))

for fly in flies:
    fly.random_move_mag = 0
    fly.perception_radius = 2*WORLD_SIZE

flies.append(Simple_Fixed_Fly({'position': (0.7*WORLD_SIZE,0), 'velocity': (0,0), 'angle':0, 'WORLD_SIZE':WORLD_SIZE, 'DELTA_T':DELTA_T}))

nflies = len(flies)

history_positions = np.zeros((NFRAMES,nflies,2))
for kk in range(NFRAMES):
    for ii in range(nflies):
        flies[ii].update([flies[jj] for jj in range(nflies) if jj != ii])
        history_positions[kk,ii] = flies[ii].position

plt.figure(figsize=[12,6])
plt.subplot(121)
plt.xlim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
plt.ylim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
world_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE, fill=False, facecolor=None, edgecolor='k', linestyle='--', lw=2)
plt.gca().add_artist(world_circle)
wall_margin_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE-flies[0].WALL_MARGIN, fill=False, facecolor=None, edgecolor='k', linestyle='--', lw=2)
plt.gca().add_artist(wall_margin_circle)
for ii in range(nflies):
    plt.plot(history_positions[:,ii,0],history_positions[:,ii,1],color=flies[ii].color)
for ifly in range(nflies-1):
    plt.scatter(history_positions[0,ifly,0],history_positions[0,ifly,1],s=40)
plt.scatter(history_positions[0,-1,0],history_positions[0,-1,1],s=40,c='m')

plt.subplot(122)
for ifly in range(nflies-1):
    pair_distance = np.linalg.norm(history_positions[:,ifly,:]-history_positions[:,-1,:], axis=1)
    plt.plot(np.arange(NFRAMES)*DELTA_T,pair_distance)
plt.show()