import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
from bounded_flies import *

class Simple_Fleeseeking_Fly(Fly):
    def __init__(self,params):
        Fly.__init__(self,params)
        self.max_trans_speed = 4
        self.target_preference = 'all'
        self.seeker_radius = 6
        self.fleer_radius = 1
        self.perception_radius = self.seeker_radius

    def update(self,targets=[]):

        self.iframe += 1
        
        close_seek_targets = self.perceive_targets(targets,with_fov=True,radius=self.seeker_radius)
        close_flee_targets = self.perceive_targets(targets,with_fov=False,radius=self.fleer_radius)

        # calculate forces
        self.acceleration = np.array([0,0],dtype=np.float32)
        if len(close_seek_targets) > 0:
            self.acceleration += self.seek_closest(close_seek_targets)

        # close_flee_targets = [target for target in close_flee_targets if target.fly_id != self.seeking_target]
        
        # if len(close_flee_targets) > 0:
        #     self.acceleration -= self.seek_closest(close_flee_targets)
        if np.linalg.norm(self.acceleration) < 0.01:
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
        self.perception_radius = 4

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

def animate(i):
    plt.gca().cla()
    plt.title(flies[0].iframe)
    plt.xlim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
    plt.ylim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
    
    for ii in range(len(flies)):
        flies[ii].update([flies[jj] for jj in range(len(flies)) if jj != ii])

    for fly in flies:
        fly.plot(plt.gca(), show_velocity=True, show_perception=fly.identity == 'male', with_fov=True)
        # fly.plot(plt.gca(), show_velocity=True, show_perception=True, show_collision = False)

    world_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE, fill=False, facecolor=None, edgecolor='k', linestyle='-', lw=2)
    plt.gca().add_artist(world_circle)
    wall_margin_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE-flies[0].WALL_MARGIN, fill=False, facecolor=None, edgecolor='k', linestyle='--', lw=2)
    plt.gca().add_artist(wall_margin_circle)

WORLD_SIZE = 20
DELTA_T = 0.05
INTERVAL_ANIMATION = 1

flies = init_random(myclass=Simple_Fleeseeking_Fly,n=5,vel_scale=2,WORLD_SIZE=WORLD_SIZE, DELTA_T=DELTA_T)
target_flies = init_random(myclass=Simple_Fleeing_Fly,n=3,vel_scale=2,WORLD_SIZE=WORLD_SIZE, DELTA_T=DELTA_T)
flies.extend(target_flies)

# create animation using the animate() function
fig = plt.figure(figsize=[10,10])
myAnimation = animation.FuncAnimation(fig, animate, interval=INTERVAL_ANIMATION)
plt.show()

