import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
from bounded_flies import *

class Simple_Moving_Fly(Fly):
    def __init__(self,params):
        Fly.__init__(self,params)
        self.max_trans_speed = 4

    def update(self,targets=[]):

        self.iframe += 1

        # calculate forces
        self.acceleration = np.array([0,0],dtype=np.float32)
        self.acceleration += self.random_move()
        # self.acceleration += self.get_friction()

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
        # fly.plot(plt.gca(), show_velocity=True, show_perception=fly.identity == 'male')
        fly.plot(plt.gca(), show_velocity=True, show_perception=True, show_collision = False)

    world_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE, fill=False, facecolor=None, edgecolor='k', linestyle='-', lw=2)
    plt.gca().add_artist(world_circle)
    wall_margin_circle = mpl.patches.Circle((0,0), radius=WORLD_SIZE-flies[0].WALL_MARGIN, fill=False, facecolor=None, edgecolor='k', linestyle='--', lw=2)
    plt.gca().add_artist(wall_margin_circle)

WORLD_SIZE = 20
DELTA_T = 0.05
INTERVAL_ANIMATION = 100

n = 5
angles = np.linspace(0,2*np.pi,n+1)[:-1]
xs = 16*np.cos(angles)
ys = 16*np.sin(angles)
vxs = 2*np.cos(angles)
vys = 2*np.sin(angles)
flies = []
for x, y, vx, vy, angle in zip(xs,ys,vxs,vys,angles):
    flies.append(Simple_Moving_Fly({'position': (x,y), 'velocity': (vx,vy), 'angle':angle, 'WORLD_SIZE':WORLD_SIZE, 'DELTA_T':DELTA_T}))

# create animation using the animate() function
fig = plt.figure(figsize=[10,10])
myAnimation = animation.FuncAnimation(fig, animate, interval=INTERVAL_ANIMATION)
plt.show()