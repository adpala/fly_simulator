import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from flies import *

class Simple_Moving_Fly(Fly):
    def update(self,targets=[]):

        self.iframe += 1

        # calculate forces
        self.acceleration = np.array([0,0],dtype=np.float32)
        if self.iframe % 20 == 0:
            self.velocity = np.array([0,0],dtype=np.float32)
            self.acceleration += self.random_move()


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


def animate(i):
    plt.gca().cla()
    plt.title(flies[0].iframe)
    plt.xlim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
    plt.ylim([-WORLD_SIZE*1.1,WORLD_SIZE*1.1])
    
    for ii in range(len(flies)):
        flies[ii].update([flies[jj] for jj in range(len(flies)) if jj != ii])

    for fly in flies:
        fly.plot(plt.gca(),show_velocity=True)

    plt.axhline(y=-WORLD_SIZE,color='k',linestyle='--')
    plt.axhline(y=WORLD_SIZE,color='k',linestyle='--')
    plt.axvline(x=-WORLD_SIZE,color='k',linestyle='--')
    plt.axvline(x=WORLD_SIZE,color='k',linestyle='--')

WORLD_SIZE = 20
DELTA_T = 0.1
INTERVAL_ANIMATION = 100
NFLIES = 5

flies = init_random(myclass=Simple_Moving_Fly,n=NFLIES,vel_scale=2,WORLD_SIZE=WORLD_SIZE, DELTA_T=DELTA_T)

# create animation using the animate() function
fig = plt.figure()
plt.axis('equal')
myAnimation = animation.FuncAnimation(fig, animate, interval=INTERVAL_ANIMATION)
plt.show()
