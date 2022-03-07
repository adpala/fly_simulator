import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

def init_random(myclass,n,vel_scale=2,WORLD_SIZE=20,DELTA_T=1):
    xs = (WORLD_SIZE*np.random.random(n)-WORLD_SIZE/2)/2
    ys = (WORLD_SIZE*np.random.random(n)-WORLD_SIZE/2)/2
    vxs = (np.random.random(n)-0.5)*vel_scale
    vys = (np.random.random(n)-0.5)*vel_scale
    angles = (np.random.random(n)-0.5)*2*np.pi
    flies = []
    for x, y, vx, vy, angle in zip(xs,ys,vxs,vys,angles):
        flies.append(myclass({'position': (x,y), 'velocity': (vx,vy), 'angle':angle, 'WORLD_SIZE':WORLD_SIZE, 'DELTA_T':DELTA_T}))
    return flies

class Fly():

    def __init__(self, params) -> None:

        self.defaultParameters = {
            'identity': 'male',
            'position': (0,0),
            'velocity': (0,0),
            'angle': 0,
            'rotation_speed': 0,
            'perception_radius': 10,
            'max_trans_speed': 5,
            'max_rotation_speed': 5,
            'random_move_mag': 10,
            'collision_radius': 2,
            'color': 'r',
            'rotation_constant': 10,
            'iframe': 0,
            'target_preference': 'female',
            'friction_constant': 0.1,
            'WORLD_SIZE': 20,
            'DELTA_T': 1,
            'WALL_MARGIN': 2
        }

        self.parseParameters(params)

        self.position = np.asarray(self.position,dtype='float32')
        self.velocity = np.asarray(self.velocity,dtype='float32')

    def parseParameters(self, parameters, verbose=False):
        """unpacks dict of parameters, checks if they are allowed, and fills with defaults if a required parameter is not given.

        Args:
            parameters ([type]): [description]
        """

        self.parsedKeys = []  # store parsed keys to print them if needed

        # unpack parameters and assign as attributes if allowed
        for name, value in parameters.items():
            setattr(self, name, value)
            self.parsedKeys.append(name)
            if verbose:
                print(f"unpacking {name}")

        # make defaults of missing parameters
        for name, value in self.defaultParameters.items():
            if name not in parameters.keys():
                if verbose:
                    print(f"setting {name} to default")
                setattr(self, name, value)
                self.parsedKeys.append(name)

        # # assert that required parameters are given
        # for name in self.requiredKeys:
        #     assert name in parameters.keys(), f"{name} parameter missing"

    @property
    def heading(self):
        """Unit vector from ANGLE"""
        return np.array([np.cos(self.angle), np.sin(self.angle)])

    @staticmethod
    def angle_to_heading(angle):
        """Unit vector from ANGLE."""
        return np.array([np.cos(angle),np.sin(angle)])
    
    @staticmethod
    def heading_to_angle(vector):
        """Angle from unit vector."""
        return np.arctan2(*vector[::-1])

    @staticmethod
    def wrap_angle(angle):
        if np.abs(angle) >= 2*np.pi:
            angle -= np.sign(angle)*2*np.pi
        return angle

    def get_friction(self):
        return -self.friction_constant*self.velocity

    def update(self,targets=[]):
        pass

    def get_distance(self,target):
        '''for single target'''
        return np.linalg.norm(target.position - self.position)

    @staticmethod
    def unit_vector(v):
        mag = np.linalg.norm(v)
        if mag > 0:
            return v / np.linalg.norm(v)
        else:
            return np.zeros_like(v)

    @staticmethod
    def limit_vector(vector,limit_mag):
        mag = np.linalg.norm(vector)
        if mag > 0:
            vector *= min(limit_mag, mag)/mag
        return vector

    def bound_collision(self):
        r = np.linalg.norm(self.position)
        if r > (self.WORLD_SIZE-self.WALL_MARGIN):
            # perpendicular
            perpendicular_ang = self.heading_to_angle(self.position)
            perpendicular_vel = np.dot(self.unit_vector(self.position),self.velocity)
            if perpendicular_vel > 0:
                perpendicular_vel = -perpendicular_vel
            # parallel
            parallel_ang = perpendicular_ang-np.pi/2
            parallel_vel = np.linalg.norm(self.velocity)*np.cos(parallel_ang)

            self.velocity = parallel_vel*self.angle_to_heading(parallel_ang) + perpendicular_vel*self.angle_to_heading(perpendicular_ang)
        return self.velocity

    def random_move(self):
        return self.angle_to_heading(2*np.pi*np.random.random_sample())*self.random_move_mag
        # return (np.random.random(2)-0.5)*self.random_move_mag

    def seek(self,target):
        target_direction = self.angle_to_heading(self.heading_to_angle(target.position - self.position))
        return target_direction*self.max_trans_speed - self.velocity

    def collisions(self,targets=[]):
        collision_force = np.array([0,0],dtype='float32')
        for target in targets:
            distance = self.get_distance(target)
            if distance < self.collision_radius:
                target_relative_position = target.position - self.position
                collision_force += -20*self.limit_vector(target_relative_position, 1)/(distance**2)
        return collision_force

    def perceive_targets(self,targets=[]):
        if self.target_preference == 'all':
            return [target for target in targets if self.get_distance(target) < self.perception_radius]
        elif self.target_preference != 'none':
            return [target for target in targets if (self.get_distance(target) < self.perception_radius) and (target.identity == self.target_preference)]
        else:
            return []


    def seek_closest(self,targets=[]):
        seek_force = np.array([0,0])
        distances = [self.get_distance(target) for target in targets if self.get_distance(target)]
        if len(distances) > 0:
            seek_force = self.seek(targets[np.argmin(distances)])
        return seek_force

    def steering(self,targets=[]):
        '''matching velocity to flockmates'''
        align_velocity = np.array([0,0],dtype='float32')
        if len(targets) > 0:
            align_velocity = np.mean([target.velocity for target in targets], axis = 0)
        return self.limit_vector(align_velocity - self.velocity, self.max_steering_force)

    def plot(self,ax=None,show_perception=False,show_collision=False,show_velocity=False):
        if not ax:
            ax = plt.gca()
        ax.plot(*self.position, f'o{self.color}')
        ax.plot([self.position[0], self.position[0]+self.heading[0]], [self.position[1], self.position[1]+self.heading[1]], f'-{self.color}')
        if show_velocity:
            ax.plot([self.position[0], self.position[0]+self.velocity[0]], [self.position[1], self.position[1]+self.velocity[1]], f'-b')
        if show_perception:
            perception_circle = mpl.patches.Circle(self.position, radius=self.perception_radius, fill=False, facecolor=None, edgecolor=f'{self.color}',linestyle='--', lw=0.5)
            ax.add_artist(perception_circle)
        if show_collision:
            collision_circle = mpl.patches.Circle(self.position, radius=self.collision_radius, fill=False, facecolor=None, edgecolor='k',linestyle='--', lw=0.5)
            ax.add_artist(collision_circle)

