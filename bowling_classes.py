#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import random
import math
# set up the classes for bowling pin and bowling ball.

# Used by run_bowling_sim.py
# Please overwrite the calc_collisions function. It currently does some jank wizardry with uniform random variables. Also the bowling_ball class and bowling pin_class share a bunch of parameters and methods. May be able to create some sort of abstract class and inherit from it, if python allows it.
# also, we are doing dumb stuff here with linear and angular velocities. We assume that all collisions (generated in calc_collision) result in changes solely in linear velocity. When we update the linear velocity of an object, we subtract a random fraction of the energy and convert it to random rotation in 2 directions, split by a uniform random fraction.


# =====Big List of Assumptions We Make===========
# Assume that the center of mass of bowling pins stay in 2 dimensions.
# Assume that mass of bowling ball is uniformly distributed
# Assume all collisions are perfectly elastic
# Assume that bowling pins phase through everything between time steps and 
#   angle and velocity are updated linearly during those time steps
# Assume that upon collision between pins, kinetic energy is randomly distributed between
#   pins by a uniform distribution. Linear and rotational energies are split randomly by uniform distribution. Rotational components are split by uniform distribution between phi and theta.
# Assume that upon collision between a bowling ball and a pin, up to 50% of the ball's kinetic energy is transferred to the pin, with a random uniform distribution.
# Assume that the bowling ball has not rotation, but can instead follow a curved path.
# Assumethat bowling pins do not spin about their major axis.
# Assume that there is no friction/negligible
# Assume that the lane is flat.
# Assume gutter balls can be neglected


class bowling_pin:
    # costructor. values in meters, kg, seconds
    def __init__(self, diam_max, height, pos, is_collidable):
        self.mass = 1.55
        self.diam = diam_max
        self.height = height
        # Andrew Daniel Kickertz used CAD to find the inertia of a standard bowling pin.
        # https://search.proquest.com/docview/1221594386?pq-origsite=gscholar&fromopenview=true
        self.I = 0.013428 #kg·m2
        # x and y position of the pin com
        self.pos = pos
        # velocity
        self.vel = [0,0]
        # # linear momentum components
        # self.rho = [0,0]
        # assume that the center of mass of a pin is 1/3 the height
        self.com_dist = height/3
        # angles [phi, theta]. phi is tilting in the direction of the lane with positive towards the back. theta is tilt perpendiculat to the lane. clkwise is positive.
        self.ang = [0,0]
        # angular vel in phi and theta
        self.ang_vel = [0,0]
        # total kinetoc energy of the pin. Joules
        self.KE = 0
        # is the pin a valid collision object? Flag or out of bounds
        self.is_collidable = is_collidable
        
        # each pin has collision region that is shaped like a gourd.
        # distance of collision spheres relative to the center of mass along major axis
        self.collision_rel_dist = [0, 0.5*self.height]
        # the radi of the spheres, respectively are
        # self.collision_radi = [self.height/3, self.height/6]
        self.collision_radi = [self.height/3, self.height/6]
        # position of the collision radi in the world frame
        self.collision_wf = np.array([self.pos, self.pos])
        # set a decay constant, which dampens motion.
        self.dampen = 0.025
    def update_KE(self):
        # sum liner and angular energies.
        self.KE = np.sum(self.mass*np.power(self.vel,2))/2 + np.sum(self.I*np.power(self.ang,2))
    # takes a new velocity vector. We take a random fraction of that linear energy uniformly [0,1] and convert it to angular energy. Randomly split into phi and theta components and convert to angular velocity.
    def update_collision(self, new_lin_vel):
        self.KE = np.sum(self.mass*np.power(new_lin_vel,2))/2
        rand = np.random.rand(4)
        self.vel = new_lin_vel*np.power(rand[0],1/2)
        ang_KE_tot = self.KE * (1-rand[0])
        ang_KE_comp = np.array([ang_KE_tot * rand[1], ang_KE_tot * (1-rand[1])])
        self.ang_vel = np.power(2*ang_KE_comp/self.I,1/2)
        if rand[2] > 0.5:
            self.ang_vel[0] *= -1
        if rand[3] > 0.5:
            self.ang_vel[1] *= -1
        self.update_KE()

    # update the linear and angular positions. Also update position of collision circles.
    # the angles are in "3d" space we will project the pin onto the plane of the lane
    # Assume that the size of the collision spheres are constant. Only distance is squeezed.
    def update_position(self,dt):
        self.pos = self.pos + np.array(self.vel)*dt
        self.ang = (self.ang + np.array(self.ang_vel)*dt) % math.pi
        vec_long = np.sin(self.ang)
        self.collision_wf=np.array([self.pos + vec_long * self.collision_rel_dist[0], self.pos + vec_long * self.collision_rel_dist[1]])

        self.ang_vel[0] *= (1-self.dampen)
        self.ang_vel[1] *= (1-self.dampen)
        self.vel[0] *= (1-self.dampen)
        self.vel[1] *= (1-self.dampen)

        self.update_KE

class bowling_ball:
    # costructor. values in meters, kg, seconds
    # spin is in radians and ctrclkwise as viewed from above is positive
    def __init__(self, mass, diam, init_vel, spin):
        self.mass = mass
        self.diam = diam
        self.pos = [0, 0]
        # velocity
        self.vel = init_vel
        self.KE = 0
        # spin will cause the ball to update velocity components by a certain angle every timestep.
        self.spin = spin
        # is the pin a valid collision object? Always true for the ball
        self.is_collidable = True

    def update_KE(self):
        # sum liner and angular energies.
        self.KE = np.sum(self.mass*np.power(self.vel,2))/2
    # takes a new velocity vector. We take a random fraction of that linear energy uniformly [0,1] and convert it to angular energy. Randomly split into phi and theta components and convert to angular velocity.
    def update_collision(self, new_lin_vel, energy_frac):
        self.vel = self.vel*(1-energy_frac) + new_lin_vel * energy_frac
        self.update_KE()

    # update the linear and angular positions. Also update position of collision circles.
    # the angles are in "3d" space we will project the pin onto the plane of the lane
    # Assume that the size of the collision spheres are constant. Only distance is squeezed.
    def update_position(self,dt):
        self.pos = self.pos + np.array([self.vel[0]*np.sin(self.spin), self.vel[1]*np.cos(self.spin)])*dt
        self.update_KE()

# what happens in the case of a collision between multiple objects.
# mass is [mass_ball, mass_pin].
# we assume all objects are pins.
# bowling_obj is a list of balls and/or pins in collision
#
# We do a lot of non-physics. Basically bs-ing using uniform distribution
# which we justify as a result of bowling pin shape being oblong and 
# not simple. Therefore, there are occasions when a pin will just go wild 
# in a random direction.

# [Overload this function]. take inputs of [bowling_ball, pins1, 2, ...], [indicies of objects to check collision], [mass_ball, mass_pin]
# Returns nothing. Calls the update_collision method of the collided objects, which updates velocities, angular velocities, and KineticEnergy. We don't currently use Kinetic energy in a meaningful way because I am big dumb in physics and don't feel like thinking about it.
# this function can take more than 2 indicies to collide, but there is really no point.

# calculates a single collision involving bowling_objects in a bowling_objects numpy array denoted by index values in a list of indicies.
def calc_collision(bowling_obj,indicies, mass):
    # number of objects in collision
    num_objects = len(indicies)
    tot_energy = 0
    ball_in_collision = False
    ball_idx = -1
    avg_vel = 0
    # fraction of the mass of the ball participating in the collision. max 20%
    ball_energy_frac = np.random.rand(1)/50
    for idx in indicies:
        if isinstance(bowling_obj[idx], bowling_ball):
            ball_in_collision = True
            tot_energy = tot_energy + bowling_obj[idx].KE#*ball_energy_frac
            ball_idx = 0
        else:
            tot_energy = tot_energy + bowling_obj[idx].KE
        avg_vel += np.sqrt(np.sum(np.square(bowling_obj[idx].vel)))
    # generate y components
    components = np.random.rand(len(indicies),2) * avg_vel
    # print(components)
    # print(indicies)
    if ball_in_collision:
        components[ball_idx,:] = components[ball_idx,:] * mass[0]*ball_energy_frac/mass[1]
        norm_components = components - np.sum(components,axis=0)/num_objects
        # print(norm_components)
        for i in range(len(indicies)):
            if indicies[i] == ball_idx:
                bowling_obj[indicies[i]].update_collision(norm_components[i,:], ball_energy_frac)
            else:
                bowling_obj[indicies[i]].update_collision(norm_components[i,:])
    else:
        norm_components = components - np.sum(components,axis=0)/num_objects 
        for i in range(len(indicies)):
            bowling_obj[indicies[i]].update_collision(norm_components[i,:])

# pins is a list with indicies of pins to spawn.
# ball_dist is how far in front of the ball to spawn the pins.
# angle: right is positive.
# speed: how fast the ball is going.
# ball_mass, ball_diam, ball_vel, ball_spin are ball parameters
# pin_diam, pin_height are pin parameters
def init_lane(pins, ball_dist, ball_mass, ball_diam, ball_vel, ball_spin, pin_diam, pin_height):
    bowling_obj = []
    bowling_obj.append(bowling_ball(ball_mass,ball_diam,ball_vel,ball_spin))
    # distance between each pin in meters
    ps = 12*0.0254
    # distance between rows of pins
    rd = ps * np.sqrt(3)/2
    pin_pos = np.array([[0,0], [-ps/2,rd],[ps/2, rd], [-ps, 2*rd],[0, 2*rd],[ps, 2*rd], [-3/2*ps, 3*rd],[-1/2*ps, 3*rd],[1/2*ps, 3*rd],[3/2*ps, 3*rd]])
    pin_pos[:,1] = pin_pos[:,1] + ball_dist
    for i in range(10):
        if i in pins:
            bowling_obj.append(bowling_pin(pin_diam,pin_height,pin_pos[i], is_collidable = True))
        else:
            bowling_obj.append(bowling_pin(pin_diam,pin_height,pin_pos[i], is_collidable = False))
    return np.array(bowling_obj), np.array(pin_pos)

# function for detecting collisions between pins.
# use L2 distance
def distance(pos1, pos2):
    return np.abs(np.sqrt(np.sum(np.square(pos1-pos2))))

def pairwise_collision_check(obj1, obj2):
    if (not obj1.is_collidable) | (not obj2.is_collidable):
        return False
    if isinstance(obj1, bowling_ball):
        if (distance(obj1.pos, obj2.collision_wf[0,:]) <= (obj1.diam/2 + obj2.collision_radi[0])) | (distance(obj1.pos, obj2.collision_wf[1,:]) <= (obj1.diam/2 + obj2.collision_radi[1])):
            return True
    else:
        for p in [0,1]:
            for q in [0,1]:
                if (distance(obj1.collision_wf[p,:], obj2.collision_wf[q,:]) <= (obj1.collision_radi[p] + obj2.collision_radi[q])):
                    return True
    return False
        

# assume that the ball is always in the first index.
def check_collision(obj):
    ball = obj[0]
    pins = obj[1:]

    collision = []
    for i in range(len(obj)):
        collision.append([])

    for i in np.arange(len(obj)):
        for j in np.arange(i+1,len(obj)):
            if pairwise_collision_check(obj[i], obj[j]):
                # print(str(i) +' '+ str(j))
                mass = [obj[0].mass, 1.55]
                calc_collision(obj,[i,j], mass)
    #             print(str(i)+str(j))
    #             collision[i].append(j)
    #             collision[j].append(i)
    #             # We assume that if a collides with b and b collides with c, then a also collides with c.
    #             # collision is a list of lists that tracks what objects each object is in collision with.
    #             for lst in collision:
    #                 if ((i in lst) &  (not (j in lst))):
    #                     lst.append(j)
    #                 if ((not (i in lst)) & (j in lst)):
    #                     lst.append(i)
    # # using list comprehension 
    # # to remove duplicated  
    # # from list  
    # collisions_no_duplicates = [] 
    # [collisions_no_duplicates.append(x) for x in collision if x not in collisions_no_duplicates] 
    # print(collisions_no_duplicates)
    # mass = [obj[0].mass, 1.55]
    # for col in collisions_no_duplicates:
    #     calc_collision(obj,col, mass)
    

