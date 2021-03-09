#!/usr/bin/env python3
import bowling_classes as bc
import matplotlib.pyplot as plt
import numpy as np
import sys

# check which pins are left standing.
# o is all bowling objects, pin list is the list of pins that we set up. pin_init_pos is the initial position of the pins. Tolerance is the lineardistance away from the initial position that we consider a knock down.
def check_pins(o, pins,  pin_init_pos, tolerance = 0.005):
    pin_list = o[1:]
    pins_remaining = []
    for i in pins:
        dist = bc.distance(pin_list[i].pos, pin_init_pos[i])
        if dist <= tolerance:
            pins_remaining.append(i)
    return np.array(pins_remaining)

# plots bowling ball and pins.
def plot_scene(o,dt):
    fig = plt.gcf()
    fig.set_size_inches(5, 8)
    ax = plt.gca()
    ax.cla() # clear things for fresh plot
    ax.set_xlim((-0.65, 0.65))
    ax.set_ylim((-1, 3))
    pins = o[1:]
    ball = o[0]
    # now make a circle with no fill, which is good for hi-lighting key results
    ax.add_patch(plt.Circle((ball.pos[0], ball.pos[1]), ball.diam/2, color='b'))
    ax.plot([o[0].pos[0],o[0].vel[0]], [o[0].pos[1],o[0].vel[1]])
    for obj in pins:
        if obj.is_collidable:
            # ax.add_patch(plt.Circle((obj.pos[0], obj.pos[1]), obj.diam/2, color='g', fill=False))
            ax.add_patch(plt.Circle((obj.collision_wf[0,0], obj.collision_wf[0,1]), obj.collision_radi[0], color='g', fill=False))
            ax.add_patch(plt.Circle((obj.collision_wf[1,0], obj.collision_wf[1,1]), obj.collision_radi[1], color='r', fill=False))
        else:
            ax.add_patch(plt.Circle((obj.collision_wf[0,0], obj.collision_wf[0,1]), obj.collision_radi[0], color='silver', fill=False))
            ax.add_patch(plt.Circle((obj.collision_wf[1,0], obj.collision_wf[1,1]), obj.collision_radi[1], color='silver', fill=False))
        ax.plot(obj.collision_wf[:,0], obj.collision_wf[:,1])
    plt.draw()
    plt.pause(0.1)
    plt.cla()
def forward_sim(o, plt_bool=False):
    dt = 0.01
    time = np.arange(25)
    for i in time:
        for obj in o:
            obj.update_position(dt)
        bc.check_collision(o)
        if plt_bool:
            plot_scene(o,dt)

# set a lane depending on the pins we want standing.
def bowl(pins,plt_bool):
    o,pin_pos = bc.init_lane(pins,1, 10, 0.22, [0,7.5], 0.3, 0.12, 0.381)
    forward_sim(o,plt_bool)
    remain = check_pins(o, pins, pin_pos, tolerance = 0.01)
    return remain

def main(argv):
    try:
        if argv[0] == 'True':
            plt_bool = True
        elif argv[0] == 'False':
            plt_bool = False
        else:
            plt_bool = False
            print('Not plotting. Use argument: "True" to enable plotting')
    except Exception as e:
        print ('./run_bowling_sim.py True/False <indicates show plot or not>')
        sys.exit(2)
    pins_knocked = []
    for frame in range(9):
        print("Frane " + str(frame+1))
        print("================")
        # ball_dist is how far in front of the ball to spawn the pins.
        # angle: right is positive.
        # speed: how fast the ball is going.
        # ball_mass, ball_diam, ball_vel, ball_spin are ball parameters
        # pin_diam, pin_height are pin parameters
        # def init_lane(pins, ball_dist, ball_mass, ball_diam, ball_vel, ball_spin, pin_diam, pin_height):
        # pins = [1,2,3]
        pins = range(10)
        remain = bowl(pins,plt_bool)
        pins_knocked.append(len(remain))
        print(remain)
        if len(remain) == 0:
            pins_knocked.append(0)
            print(0)
        else:
            remain = bowl(remain,plt_bool)
            pins_knocked.append(len(remain))
            print(remain)
    # frame 10 shenanigans
    print("Frane " + str(10))
    print("================")
    remain = bowl(range(10),plt_bool)
    pins_knocked.append(len(remain))
    print(remain)
    if len(remain) == 0:
        remain = bowl(range(10),plt_bool)
        pins_knocked.append(len(remain))
        print(remain)
        if len(remain) == 0:
            remain = bowl(range(10),plt_bool)
            pins_knocked.append(len(remain))
            print(remain)
            if len(remain) == 0:
                remain = bowl(range(10),plt_bool)
                pins_knocked.append(len(remain))
                print(remain)
            else:
                remain = bowl(remain,plt_bool)
                pins_knocked.append(len(remain))
                print(remain)
        else:
            remain = bowl(remain,plt_bool)
            pins_knocked.append(len(remain))
            print(remain)
            if len(remain) == 0:
                remain = bowl(range(10),plt_bool)
                pins_knocked.append(len(remain))
                print(remain)
            else:
                remain = bowl(remain,plt_bool)
                pins_knocked.append(len(remain))
                print(remain)
    else:
        remain = bowl(remain,plt_bool)
        pins_knocked.append(len(remain))
        print(remain)
        if len(remain) == 0:
            remain = bowl(range(10),plt_bool)
            pins_knocked.append(len(remain))
            print(remain)
        else:
            remain = bowl(remain,plt_bool)
            pins_knocked.append(0)
            print(0)
    print(pins_knocked)
    return(pins_knocked)
if __name__ == "__main__":
    main(sys.argv[1:])