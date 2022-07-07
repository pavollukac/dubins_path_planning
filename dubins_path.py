import imp
from math import tan, atan2, acos, pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from car import SimpleCar
from comps.environment import Environment
from test_cases.cases import TestCase
from utils.utils import transform, directional_theta, plot_a_car, distance

from time import time

from line_cross import *


class Params:
    """ Store parameters for different dubins paths. """

    def __init__(self, d):

        self.d = d      # dubins type
        self.t1 = None  # first tangent point
        self.t2 = None  # second tangent point
        self.c1 = None  # first center point
        self.c2 = None  # second center point
        self.len = None # total travel distance
        self.p0 = None  # start position
        self.p1 = None  # end position


class DubinsPath:
    """
    Consider four dubins paths
    - LSL
    - LSR
    - RSL
    - RSR
    and find the shortest obstacle-free one.
    """

    def __init__(self, car):

        self.car = car
        self.r = self.car.l / tan(self.car.max_phi)
        
        
        # turn left: 1, turn right: -1
        self.direction = {
            'LSL': [1, 1],
            'LSR': [1, -1],
            'RSL': [-1, 1],
            'RSR': [-1, -1]
        }
    
    def find_tangents(self, start_pos, end_pos):
        """ Find the tangents of four dubins paths. """

        # self.start_pos = start_pos
        # self.end_pos = end_pos

        # x1, y1, theta1 = start_pos
        # x2, y2, theta2 = end_pos
        
        # self.s = np.array(start_pos[:2])
        # self.e = np.array(end_pos[:2])
        
        # self.lc1 = transform(x1, y1, 0, self.r, theta1, 1)
        # self.rc1 = transform(x1, y1, 0, self.r, theta1, 2)
        # self.lc2 = transform(x2, y2, 0, self.r, theta2, 1)
        # self.rc2 = transform(x2, y2, 0, self.r, theta2, 2)
        
        # solutions = [self._LSL(), self._LSR(), self._RSL(), self._RSR()]
        # solutions = [s for s in solutions if s is not None]
        # solutions.sort(key=lambda x: x.len, reverse=False)

                
        cross_point=line_cross_section(start_pos,end_pos)

        pos1_points=distr_points(start_pos,cross_point,self.r)
        pos2_points=distr_points(end_pos,cross_point,self.r)

        i1=-1
        i2=-1

        solutions=[]
        while True:
            
            i1=min(i1+1,len(pos1_points)-1)
            i2=min(i2+1,len(pos2_points)-1)

            start_pos=pos1_points[i1]
            end_pos=pos2_points[i2]

            self.start_pos = start_pos
            self.end_pos = end_pos

            x1, y1, theta1 = start_pos
            x2, y2, theta2 = end_pos
            
            self.s = np.array(start_pos[:2])
            self.e = np.array(end_pos[:2])
            
            self.lc1 = transform(x1, y1, 0, self.r, theta1, 1)
            self.rc1 = transform(x1, y1, 0, self.r, theta1, 2)
            self.lc2 = transform(x2, y2, 0, self.r, theta2, 1)
            self.rc2 = transform(x2, y2, 0, self.r, theta2, 2)
            
            sub_solutions = [self._LSL(), self._LSR(), self._RSL(), self._RSR()]
            sub_solutions = [s for s in sub_solutions if s is not None]
            #sub_solutions.sort(key=lambda x: x.len, reverse=False)

            solutions=solutions+sub_solutions

            if   i1==len(pos1_points)-1 and i2==len(pos2_points)-1:
                break


        start_pos=pos1_points[0]
        end_pos=pos2_points[0]

        self.start_pos = start_pos
        self.end_pos = end_pos
            
        return solutions
    
    def get_params(self, dub, c1, c2, t1, t2,start_pos,end_pos):
        """ Calculate the dubins path length. """
        
        v1 = self.s - c1
        v2 = t1     - c1
        v3 = t2     - t1
        v4 = t2     - c2
        v5 = self.e - c2

        delta_theta1 = directional_theta(v1, v2, dub.d[0])
        delta_theta2 = directional_theta(v4, v5, dub.d[1])

        arc1    = abs(delta_theta1*self.r)
        tangent = np.linalg.norm(v3)
        arc2    = abs(delta_theta2*self.r)

        theta = self.start_pos[2] + delta_theta1

        dub.t1 = t1.tolist() + [theta]
        dub.t2 = t2.tolist() + [theta]
        dub.c1 = c1
        dub.c2 = c2
        dub.len = arc1 + tangent + arc2
        dub.p0 = start_pos
        dub.p1 = end_pos
        
        return dub
    
    def _LSL(self):

        lsl = Params(self.direction['LSL'])

        cline = self.lc2 - self.lc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) - acos(0)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta, 1)

        lsl = self.get_params(lsl, self.lc1, self.lc2, t1, t2, self.start_pos, self.end_pos)

        return lsl

    def _LSR(self):

        lsr = Params(self.direction['LSR'])

        cline = self.rc2 - self.lc1
        R = np.linalg.norm(cline) / 2

        if R < self.r:
            return None
        
        theta = atan2(cline[1], cline[0]) - acos(self.r/R)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta+pi, 1)

        lsr = self.get_params(lsr, self.lc1, self.rc2, t1, t2, self.start_pos, self.end_pos)

        return lsr

    def _RSL(self):

        rsl = Params(self.direction['RSL'])

        cline = self.lc2 - self.rc1
        R = np.linalg.norm(cline) / 2

        if R < self.r:
            return None
        
        theta = atan2(cline[1], cline[0]) + acos(self.r/R)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta+pi, 1)

        rsl = self.get_params(rsl, self.rc1, self.lc2, t1, t2, self.start_pos, self.end_pos)

        return rsl

    def _RSR(self):

        rsr = Params(self.direction['RSR'])

        cline = self.rc2 - self.rc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) + acos(0)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta, 1)

        rsr = self.get_params(rsr, self.rc1, self.rc2, t1, t2, self.start_pos, self.end_pos)

        return rsr
    
    def best_tangent(self, solutions):
        """ Get the shortest obstacle-free dubins path. """

        pos_org = self.start_pos
        pos_end = self.end_pos

        if not solutions:
            return None, None, False
        i=0
        for s in solutions:
            i+=1
            pos0 = s.p0
            pos1 = s.p1

            route = self.get_route(s)
            # self.start_pos=pos0
            # self.end_pos=pos1
            # if i==5 :
            #     safe=True
            #     break

            safe = self.is_straight_route_safe(s.t1, s.t2)
            if not safe:
                continue

            safe = self.is_turning_route_safe(pos0, s.t1, s.d[0], s.c1, self.r)
            if not safe:
                continue

            safe = self.is_turning_route_safe(s.t2, pos1, s.d[1], s.c2, self.r)
            if not safe:
                continue

            if safe:
                break

        self.start_pos=pos_org
        self.end_pos=pos_end
        
        return route, s.len, safe
    
    def is_straight_route_safe(self, t1, t2):
        """ Check a straight route is safe. """
        # a straight route is simply a rectangle

        vertex1 = self.car.get_car_bounding(t1)
        vertex2 = self.car.get_car_bounding(t2)

        vertex = [vertex2[0], vertex2[1], vertex1[3], vertex1[2]]

        return self.car.env.rectangle_safe(vertex)
    
    def is_turning_route_safe(self, start_pos, end_pos, d, c, r):
        """ Check if a turning route is safe. """
        # a turning_route is decomposed into:
        #   1. start_pos (checked previously as end_pos)
        #   2. end_pos
        #   3. inner ringsector
        #   4. outer ringsector

        if not self.car.is_pos_safe(end_pos):
            return False
        
        rs_inner, rs_outer = self.construct_ringsectors(start_pos, end_pos, d, c, r)
        
        if not self.car.env.ringsector_safe(rs_inner):
            return False
        
        if not self.car.env.ringsector_safe(rs_outer):
            return False

        return True
    
    def construct_ringsectors(self, start_pos, end_pos, d, c, r):
        """ Construct inner and outer ringsectors of a turning route. """
        
        x, y, theta = start_pos

        delta_theta = end_pos[2] - theta

        p_inner = start_pos[:2]
        id = 1 if d == -1 else 2
        p_outer = transform(x, y, 1.3*self.car.l, 0.4*self.car.l, theta, id)

        r_inner = r - self.car.carw / 2
        r_outer = distance(p_outer, c)

        v_inner = [p_inner[0]-c[0], p_inner[1]-c[1]]
        v_outer = [p_outer[0]-c[0], p_outer[1]-c[1]]

        if d == -1:
            end_inner = atan2(v_inner[1], v_inner[0]) % (2*pi)
            start_inner = (end_inner + delta_theta) % (2*pi)

            end_outer = atan2(v_outer[1], v_outer[0]) % (2*pi)
            start_outer = (end_outer + delta_theta) % (2*pi)
        
        if d == 1:
            start_inner = atan2(v_inner[1], v_inner[0]) % (2*pi)
            end_inner = (start_inner + delta_theta) % (2*pi)

            start_outer = atan2(v_outer[1], v_outer[0]) % (2*pi)
            end_outer = (start_outer + delta_theta) % (2*pi)
        
        rs_inner = [c[0], c[1], r_inner, r, start_inner, end_inner]
        rs_outer = [c[0], c[1], r, r_outer, start_outer, end_outer]

        return rs_inner, rs_outer
    
    def get_route(self, s):
        """ Get the route of dubins path. """

        phi1 = self.car.max_phi if s.d[0] == 1 else -self.car.max_phi
        phi2 = self.car.max_phi if s.d[1] == 1 else -self.car.max_phi

        phil = [0,phi1, 0, phi2,0]
        goal = [s.p0,s.t1, s.t2,s.p1, self.end_pos]
        ml = [1, 1, 1,1,1]
        
        return list(zip(goal, phil, ml))


def main():

    tc = TestCase()

    env = Environment(tc.obs,50,40)

    car = SimpleCar(env, tc.start_pos3, tc.end_pos3)

    dubins = DubinsPath(car)

    t = time()
    solutions = dubins.find_tangents(car.start_pos, car.end_pos)
    route, cost, safe = dubins.best_tangent(solutions)

    print('Total time: {}s'.format(round(time()-t, 3)))

    if not safe:
        print('No valid dubins path!')
        #return
    if True:
        
        path = car.get_path(car.start_pos, route)
        path = path[::5] + [path[-1]]

        carl = []
        for i in range(len(path)):
            carl.append(path[i].model[0])

        end_state = car.get_car_state(car.end_pos)

    tangents = []
    for s in solutions:
         tangents.append([s.t1[:2], s.t2[:2]])
    # s = solutions[4]
    # tangents.append([s.t1[:2], s.t2[:2]])        
    lc = LineCollection(tangents, color='k', linewidth=1)
    
    lcircle1 = plt.Circle(dubins.lc1, dubins.r, fc='None', ec='k')
    rcircle1 = plt.Circle(dubins.rc1, dubins.r, fc='None', ec='k')
    lcircle2 = plt.Circle(dubins.lc2, dubins.r, fc='None', ec='k')
    rcircle2 = plt.Circle(dubins.rc2, dubins.r, fc='None', ec='k')

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=5)
    
    ax.add_collection(lc)
    ax.add_patch(lcircle1)
    ax.add_patch(rcircle1)
    ax.add_patch(lcircle2)
    ax.add_patch(rcircle2)
    
    ax = plot_a_car(ax, end_state.model)

    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _car = PatchCollection([])
    ax.add_collection(_car)
    frames = len(path) + 1
    #frames=100
    def animate(i):

        sub_carl = carl[:min(i+1, len(path))]
        _carl.set_paths(sub_carl[::4])
        _carl.set_color('m')
        _carl.set_alpha(0.1)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)].model)
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_zorder(3)

        return _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=10,
                                  repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
