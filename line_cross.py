from attr import s
import matplotlib.pyplot as plt
import numpy as np

from skspatial.objects import Line

from math import cos, e,sin, sqrt,tan, atan2, acos, pi

def position_to_2points(pos):
    x,y,slope=pos

    x2=x+cos(slope)
    y2=y+sin(slope)

    return [x, y], [x2, y2]

def line_cross_section(pos1, pos2):

    p1=position_to_2points(pos1)
    p2=position_to_2points(pos2)


    # Define the two lines.
    line_1 = Line.from_points(p1[0],p1[1])
    line_2 = Line.from_points(p2[0],p2[1])

    if pos1[2]!=pos2[2]:
        # Compute the intersection point
        intersection_point = line_1.intersect_line(line_2)

        return intersection_point
    pass
    return None 

def points_distance (p0,p1):
    x0,y0,phi0=p0
    x1,y1,phi1=p1
    distance=sqrt((x1-x0)**2+(y1-y0)**2)
    return distance

def distr_points(start_pos, cros_point, radius):
    # split line between two points, by normal distribution
    #   * *  *   *    *     *    *   *  * *

    delta=radius

    x0,y0,phi=start_pos
    x1,y1=cros_point
    end_pos=[x1,y1,phi]

    dist_total=points_distance(start_pos,end_pos)
    dist_mid=dist_total/2

    vector_delta_norm=[x1-x0,y1-y0]
    vector_delta_norm = (vector_delta_norm / np.linalg.norm(vector_delta_norm))
    incr_end=vector_delta_norm*radius*1.1
    incr_start=[0,0,0]
    incr_end=[incr_end[0],incr_end[1],0]

    vector_delta=vector_delta_norm*delta
    delta_incr=[vector_delta[0],vector_delta[1],0]

    

    #mid_pos=[(start_pos[0]+end_pos[0])/2,(start_pos[1]+end_pos[1])/2,(start_pos[2]+end_pos[2])/2]
    mid_pos=[(x + y)/2 for x, y in zip(start_pos, end_pos)]

    distances=[start_pos]
    distnances_end=[start_pos]
    distnances_end.append(end_pos)

    while True:
        incr_start = [x + y for x, y in zip(incr_start, delta_incr)]
        new_val1 = [x + y for x, y in zip(incr_start, distances[-1])] 
        new_val2 = [-x + y for x, y in zip(incr_end, distnances_end[-1])]
        tes_distance=points_distance(start_pos,new_val1) + points_distance(end_pos,new_val2)
        if tes_distance>dist_total:
            break
        
        incr_end = [x + y for x, y in zip(incr_end, delta_incr)]
        distances.append(new_val1)
        distnances_end.append(new_val2)
        

    
    distances.reverse()
    distnances_end.pop(1)
    distances= distnances_end +distances
    distances.pop()
    return distances







