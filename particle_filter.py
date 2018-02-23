from grid import *
import copy
import math
from particle import Particle
from utils import *
from setting import *
import random

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    dx, dy, dh = odom
    motion_particles = []
    # Move the particles
    for (px, py, ph) in [p.xyh for p in particles]:
        #px, py, ph = add_odometry_noise(?,?,?)
        new_dx, new_dy = rotate_point(dx, dy, ph + dh)
        moved_particle = Particle(px + new_dx, py + new_dy, (ph + dh) % 360)
        motion_particles.append(moved_particle)

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    observed_markers = measured_marker_list
    measured_particles = []
    weights = []

    if observed_markers:
        for p in particles:
            particle_prob = 1
            used_observed_markers = []
            for mx, my, mh in p.read_markers(grid):
                # find best matching markers of observation for current marker
                best_marker_match_index = -1
                best_marker_match_prob = 0
                for i, (ox, oy, oh) in enumerate(observed_markers):
                    px, py = rotate_point(mx, my, oh) # - mh
                    dist_diff = grid_distance(ox, oy, px, py)
                    angle_diff = diff_heading_deg(mh, oh)
                    exponent = float(dist_diff ** 2) / (2 * MARKER_TRANS_SIGMA ** 2) \
                            - float(angle_diff ** 2) / (2 * MARKER_ROT_SIGMA ** 2)
                    marker_observation_prob = math.exp(-1 * exponent)

                    # check if this observation gives the optimal matching
                    if marker_observation_prob > best_marker_match_prob:
                        best_marker_match_index = i
                        best_marker_match_prob = marker_observation_prob

                # Check we have found a marker match
                if best_marker_match_index >= 0:
                    # use highest found probability
                    particle_prob *= best_marker_match_prob

                    # move the used marker to the used_markers list
                    used_marker = observed_markers.pop(best_marker_match_index)
                    used_observed_markers.append(used_marker)
            weights.append(0 if particle_prob == 1 else particle_prob)
            observed_markers.extend(used_observed_markers)
        print(str(len(particles)) + ' particles')
        print('Left weights creation loop')
    else:
        weights = [1.0 / PARTICLE_COUNT] * PARTICLE_COUNT

    # Normalization step
    ## TODO: 
    #total_weight becoming 0. 
    #Thus weights is all 0s. 
    #Thus particle_probs are all 0s
   
    # If all particles end up with probability 0, reset
    total_weight = sum(weights)
    prob_dist = [float(prob) / total_weight for prob in weights]

    print("%1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f " % tuple(weights[:10]))
    print("%1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f %1.8f " % tuple(prob_dist[:10]))

    # Resampling step
    for i, prob in enumerate(prob_dist):
        for _ in range(int(prob * PARTICLE_COUNT)):
            measured_particles.append(copy.deepcopy(particles[i]))
    print('Left primary resampling loop')

    # Random Resample
    count = 0
    while len(measured_particles) < PARTICLE_COUNT:
        x, y = grid.random_free_place()
        measured_particles.append(Particle(x, y))
        count += 1
    print('Random resampled ' + str(count) + ' particles')

    #return measured_particles
    return measured_particles

################################################
'''
    for (rx, ry, rh) in measured_marker_list:
        del_goal_x_0, del_goal_y_0 = rotate_point(rx, ry, rh)
        del_goal_x_1 = 1 - del_goal_x_0
        del_goal_y_1 = 1 - del_goal_y_0


    # take list of markers, find marker positions using grid.
        measured_particles = []
        weights = []

        # create list of weights
        for (px, py, ph) in [p.xyh for p in particles]:
            dist = min(abs(del_goal_y_0 - py),
                       abs(del_goal_x_0 - px),
                       abs(del_goal_y_1 - py),
                       abs(del_goal_x_1 - px))
            weight = 1.0 / dist
            weights.append(weight)

        # normalize to get prob distribution
        total_weight = sum(weights)
        prob_dist = [prob / total_weight for prob in weights]


'''

